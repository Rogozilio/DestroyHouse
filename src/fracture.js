import * as THREE from 'three';
import { DestructibleMesh, FractureOptions } from '../vendor/three-pinata/three-pinata.es.js';
import { fractureNoise } from './noise.js?v=10';

const tmpBox = new THREE.Box3();

export function extractTriangles(root) {
  root.updateWorldMatrix(true, true);
  const triangles = [];
  const bounds = new THREE.Box3();
  const mergedPositions = [];
  const mergedNormals = [];
  const mergedUvs = [];
  let material = null;

  root.traverse((node) => {
    if (!node.isMesh || !node.geometry?.attributes?.position) return;
    const geometry = node.geometry;
    const position = geometry.attributes.position;
    const normal = geometry.attributes.normal;
    const uv = geometry.attributes.uv;
    const index = geometry.index;
    const normalMatrix = new THREE.Matrix3().getNormalMatrix(node.matrixWorld);
    material ||= Array.isArray(node.material) ? node.material[0] : node.material;

    const readVertex = (i) => new THREE.Vector3()
      .fromBufferAttribute(position, i)
      .applyMatrix4(node.matrixWorld);
    const readNormal = (i, fallback) => normal
      ? new THREE.Vector3().fromBufferAttribute(normal, i).applyMatrix3(normalMatrix).normalize()
      : fallback.clone();
    const readUv = (i) => uv
      ? new THREE.Vector2().fromBufferAttribute(uv, i)
      : new THREE.Vector2();

    const triCount = index ? index.count / 3 : position.count / 3;
    for (let i = 0; i < triCount; i++) {
      const ia = index ? index.getX(i * 3) : i * 3;
      const ib = index ? index.getX(i * 3 + 1) : i * 3 + 1;
      const ic = index ? index.getX(i * 3 + 2) : i * 3 + 2;
      const a = readVertex(ia);
      const b = readVertex(ib);
      const c = readVertex(ic);
      const centroid = new THREE.Vector3().addVectors(a, b).add(c).multiplyScalar(1 / 3);
      const faceNormal = new THREE.Vector3()
        .subVectors(b, a)
        .cross(new THREE.Vector3().subVectors(c, a))
        .normalize();
      for (const [vertexIndex, vertex] of [[ia, a], [ib, b], [ic, c]]) {
        const vertexNormal = readNormal(vertexIndex, faceNormal);
        const vertexUv = readUv(vertexIndex);
        mergedPositions.push(vertex.x, vertex.y, vertex.z);
        mergedNormals.push(vertexNormal.x, vertexNormal.y, vertexNormal.z);
        mergedUvs.push(vertexUv.x, vertexUv.y);
      }
      triangles.push({ a, b, c, centroid });
      bounds.expandByPoint(a).expandByPoint(b).expandByPoint(c);
    }
  });

  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.Float32BufferAttribute(mergedPositions, 3));
  geometry.setAttribute('normal', new THREE.Float32BufferAttribute(mergedNormals, 3));
  geometry.setAttribute('uv', new THREE.Float32BufferAttribute(mergedUvs, 2));
  geometry.computeBoundingBox();
  geometry.computeBoundingSphere();

  return { triangles, bounds, material, geometry };
}

// A copy of the source shrunk inward by `inset` world units, used to fracture
// the structural core that sits just inside the surface shell.
export function insetSource(source, inset) {
  const center = source.bounds.getCenter(new THREE.Vector3());
  const size = source.bounds.getSize(new THREE.Vector3());
  const scale = new THREE.Vector3(
    Math.max(0.1, (size.x - 2 * inset) / Math.max(size.x, 1e-4)),
    Math.max(0.1, (size.y - 2 * inset) / Math.max(size.y, 1e-4)),
    Math.max(0.1, (size.z - 2 * inset) / Math.max(size.z, 1e-4)),
  );
  const geometry = source.geometry.clone();
  const pos = geometry.attributes.position;
  for (let i = 0; i < pos.count; i++) {
    pos.setX(i, center.x + (pos.getX(i) - center.x) * scale.x);
    pos.setY(i, center.y + (pos.getY(i) - center.y) * scale.y);
    pos.setZ(i, center.z + (pos.getZ(i) - center.z) * scale.z);
  }
  pos.needsUpdate = true;
  geometry.computeBoundingBox();
  geometry.computeBoundingSphere();
  return {
    triangles: source.triangles,
    bounds: geometry.boundingBox.clone(),
    material: source.material,
    geometry,
  };
}

export function createFracture(source, options) {
  const { triangles, bounds, geometry } = source;
  if (triangles.length < 8 || bounds.isEmpty() || !geometry?.attributes?.position) {
    return createProxyFracture(source.bounds, options);
  }

  const fragmentCount = THREE.MathUtils.clamp(options.shards | 0, 8, 160);
  const destructible = new DestructibleMesh(geometry);
  destructible.updateMatrixWorld(true);
  const modeSeed = {
    voronoi: 0,
    ridge: 1009,
    bands: 2027,
    turbulence: 4051,
  }[options.noise] ?? 0;
  let fractureOptions;

  if (options.noise === 'voronoi') {
    fractureOptions = new FractureOptions({
      fractureMethod: 'voronoi',
      fragmentCount,
      seed: options.seed + modeSeed,
      textureScale: new THREE.Vector2(0.8, 0.8),
      voronoiOptions: {
        mode: '3D',
        useApproximation: fragmentCount > 32,
        approximationNeighborCount: Math.min(24, Math.max(18, Math.round(Math.sqrt(fragmentCount) * 2.5))),
      },
    });
  } else {
    const fracturePlanes = options.noise === 'bands'
      ? { x: false, y: true, z: false }
      : options.noise === 'ridge'
        ? { x: true, y: false, z: true }
        : { x: true, y: true, z: true };
    fractureOptions = new FractureOptions({
      fractureMethod: 'simple',
      fragmentCount,
      fracturePlanes,
      seed: options.seed + modeSeed,
      textureScale: new THREE.Vector2(0.8, 0.8),
    });
  }

  const fragments = destructible.fracture(fractureOptions);
  const shards = fragments
    .map((fragment, index) => fragmentToShard(fragment, index, options.density))
    .filter((shard) => shard.geometry.index?.count >= 12);

  if (shards.length < 4) return createProxyFracture(bounds, options);
  return {
    shards,
    bounds,
    mode: options.noise === 'voronoi' ? 'volumetric 3D Voronoi' : `volumetric ${options.noise}`,
  };
}

function fragmentToShard(fragment, index, density) {
  const geometry = fragment.geometry;
  geometry.computeBoundingBox();
  geometry.computeBoundingSphere();
  const localBounds = geometry.boundingBox.clone();
  const center = fragment.position.clone();
  const bounds = localBounds.clone().translate(center);
  const half = localBounds.getSize(new THREE.Vector3())
    .multiplyScalar(0.5)
    .max(new THREE.Vector3(0.025, 0.025, 0.025));
  return {
    index,
    geometry,
    center,
    half,
    bounds,
    seed: center.clone(),
    mass: Math.max(0.15, computeGeometryVolume(geometry) * Math.max(0.1, density) * 8),
  };
}

function computeGeometryVolume(geometry) {
  const position = geometry.attributes.position;
  const index = geometry.index;
  let volume = 0;
  const a = new THREE.Vector3();
  const b = new THREE.Vector3();
  const c = new THREE.Vector3();
  const cross = new THREE.Vector3();
  const count = index ? index.count : position.count;
  for (let i = 0; i < count; i += 3) {
    const ia = index ? index.getX(i) : i;
    const ib = index ? index.getX(i + 1) : i + 1;
    const ic = index ? index.getX(i + 2) : i + 2;
    a.fromBufferAttribute(position, ia);
    b.fromBufferAttribute(position, ib);
    c.fromBufferAttribute(position, ic);
    volume += a.dot(cross.crossVectors(b, c)) / 6;
  }
  return Math.abs(volume);
}

function createProxyFracture(bounds, options) {
  const fallbackBounds = bounds?.isEmpty?.() === false ? bounds.clone() : new THREE.Box3(
    new THREE.Vector3(-1.5, 0, -1.5),
    new THREE.Vector3(1.5, 2.2, 1.5),
  );
  const size = fallbackBounds.getSize(new THREE.Vector3());
  const grid = Math.ceil(Math.cbrt(options.shards));
  const shards = [];
  let index = 0;

  for (let x = 0; x < grid; x++) {
    for (let y = 0; y < grid; y++) {
      for (let z = 0; z < grid; z++) {
        if (index >= options.shards) break;
        const center = new THREE.Vector3(
          fallbackBounds.min.x + (x + 0.5) * size.x / grid,
          fallbackBounds.min.y + (y + 0.5) * size.y / grid,
          fallbackBounds.min.z + (z + 0.5) * size.z / grid,
        );
        const n = fractureNoise(options.noise, center.clone().multiplyScalar(0.75), options.seed + index);
        const half = new THREE.Vector3(size.x, size.y, size.z).multiplyScalar(0.42 / grid + n * 0.025);
        const geometry = makeNoisyBox(half, index, options);
        tmpBox.setFromCenterAndSize(center, half.clone().multiplyScalar(2));
        shards.push({ index, geometry, center, half, bounds: tmpBox.clone(), seed: center.clone(), mass: half.length() * options.density });
        index++;
      }
    }
  }

  return { shards, bounds: fallbackBounds, mode: 'proxy' };
}

function makeNoisyBox(half, index, options) {
  const geometry = new THREE.BoxGeometry(half.x * 2, half.y * 2, half.z * 2, 1, 1, 1);
  const pos = geometry.attributes.position;
  const color = new THREE.Color().setHSL(0.045 + (index % 9) * 0.028, 0.62, 0.56);
  const colors = [];

  for (let i = 0; i < pos.count; i++) {
    const x = pos.getX(i);
    const y = pos.getY(i);
    const z = pos.getZ(i);
    const n = fractureNoise(options.noise, { x, y, z }, options.seed + index + i);
    const scale = 1 + (n - 0.5) * 0.24;
    pos.setXYZ(i, x * scale, y * scale, z * scale);
    colors.push(color.r, color.g, color.b);
  }

  geometry.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3));
  geometry.computeVertexNormals();
  return geometry;
}

export function findNeighborPairs(shards, bounds, maxLinks) {
  const pairs = [];
  const diag = bounds.getSize(new THREE.Vector3()).length();
  const search = Math.max(0.06, diag / Math.sqrt(Math.max(shards.length, 1)) * 2.4);

  for (let i = 0; i < shards.length; i++) {
    const expanded = shards[i].bounds.clone().expandByScalar(search);
    const candidates = [];
    for (let j = i + 1; j < shards.length; j++) {
      if (!expanded.intersectsBox(shards[j].bounds)) continue;
      const d = shards[i].center.distanceTo(shards[j].center);
      candidates.push({ a: i, b: j, distance: d });
    }
    candidates.sort((a, b) => a.distance - b.distance);
    pairs.push(...candidates.slice(0, 5));
  }

  pairs.sort((a, b) => a.distance - b.distance);
  return pairs.slice(0, maxLinks);
}
