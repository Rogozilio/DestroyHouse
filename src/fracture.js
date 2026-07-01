import * as THREE from 'three';
import { fractureNoise, hash3 } from './noise.js';

const tmpBox = new THREE.Box3();
const tmpVec = new THREE.Vector3();

export function extractTriangles(root) {
  root.updateWorldMatrix(true, true);
  const triangles = [];
  const bounds = new THREE.Box3();
  let material = null;

  root.traverse((node) => {
    if (!node.isMesh || !node.geometry?.attributes?.position) return;
    const geometry = node.geometry;
    const position = geometry.attributes.position;
    const index = geometry.index;
    material ||= Array.isArray(node.material) ? node.material[0] : node.material;

    const readVertex = (i) => new THREE.Vector3()
      .fromBufferAttribute(position, i)
      .applyMatrix4(node.matrixWorld);

    const triCount = index ? index.count / 3 : position.count / 3;
    for (let i = 0; i < triCount; i++) {
      const ia = index ? index.getX(i * 3) : i * 3;
      const ib = index ? index.getX(i * 3 + 1) : i * 3 + 1;
      const ic = index ? index.getX(i * 3 + 2) : i * 3 + 2;
      const a = readVertex(ia);
      const b = readVertex(ib);
      const c = readVertex(ic);
      const centroid = new THREE.Vector3().addVectors(a, b).add(c).multiplyScalar(1 / 3);
      triangles.push({ a, b, c, centroid });
      bounds.expandByPoint(a).expandByPoint(b).expandByPoint(c);
    }
  });

  return { triangles, bounds, material };
}

export function createFracture(source, options) {
  const { triangles, bounds } = source;
  if (triangles.length < 8 || bounds.isEmpty()) {
    return createProxyFracture(source.bounds, options);
  }

  const seedCount = Math.max(4, options.shards | 0);
  const detailedTriangles = tessellateTriangles(triangles, seedCount);
  const seeds = makeSeeds(detailedTriangles, bounds, seedCount, options);
  const buckets = Array.from({ length: seeds.length }, () => []);
  const size = bounds.getSize(new THREE.Vector3());
  const center = bounds.getCenter(new THREE.Vector3());
  const inv = new THREE.Vector3(1 / Math.max(size.x, 0.001), 1 / Math.max(size.y, 0.001), 1 / Math.max(size.z, 0.001));
  const cellScaleSq = size.lengthSq() / Math.max(seedCount, 1);

  for (const tri of detailedTriangles) {
    const p = tri.centroid;
    const normalized = tmpVec.copy(p).sub(center).multiply(inv);
    let best = 0;
    let bestScore = Infinity;

    for (let i = 0; i < seeds.length; i++) {
      const seed = seeds[i];
      const n = fractureNoise(options.noise, normalized, options.seed + i);
      const dx = (p.x - seed.x) * seed.metric.x;
      const dy = (p.y - seed.y) * seed.metric.y;
      const dz = (p.z - seed.z) * seed.metric.z;
      const noiseOffset = (n - 0.5) * options.noiseStrength * cellScaleSq * seed.noiseScale;
      const score = dx * dx + dy * dy + dz * dz + noiseOffset;
      if (score < bestScore) {
        bestScore = score;
        best = i;
      }
    }

    buckets[best].push(tri);
  }

  const shards = buckets
    .map((bucket, index) => bucketToShard(bucket, index, seeds[index], options))
    .filter(Boolean);

  if (shards.length < 4) return createProxyFracture(bounds, options);
  return { shards, bounds, mode: 'detailed mesh' };
}

function tessellateTriangles(triangles, shardCount) {
  const areas = triangles.map(triangleArea);
  const totalArea = areas.reduce((sum, area) => sum + area, 0);
  const targetTriangles = Math.max(triangles.length, shardCount * 18);
  const detailed = [];

  for (let index = 0; index < triangles.length; index++) {
    const tri = triangles[index];
    const share = areas[index] / Math.max(totalArea, 1e-8);
    const divisions = THREE.MathUtils.clamp(Math.ceil(Math.sqrt(share * targetTriangles)), 1, 14);
    subdivideTriangle(tri, divisions, detailed);
  }

  return detailed;
}

function triangleArea(tri) {
  const abx = tri.b.x - tri.a.x;
  const aby = tri.b.y - tri.a.y;
  const abz = tri.b.z - tri.a.z;
  const acx = tri.c.x - tri.a.x;
  const acy = tri.c.y - tri.a.y;
  const acz = tri.c.z - tri.a.z;
  const cx = aby * acz - abz * acy;
  const cy = abz * acx - abx * acz;
  const cz = abx * acy - aby * acx;
  return Math.sqrt(cx * cx + cy * cy + cz * cz) * 0.5;
}

function subdivideTriangle(tri, divisions, target) {
  const point = (i, j) => {
    const u = i / divisions;
    const v = j / divisions;
    return new THREE.Vector3(
      tri.a.x + (tri.b.x - tri.a.x) * u + (tri.c.x - tri.a.x) * v,
      tri.a.y + (tri.b.y - tri.a.y) * u + (tri.c.y - tri.a.y) * v,
      tri.a.z + (tri.b.z - tri.a.z) * u + (tri.c.z - tri.a.z) * v,
    );
  };

  for (let i = 0; i < divisions; i++) {
    for (let j = 0; j < divisions - i; j++) {
      const a = point(i, j);
      const b = point(i + 1, j);
      const c = point(i, j + 1);
      target.push(makeTriangle(a, b, c));

      if (i + j < divisions - 1) {
        const d = point(i + 1, j + 1);
        target.push(makeTriangle(b, d, c));
      }
    }
  }
}

function makeTriangle(a, b, c) {
  return {
    a,
    b,
    c,
    centroid: new THREE.Vector3().addVectors(a, b).add(c).multiplyScalar(1 / 3),
  };
}

function makeSeeds(triangles, bounds, count, options) {
  const stride = Math.max(1, Math.ceil(triangles.length / 6000));
  const candidates = [];
  for (let i = 0; i < triangles.length; i += stride) candidates.push(triangles[i].centroid);

  const seedCount = Math.min(count, candidates.length);
  const seeds = [];
  const used = new Set();
  const minDistances = new Float64Array(candidates.length);
  minDistances.fill(Infinity);
  let selected = Math.floor(hash3(count, 2, 7, options.seed) * candidates.length);

  for (let i = 0; i < seedCount; i++) {
    const position = candidates[selected];
    used.add(selected);

    seeds.push({
      x: position.x,
      y: position.y,
      z: position.z,
      metric: new THREE.Vector3(
        1 + hash3(i, 19, 23, options.seed) * options.anisotropy,
        1 + hash3(i, 29, 31, options.seed) * options.anisotropy,
        1 + hash3(i, 37, 41, options.seed) * options.anisotropy,
      ),
      noiseScale: 0.55 + hash3(i, 43, 47, options.seed) * 0.9,
    });

    let next = selected;
    let bestDistance = -1;
    for (let candidate = 0; candidate < candidates.length; candidate++) {
      if (used.has(candidate)) continue;
      const distance = candidates[candidate].distanceToSquared(position);
      minDistances[candidate] = Math.min(minDistances[candidate], distance);
      const jitter = 0.9 + hash3(candidate, i, 53, options.seed) * 0.2;
      const score = minDistances[candidate] * jitter;
      if (score > bestDistance) {
        bestDistance = score;
        next = candidate;
      }
    }
    selected = next;
  }

  return seeds;
}

function bucketToShard(bucket, index, seed, options) {
  if (bucket.length === 0) return null;

  const positions = [];
  const colors = [];
  const bounds = new THREE.Box3();
  const tint = new THREE.Color().setHSL(0.045 + (index % 11) * 0.022, 0.62, 0.52 + (index % 5) * 0.035);

  for (const tri of bucket) {
    for (const vertex of [tri.a, tri.b, tri.c]) {
      positions.push(vertex.x, vertex.y, vertex.z);
      bounds.expandByPoint(vertex);
      colors.push(tint.r, tint.g, tint.b);
    }
  }

  const center = bounds.getCenter(new THREE.Vector3());
  for (let i = 0; i < positions.length; i += 3) {
    positions[i] -= center.x;
    positions[i + 1] -= center.y;
    positions[i + 2] -= center.z;
  }

  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
  geometry.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3));
  geometry.computeVertexNormals();
  geometry.computeBoundingBox();
  geometry.computeBoundingSphere();

  return {
    index,
    geometry,
    center,
    half: bounds.getSize(new THREE.Vector3()).multiplyScalar(0.5).max(new THREE.Vector3(0.04, 0.04, 0.04)),
    bounds,
    seed: new THREE.Vector3(seed.x, seed.y, seed.z),
    mass: Math.max(0.15, bucket.length * 0.025 * options.density),
  };
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
