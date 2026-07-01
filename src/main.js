import * as THREE from 'three';
import { OrbitControls } from '../vendor/three/examples/jsm/controls/OrbitControls.js';
import { FBXLoader } from '../vendor/three/examples/jsm/loaders/FBXLoader.js';
import { createFracture, extractTriangles, findNeighborPairs } from './fracture.js';
import { JoltPhysics } from './physics.js';
import { ImGuiPanel } from './imgui-panel.js';

const canvas = document.querySelector('#scene');
const uiCanvas = document.querySelector('#ui');
const status = document.querySelector('#status');

const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.setClearColor(0x111317, 1);
renderer.shadowMap.enabled = true;

const scene = new THREE.Scene();
scene.fog = new THREE.Fog(0x111317, 7, 24);

const camera = new THREE.PerspectiveCamera(52, 1, 0.05, 200);
camera.position.set(4.4, 3.2, 5.2);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.target.set(0, 1.1, 0);
controls.mouseButtons.LEFT = null;
controls.mouseButtons.MIDDLE = THREE.MOUSE.PAN;
controls.mouseButtons.RIGHT = THREE.MOUSE.ROTATE;

const hemi = new THREE.HemisphereLight(0xd8f2ff, 0x3d332c, 1.4);
scene.add(hemi);
const key = new THREE.DirectionalLight(0xffffff, 2.6);
key.position.set(3.5, 6, 4);
key.castShadow = true;
scene.add(key);

const floorMaterial = new THREE.MeshStandardMaterial({ color: 0x22272d, roughness: 0.86, metalness: 0.02 });
const floor = new THREE.Mesh(new THREE.PlaneGeometry(40, 40), floorMaterial);
floor.rotation.x = -Math.PI / 2;
floor.receiveShadow = true;
scene.add(floor);

const shardOuterMaterial = new THREE.MeshStandardMaterial({
  color: 0x39c85a,
  vertexColors: false,
  side: THREE.DoubleSide,
  roughness: 0.78,
  metalness: 0.02,
});
const shardInnerMaterial = new THREE.MeshStandardMaterial({
  color: 0x1f6f35,
  vertexColors: false,
  side: THREE.DoubleSide,
  roughness: 0.94,
  metalness: 0,
});
const proxyMaterial = new THREE.MeshBasicMaterial({
  colorWrite: false,
  depthWrite: false,
  side: THREE.DoubleSide,
});
const ghostMaterial = new THREE.MeshStandardMaterial({
  color: 0x6f8494,
  transparent: true,
  opacity: 0.075,
  wireframe: true,
  depthWrite: false,
});
const lineMaterial = new THREE.LineBasicMaterial({ color: 0x49b6b0, transparent: true, opacity: 0.42 });

const shardGroup = new THREE.Group();
const jointGroup = new THREE.Group();
const ghostGroup = new THREE.Group();
const projectileGroup = new THREE.Group();
scene.add(shardGroup, jointGroup, ghostGroup, projectileGroup);

const raycaster = new THREE.Raycaster();
const projectileRaycaster = new THREE.Raycaster();
const pointer = new THREE.Vector2();
const timer = new THREE.Timer();
const loadColor = new THREE.Color();
const insideLoadColor = new THREE.Color();
const shardPalette = [
  0x4fbd68,
  0xe0b64c,
  0xdd705c,
  0x55a5d5,
  0x9b7bd2,
  0x4eb6a4,
  0xd66f9a,
  0xa9c957,
].map((color) => new THREE.Color(color));
timer.connect(document);

const state = {
  interactionMode: 'impact',
  noise: 'voronoi',
  shards: 64,
  noiseStrength: 0.9,
  anisotropy: 1.25,
  jointType: 'locked',
  jointSoftness: 0,
  clusterSize: 12,
  loadSafety: 2.75,
  failureDelay: 0.35,
  impactForce: 34,
  impactRadius: 0.72,
  density: 1,
  seed: 7,
  anchorBase: true,
  showJoints: true,
  showLoadHeatmap: false,
  stats: { physics: 'loading', shards: 0, bodies: 0, joints: 0 },
};

let physics = null;
let source = null;
let houseOffsets = [];
let bounds = null;
let shardRecords = [];
let projectiles = [];
let sourceRoot = null;
let renderedJointRevision = -1;

const projectileRadius = 0.16;
const projectileLifetime = 4000;
const projectileGeometry = new THREE.SphereGeometry(projectileRadius, 24, 16);
const projectileMaterial = new THREE.MeshStandardMaterial({
  color: 0xe5a83f,
  emissive: 0x4a2205,
  emissiveIntensity: 0.45,
  roughness: 0.34,
  metalness: 0.42,
});

const panel = new ImGuiPanel(uiCanvas, state, {
  refracture: () => rebuild(),
  reset: () => resetSimulation(),
});

init().catch((error) => {
  console.error(error);
  status.textContent = `Error: ${error.message}`;
});

async function init() {
  resize();
  window.addEventListener('resize', resize);
  renderer.domElement.addEventListener('pointerdown', onScenePointer);
  renderer.domElement.addEventListener('contextmenu', (event) => event.preventDefault());

  physics = await JoltPhysics.create();
  state.stats.physics = 'ready';
  status.textContent = 'Loading Home.fbx...';

  sourceRoot = await new FBXLoader().loadAsync(new URL('../assets/Home.fbx', import.meta.url).href);
  normalizeSource(sourceRoot);
  source = extractTriangles(sourceRoot);
  const houseSize = source.bounds.getSize(new THREE.Vector3());
  const spacing = new THREE.Vector2(houseSize.x * 1.18, houseSize.z * 1.18);
  houseOffsets = [
    new THREE.Vector3(-spacing.x * 0.5, 0, -spacing.y * 0.5),
    new THREE.Vector3(spacing.x * 0.5, 0, -spacing.y * 0.5),
    new THREE.Vector3(-spacing.x * 0.5, 0, spacing.y * 0.5),
    new THREE.Vector3(spacing.x * 0.5, 0, spacing.y * 0.5),
  ];

  for (const offset of houseOffsets) {
    const house = sourceRoot.clone(true);
    house.position.add(offset);
    ghostGroup.add(house);
  }
  ghostGroup.traverse((node) => {
    if (node.isMesh) node.material = ghostMaterial;
  });
  bounds = new THREE.Box3();
  for (const offset of houseOffsets) bounds.union(source.bounds.clone().translate(offset));

  fitCamera(bounds);
  await rebuild();
  animate();
}

async function rebuild() {
  if (!source || !physics) return;
  clearProjectiles();
  clearGroups();
  physics = await JoltPhysics.create();

  const fracture = createFracture(source, state);
  bounds = new THREE.Box3();
  for (const offset of houseOffsets) bounds.union(source.bounds.clone().translate(offset));

  const allShards = [];
  const allPairs = [];
  const basePairs = findNeighborPairs(
    fracture.shards,
    fracture.bounds,
    Math.floor(fracture.shards.length * 3.5),
  );
  for (let houseIndex = 0; houseIndex < houseOffsets.length; houseIndex++) {
    const shardOffset = allShards.length;
    const offset = houseOffsets[houseIndex];
    for (const shard of fracture.shards) {
      allShards.push({
        ...shard,
        index: allShards.length,
        geometry: shard.geometry,
        center: shard.center.clone().add(offset),
        bounds: shard.bounds.clone().translate(offset),
        seed: shard.seed.clone().add(offset),
        half: shard.half.clone(),
      });
    }
    for (const pair of basePairs) {
      allPairs.push({
        ...pair,
        a: pair.a + shardOffset,
        b: pair.b + shardOffset,
      });
    }
  }

  const minY = bounds.min.y;
  const size = bounds.getSize(new THREE.Vector3());
  const floorY = minY - Math.max(0.04, size.y * 0.015);
  const visualMinHalf = source.bounds.getSize(new THREE.Vector3()).multiplyScalar(0.008);
  floor.position.y = floorY;
  physics.addFloor(floorY, Math.max(8, size.length() * 2.2));

  shardRecords = allShards.map((shard) => {
    const mesh = new THREE.Group();
    const surface = new THREE.Mesh(shard.geometry, [
      shardOuterMaterial.clone(),
      shardInnerMaterial.clone(),
    ]);
    const visualHalf = shard.half.clone().max(visualMinHalf);
    const proxy = new THREE.Mesh(
      new THREE.BoxGeometry(visualHalf.x * 2.04, visualHalf.y * 2.04, visualHalf.z * 2.04),
      proxyMaterial,
    );
    surface.castShadow = true;
    mesh.add(proxy, surface);
    mesh.position.copy(shard.center);
    shardGroup.add(mesh);

    const isAnchor = state.anchorBase && shard.center.y < bounds.min.y + size.y * 0.13;
    mesh.userData.shard = shard;
    return {
      index: shard.index,
      mesh,
      surface,
      proxy,
      shard,
      isAnchor,
    };
  });

  const anchorIndices = shardRecords
    .filter((record) => record.isAnchor)
    .map((record) => record.index);
  physics.initializeClusters(allShards, allPairs, anchorIndices, {
    clusterSize: state.clusterSize,
    jointSoftness: state.jointSoftness,
    jointType: state.jointType,
  });
  physics.calibrateStructuralLoads(state.loadSafety);
  updateLoadColors();
  rebuildJointLines();

  state.stats.shards = shardRecords.length;
  state.stats.bodies = physics.getClusterCount();
  state.stats.joints = physics.constraints.length;
  status.textContent = `4 houses, ${fracture.mode}: ${state.stats.shards} shards in ${state.stats.bodies} compound bodies, ${state.stats.joints} inter-cluster joints.`;
  panel.draw();
}

function resetSimulation() {
  rebuild();
}

function clearGroups() {
  for (const record of shardRecords) {
    const materials = Array.isArray(record.surface.material)
      ? record.surface.material
      : [record.surface.material];
    for (const material of materials) material.dispose();
  }
  for (const group of [shardGroup, jointGroup]) {
    while (group.children.length) {
      const child = group.children.pop();
      child.traverse?.((node) => node.geometry?.dispose?.());
      child.geometry?.dispose?.();
    }
  }
  shardRecords = [];
}

function clearProjectiles() {
  for (const projectile of projectiles) physics?.removeBody(projectile.bodyId);
  projectileGroup.clear();
  projectiles = [];
}

function normalizeSource(root) {
  const box = new THREE.Box3().setFromObject(root);
  const size = box.getSize(new THREE.Vector3());
  const center = box.getCenter(new THREE.Vector3());
  const scale = 3.2 / Math.max(size.x, size.y, size.z, 0.001);
  root.scale.multiplyScalar(scale);
  root.position.sub(center.multiplyScalar(scale));
  root.updateWorldMatrix(true, true);
}

function fitCamera(box) {
  const size = box.getSize(new THREE.Vector3());
  const center = box.getCenter(new THREE.Vector3());
  controls.target.copy(center);
  camera.position.copy(center).add(new THREE.Vector3(size.x * 1.4 + 2, size.y * 0.85 + 1.2, size.z * 1.8 + 2));
  camera.near = 0.03;
  camera.far = Math.max(60, size.length() * 18);
  camera.updateProjectionMatrix();
}

function onScenePointer(event) {
  if (event.button !== 0 || event.target !== renderer.domElement) return;
  const rect = renderer.domElement.getBoundingClientRect();
  if (event.clientX < 324 && event.clientY < 418) return;
  pointer.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
  pointer.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
  raycaster.setFromCamera(pointer, camera);
  if (state.interactionMode === 'ball') {
    spawnProjectile(raycaster.ray);
    return;
  }

  const surfaceHits = raycaster.intersectObjects(
    shardRecords.map((record) => record.surface),
    false,
  );
  let hit = surfaceHits[0];
  if (!hit) {
    const proxyHits = raycaster.intersectObjects(
      shardRecords.map((record) => record.proxy),
      false,
    );
    hit = proxyHits[0];
  }
  if (!hit) {
    let nearest = Infinity;
    const fallbackPoint = new THREE.Vector3();
    for (const record of shardRecords) {
      const box = new THREE.Box3().setFromObject(record.mesh);
      const point = raycaster.ray.intersectBox(box, fallbackPoint);
      if (!point) continue;
      const distance = point.distanceTo(camera.position);
      if (distance < nearest) {
        nearest = distance;
        hit = { point: point.clone(), record };
      }
    }
  }
  if (!hit) return;
  const directRecord = hit.record ?? shardRecords.find((record) => (
    record.surface === hit.object
    || record.proxy === hit.object
    || record.mesh === hit.object.parent
  ));
  const rayDir = raycaster.ray.direction.clone().normalize();
  let brokenBonds = directRecord ? physics.detachShard(directRecord.index) : 0;

  const impacted = shardRecords
    .map((record) => ({ record, distance: record.mesh.position.distanceTo(hit.point) }))
    .filter(({ distance }) => distance <= state.impactRadius)
    .sort((a, b) => a.distance - b.distance);

  if (directRecord && !impacted.some(({ record }) => record === directRecord)) {
    impacted.unshift({ record: directRecord, distance: 0 });
  }

  const boundsSize = bounds.getSize(new THREE.Vector3());
  const foundationHit = hit.point.y < bounds.min.y + boundsSize.y * 0.24;
  if (foundationHit) {
    const included = new Set(impacted.map(({ record }) => record));
    for (const record of shardRecords) {
      if (!record.isAnchor || included.has(record)) continue;
      const dx = record.mesh.position.x - hit.point.x;
      const dz = record.mesh.position.z - hit.point.z;
      const horizontalDistance = Math.hypot(dx, dz);
      if (horizontalDistance > state.impactRadius * 1.35) continue;
      impacted.push({ record, distance: horizontalDistance });
      included.add(record);
    }
    impacted.sort((a, b) => a.distance - b.distance);
  }

  if (impacted.length === 0 && shardRecords.length > 0) {
    const nearest = shardRecords
      .map((record) => ({ record, distance: record.mesh.position.distanceTo(hit.point) }))
      .sort((a, b) => a.distance - b.distance)[0];
    impacted.push(nearest);
  }

  if (foundationHit) {
    const damagedAnchors = impacted
      .filter(({ record }) => record.isAnchor)
      .map(({ record }) => record.index);
    brokenBonds += physics.damageShards(damagedAnchors);
  }

  const clusterImpulses = new Map();
  for (const { record, distance } of impacted) {
    physics.releaseShard(record.index);
    const radial = record.mesh.position.clone().sub(hit.point).normalize();
    const falloff = Math.max(0.2, 1 - distance / state.impactRadius);
    const impulse = record === directRecord
      ? rayDir.clone().negate()
        .add(new THREE.Vector3(0, 0.16, 0))
        .normalize()
        .multiplyScalar(state.impactForce)
      : radial.multiplyScalar(state.impactForce * falloff * 0.5)
        .add(rayDir.clone().multiplyScalar(state.impactForce * falloff));
    const clusterId = physics.getClusterIdForShard(record.index);
    const entry = clusterImpulses.get(clusterId) ?? {
      index: record.index,
      impulse: new THREE.Vector3(),
      count: 0,
    };
    entry.impulse.add(impulse);
    entry.count++;
    clusterImpulses.set(clusterId, entry);
  }
  for (const entry of clusterImpulses.values()) {
    entry.impulse.multiplyScalar(1 / entry.count);
    physics.addImpulseToShard(entry.index, entry.impulse, hit.point);
  }

  const support = physics.releaseUnsupported();
  state.stats.bodies = physics.getClusterCount();
  state.stats.joints = physics.constraints.length;
  const directLinks = directRecord ? physics.countShardBonds(directRecord.index) : 0;
  status.textContent = `Detached hit (${directLinks} bonds); broke ${brokenBonds} bonds, ${state.stats.bodies} bodies, ${state.stats.joints} joints, ${support.anchors} anchors.`;
  panel.draw();
}

function spawnProjectile(ray) {
  const direction = ray.direction.clone().normalize();
  const position = camera.position.clone().addScaledVector(direction, projectileRadius + 0.42);
  const velocity = direction.clone().multiplyScalar(10.5);
  const mesh = new THREE.Mesh(projectileGeometry, projectileMaterial);
  mesh.position.copy(position);
  mesh.castShadow = true;
  projectileGroup.add(mesh);

  projectiles.push({
    bodyId: physics.addSphere(position, projectileRadius, 5.5, velocity),
    mesh,
    previousPosition: position.clone(),
    expiresAt: performance.now() + projectileLifetime,
    hasDamaged: false,
  });
  status.textContent = `Ball launched; ${projectiles.length} active projectile${projectiles.length === 1 ? '' : 's'}.`;
}

function updateProjectiles(timestamp) {
  for (let i = projectiles.length - 1; i >= 0; i--) {
    const projectile = projectiles[i];
    physics.syncBody(projectile.mesh, projectile.bodyId);

    if (!projectile.hasDamaged) {
      const travel = projectile.mesh.position.clone().sub(projectile.previousPosition);
      const distance = travel.length();
      if (distance > 1e-5) {
        const direction = travel.multiplyScalar(1 / distance);
        projectileRaycaster.ray.origin.copy(projectile.previousPosition);
        projectileRaycaster.ray.direction.copy(direction);
        projectileRaycaster.near = 0;
        projectileRaycaster.far = distance + projectileRadius * 1.5;
        const hit = projectileRaycaster.intersectObjects(
          shardRecords.map((record) => record.surface),
          false,
        )[0];
        if (hit) {
          damageFromProjectile(hit, direction);
          projectile.hasDamaged = true;
        }
      }
      projectile.previousPosition.copy(projectile.mesh.position);
    }

    if (timestamp >= projectile.expiresAt) removeProjectile(i);
  }
}

function damageFromProjectile(hit, direction) {
  const directRecord = shardRecords.find((record) => record.surface === hit.object);
  if (!directRecord) return;

  const damageRadius = Math.max(projectileRadius * 2, state.impactRadius * 0.8);
  const impacted = shardRecords
    .map((record) => ({ record, distance: record.mesh.position.distanceTo(hit.point) }))
    .filter(({ distance }) => distance <= damageRadius)
    .sort((a, b) => a.distance - b.distance);
  if (!impacted.some(({ record }) => record === directRecord)) {
    impacted.unshift({ record: directRecord, distance: 0 });
  }

  let brokenBonds = physics.damageShards(impacted.map(({ record }) => record.index));
  brokenBonds += physics.detachShard(directRecord.index);
  const clusterImpulses = new Map();

  for (const { record, distance } of impacted) {
    physics.releaseShard(record.index);
    const falloff = Math.max(0.22, 1 - distance / damageRadius);
    const radial = record.mesh.position.clone().sub(hit.point).normalize();
    const impulse = direction.clone().multiplyScalar(state.impactForce * 1.2 * falloff)
      .add(radial.multiplyScalar(state.impactForce * 0.35 * falloff));
    const clusterId = physics.getClusterIdForShard(record.index);
    const entry = clusterImpulses.get(clusterId) ?? {
      index: record.index,
      impulse: new THREE.Vector3(),
      count: 0,
    };
    entry.impulse.add(impulse);
    entry.count++;
    clusterImpulses.set(clusterId, entry);
  }

  for (const entry of clusterImpulses.values()) {
    entry.impulse.multiplyScalar(1 / entry.count);
    physics.addImpulseToShard(entry.index, entry.impulse, hit.point);
  }

  const support = physics.releaseUnsupported();
  state.stats.bodies = physics.getClusterCount();
  state.stats.joints = physics.constraints.length;
  status.textContent = `Ball impact: broke ${brokenBonds} bonds, ${state.stats.bodies} bodies, ${state.stats.joints} joints, ${support.anchors} anchors.`;
  panel.draw();
}

function removeProjectile(index) {
  const [projectile] = projectiles.splice(index, 1);
  if (!projectile) return;
  physics.removeBody(projectile.bodyId);
  projectileGroup.remove(projectile.mesh);
}

function makeJointLine(a, b) {
  const geometry = new THREE.BufferGeometry().setFromPoints([a.clone(), b.clone()]);
  return new THREE.Line(geometry, lineMaterial);
}

function updateJointLines() {
  jointGroup.visible = state.showJoints;
  if (renderedJointRevision !== physics.jointRevision) rebuildJointLines();
  for (const link of physics.constraints) {
    const line = link.line;
    if (!line) continue;
    const a = physics.getPosition(link.a);
    const b = physics.getPosition(link.b);
    const attr = line.geometry.attributes.position;
    attr.setXYZ(0, a.x, a.y, a.z);
    attr.setXYZ(1, b.x, b.y, b.z);
    attr.needsUpdate = true;
  }
}

function rebuildJointLines() {
  while (jointGroup.children.length) {
    const line = jointGroup.children.pop();
    line.geometry?.dispose?.();
  }
  for (const link of physics.constraints) {
    link.line = makeJointLine(physics.getPosition(link.a), physics.getPosition(link.b));
    jointGroup.add(link.line);
  }
  renderedJointRevision = physics.jointRevision;
}

function updateLoadColors() {
  const ratios = state.showLoadHeatmap ? physics.getShardLoadRatios() : null;
  for (const record of shardRecords) {
    if (ratios) {
      const ratio = THREE.MathUtils.clamp(ratios.get(record.index) ?? 0, 0, 1);
      loadColor.setHSL((1 - ratio) * 0.33, 0.82, 0.48);
    } else {
      loadColor.copy(shardPalette[(record.index * 5) % shardPalette.length]);
    }
    insideLoadColor.copy(loadColor).multiplyScalar(0.48);
    const materials = Array.isArray(record.surface.material)
      ? record.surface.material
      : [record.surface.material];
    materials[0].color.copy(loadColor);
    materials[1]?.color.copy(insideLoadColor);
  }
}

function animate(timestamp) {
  requestAnimationFrame(animate);
  timer.update(timestamp);
  const dt = timer.getDelta();
  controls.update();
  if (physics) {
    physics.step(dt, 2);
    const loadFailures = physics.updateStructuralLoads(dt, state.failureDelay);
    if (loadFailures > 0) {
      const support = physics.releaseUnsupported();
      state.stats.bodies = physics.getClusterCount();
      state.stats.joints = physics.constraints.length;
      status.textContent = `Load failure: ${loadFailures} bonds broke, ${state.stats.bodies} bodies, ${state.stats.joints} joints, ${support.anchors} anchors.`;
      panel.draw();
    }
    for (const record of shardRecords) physics.syncShard(record.mesh, record.index);
    updateProjectiles(timestamp);
    updateLoadColors();
    updateJointLines();
  }
  renderer.render(scene, camera);
}

function resize() {
  const width = window.innerWidth;
  const height = window.innerHeight;
  camera.aspect = width / height;
  camera.updateProjectionMatrix();
  renderer.setSize(width, height, false);
  panel.resize(Math.min(324, width), Math.min(680, height), Math.min(window.devicePixelRatio, 2));
}
