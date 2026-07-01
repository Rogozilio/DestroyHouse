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
scene.add(shardGroup, jointGroup, ghostGroup);

const raycaster = new THREE.Raycaster();
const pointer = new THREE.Vector2();
const timer = new THREE.Timer();
const loadColor = new THREE.Color();
const insideLoadColor = new THREE.Color();
timer.connect(document);

const state = {
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
  stats: { physics: 'loading', shards: 0, bodies: 0, joints: 0 },
};

let physics = null;
let source = null;
let bounds = null;
let shardRecords = [];
let sourceRoot = null;
let renderedJointRevision = -1;

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

  physics = await JoltPhysics.create();
  state.stats.physics = 'ready';
  status.textContent = 'Loading Home.fbx...';

  sourceRoot = await new FBXLoader().loadAsync(new URL('../assets/Home.fbx', import.meta.url).href);
  normalizeSource(sourceRoot);
  ghostGroup.add(sourceRoot.clone());
  ghostGroup.traverse((node) => {
    if (node.isMesh) node.material = ghostMaterial;
  });
  source = extractTriangles(sourceRoot);
  bounds = source.bounds;

  fitCamera(bounds);
  await rebuild();
  animate();
}

async function rebuild() {
  if (!source || !physics) return;
  clearGroups();
  physics = await JoltPhysics.create();

  const fracture = createFracture(source, state);
  bounds = fracture.bounds;
  const minY = bounds.min.y;
  const size = bounds.getSize(new THREE.Vector3());
  const floorY = minY - Math.max(0.04, size.y * 0.015);
  const visualMinHalf = size.clone().multiplyScalar(0.008);
  floor.position.y = floorY;
  physics.addFloor(floorY, Math.max(8, size.length() * 2.2));

  shardRecords = fracture.shards.map((shard) => {
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

  const pairs = findNeighborPairs(fracture.shards, bounds, Math.floor(fracture.shards.length * 3.5));
  const anchorIndices = shardRecords
    .filter((record) => record.isAnchor)
    .map((record) => record.index);
  physics.initializeClusters(fracture.shards, pairs, anchorIndices, {
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
  status.textContent = `${fracture.mode}: ${state.stats.shards} shards in ${state.stats.bodies} compound bodies, ${state.stats.joints} inter-cluster joints.`;
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
  if (event.target !== renderer.domElement) return;
  const rect = renderer.domElement.getBoundingClientRect();
  if (event.clientX < 324 && event.clientY < 418) return;
  pointer.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
  pointer.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
  raycaster.setFromCamera(pointer, camera);
  const hits = raycaster.intersectObjects(shardGroup.children, true);
  let hit = hits[0];
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
  let brokenBonds = directRecord ? physics.damageShard(directRecord.index) : 0;

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
    const impulse = radial.multiplyScalar(state.impactForce * falloff * 0.5)
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
  const ratios = physics.getShardLoadRatios();
  for (const record of shardRecords) {
    const ratio = THREE.MathUtils.clamp(ratios.get(record.index) ?? 0, 0, 1);
    loadColor.setHSL((1 - ratio) * 0.33, 0.82, 0.48);
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
