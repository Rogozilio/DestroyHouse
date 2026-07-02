import * as THREE from 'three';

const LAYER_FLOOR = 0;
const LAYER_CORE = 1;
const LAYER_SHELL = 2;
const LAYER_PROJECTILE = 3;
const OBJECT_LAYERS = 4;
const BP_STATIC = 0;
const BP_MOVING = 1;
// Convex hulls are shrunk slightly toward each shard's centre so neighbouring
// cells keep a small gap in the intact structure (otherwise every touching face
// spawns a contact and the packed lattice grinds to a halt). Detached pieces
// still collide with a near-correct shape.
const HULL_SCALE = 0.95;

export class JoltPhysics {
  static async create() {
    // Reuse a single Jolt WASM instance across rebuilds. Instantiating a new
    // one per reset leaks the whole module (tens of MB) and tanks the frame rate.
    if (!JoltPhysics.module) {
      const { default: initJolt } = await import('../vendor/jolt/jolt-physics.wasm-compat.js');
      JoltPhysics.module = await initJolt();
    }
    return new JoltPhysics(JoltPhysics.module);
  }

  destroy() {
    const Jolt = this.Jolt;
    for (const link of this.constraints) {
      if (link.line) link.line.visible = false;
      this.system.RemoveConstraint(link.constraint);
    }
    this.constraints = [];
    // Destroying the interface frees its physics system, bodies and shapes;
    // the JS contact listener has to be released separately.
    Jolt.destroy(this.jolt);
    Jolt.destroy(this.contactListener);
    this.jolt = null;
    this.system = null;
    this.bodyInterface = null;
  }

  constructor(Jolt) {
    this.Jolt = Jolt;
    this.constraints = [];
    this.bonds = [];
    this.shards = [];
    this.clusters = new Map();
    this.shardCluster = new Map();
    this.anchorShards = new Set();
    this.initialAnchorCount = 0;
    this.nextClusterId = 1;
    this.jointRevision = 0;
    this.jointType = 'locked';
    this.jointSoftness = 0;
    this.projectileBodies = new Map();
    this.clusterBodies = new Map();
    this.projectileContacts = [];
    this.stepDuration = 1 / 60;

    const objectFilter = new Jolt.ObjectLayerPairFilterTable(OBJECT_LAYERS);
    objectFilter.EnableCollision(LAYER_FLOOR, LAYER_CORE);
    objectFilter.EnableCollision(LAYER_FLOOR, LAYER_SHELL);
    objectFilter.EnableCollision(LAYER_FLOOR, LAYER_PROJECTILE);
    objectFilter.EnableCollision(LAYER_CORE, LAYER_CORE);
    objectFilter.EnableCollision(LAYER_CORE, LAYER_PROJECTILE);
    objectFilter.EnableCollision(LAYER_SHELL, LAYER_SHELL);
    objectFilter.EnableCollision(LAYER_SHELL, LAYER_PROJECTILE);
    // Core and shell overlap at the skin/core interface and are held together by
    // joints, so they must NOT collide with each other - colliding the overlap
    // blows the structure apart.

    const bpInterface = new Jolt.BroadPhaseLayerInterfaceTable(OBJECT_LAYERS, 2);
    bpInterface.MapObjectToBroadPhaseLayer(LAYER_FLOOR, new Jolt.BroadPhaseLayer(BP_STATIC));
    bpInterface.MapObjectToBroadPhaseLayer(LAYER_CORE, new Jolt.BroadPhaseLayer(BP_MOVING));
    bpInterface.MapObjectToBroadPhaseLayer(LAYER_SHELL, new Jolt.BroadPhaseLayer(BP_MOVING));
    bpInterface.MapObjectToBroadPhaseLayer(LAYER_PROJECTILE, new Jolt.BroadPhaseLayer(BP_MOVING));

    const objectVsBroadPhase = new Jolt.ObjectVsBroadPhaseLayerFilterTable(bpInterface, 2, objectFilter, OBJECT_LAYERS);
    const settings = new Jolt.JoltSettings();
    settings.mMaxBodies = 4096;
    settings.mMaxBodyPairs = 8192;
    settings.mMaxContactConstraints = 8192;
    settings.mBroadPhaseLayerInterface = bpInterface;
    settings.mObjectLayerPairFilter = objectFilter;
    settings.mObjectVsBroadPhaseLayerFilter = objectVsBroadPhase;

    this.jolt = new Jolt.JoltInterface(settings);
    this.system = this.jolt.GetPhysicsSystem();
    this.system.SetGravity(new Jolt.Vec3(0, -9.8, 0));
    // After a rebuild the freshly-placed lattice creeps a few centimetres while
    // it settles. With the default sleep threshold that slow creep keeps all 312
    // bodies awake for ~3.5s, solving 1020 stiff joints every frame (a big FPS
    // dip on Reset sim). Let bodies sleep quickly once they are barely moving;
    // impacts and broken bonds still wake them normally.
    const physicsSettings = this.system.GetPhysicsSettings();
    physicsSettings.mTimeBeforeSleep = 0.3;
    physicsSettings.mPointVelocitySleepThreshold = 0.2;
    this.system.SetPhysicsSettings(physicsSettings);
    this.bodyInterface = this.system.GetBodyInterface();
    this.installContactListener();
  }

  getBodyKey(bodyId) {
    return bodyId.GetIndexAndSequenceNumber();
  }

  installContactListener() {
    const Jolt = this.Jolt;
    this.contactListener = new Jolt.ContactListenerJS();
    this.contactListener.OnContactValidate = () => (
      Jolt.ValidateResult_AcceptAllContactsForThisBodyPair
    );
    this.contactListener.OnContactAdded = (
      body1Pointer,
      body2Pointer,
      manifoldPointer,
    ) => {
      this.recordProjectileContact(body1Pointer, body2Pointer, manifoldPointer);
    };
    this.contactListener.OnContactPersisted = () => {};
    this.contactListener.OnContactRemoved = () => {};
    this.system.SetContactListener(this.contactListener);
  }

  recordProjectileContact(body1Pointer, body2Pointer, manifoldPointer) {
    const Jolt = this.Jolt;
    const body1 = Jolt.wrapPointer(body1Pointer, Jolt.Body);
    const body2 = Jolt.wrapPointer(body2Pointer, Jolt.Body);
    const body1Key = this.getBodyKey(body1.GetID());
    const body2Key = this.getBodyKey(body2.GetID());
    const projectileIsBody1 = this.projectileBodies.has(body1Key);
    const projectileIsBody2 = this.projectileBodies.has(body2Key);
    if (projectileIsBody1 === projectileIsBody2) return;

    const projectileBody = projectileIsBody1 ? body1 : body2;
    const targetBody = projectileIsBody1 ? body2 : body1;
    const projectileBodyKey = projectileIsBody1 ? body1Key : body2Key;
    const targetBodyKey = projectileIsBody1 ? body2Key : body1Key;
    if (!this.clusterBodies.has(targetBodyKey)) return;

    const manifold = Jolt.wrapPointer(manifoldPointer, Jolt.ContactManifold);
    const targetPoints = projectileIsBody1
      ? manifold.get_mRelativeContactPointsOn2()
      : manifold.get_mRelativeContactPointsOn1();
    if (targetPoints.size() === 0) return;

    const point = projectileIsBody1
      ? manifold.GetWorldSpaceContactPointOn2(0)
      : manifold.GetWorldSpaceContactPointOn1(0);
    // Jolt's WASM getters reuse temporary vector storage, so copy every value
    // before calling another getter that returns the same vector type.
    const pointValues = [point.GetX(), point.GetY(), point.GetZ()];
    const normal = manifold.get_mWorldSpaceNormal();
    const normalValues = [normal.GetX(), normal.GetY(), normal.GetZ()];
    const projectileVelocity = projectileBody.GetLinearVelocity();
    const projectileVelocityValues = [
      projectileVelocity.GetX(),
      projectileVelocity.GetY(),
      projectileVelocity.GetZ(),
    ];
    const targetVelocity = targetBody.GetLinearVelocity();
    const targetVelocityValues = [
      targetVelocity.GetX(),
      targetVelocity.GetY(),
      targetVelocity.GetZ(),
    ];
    const relativeNormalSpeed = Math.abs(
      (projectileVelocityValues[0] - targetVelocityValues[0]) * normalValues[0]
      + (projectileVelocityValues[1] - targetVelocityValues[1]) * normalValues[1]
      + (projectileVelocityValues[2] - targetVelocityValues[2]) * normalValues[2],
    );
    if (relativeNormalSpeed <= 1e-3) return;

    const projectileMass = this.projectileBodies.get(projectileBodyKey).mass;
    const targetInverseMass = targetBody.IsDynamic()
      ? targetBody.GetMotionProperties().GetInverseMass()
      : 0;
    // The JS binding does not expose the solver's final contact impulse.
    // Estimate it from closing speed and reduced mass, then derive average force.
    const reducedMass = 1 / (1 / projectileMass + targetInverseMass);
    const impulse = reducedMass * relativeNormalSpeed;
    const force = impulse / Math.max(this.stepDuration, 1 / 240);
    const targetSubShape = projectileIsBody1
      ? manifold.get_mSubShapeID2()
      : manifold.get_mSubShapeID1();
    const shardIndex = targetBody.GetShape().GetSubShapeUserData(targetSubShape);

    this.projectileContacts.push({
      projectileBodyKey,
      shardIndex,
      point: new THREE.Vector3(...pointValues),
      relativeNormalSpeed,
      impulse,
      force,
    });
  }

  drainProjectileContacts() {
    const contacts = this.projectileContacts;
    this.projectileContacts = [];
    return contacts;
  }

  addFloor(y, radius) {
    const Jolt = this.Jolt;
    const half = new Jolt.Vec3(radius, 0.08, radius);
    const shape = new Jolt.BoxShape(half, 0.02, null);
    const settings = new Jolt.BodyCreationSettings(
      shape,
      new Jolt.RVec3(0, y - 0.08, 0),
      Jolt.Quat.prototype.sIdentity(),
      Jolt.EMotionType_Static,
      LAYER_FLOOR,
    );
    const createdId = this.bodyInterface.CreateAndAddBody(settings, Jolt.EActivation_DontActivate);
    const id = new Jolt.BodyID(createdId.GetIndexAndSequenceNumber());
    this.bodyInterface.SetFriction(id, 0.96);
    this.bodyInterface.SetRestitution(id, 0);
    return id;
  }

  addSphere(position, radius, mass, velocity) {
    const Jolt = this.Jolt;
    const shape = new Jolt.SphereShape(radius, null);
    const settings = new Jolt.BodyCreationSettings(
      shape,
      new Jolt.RVec3(position.x, position.y, position.z),
      Jolt.Quat.prototype.sIdentity(),
      Jolt.EMotionType_Dynamic,
      LAYER_PROJECTILE,
    );
    settings.mLinearDamping = 0.03;
    settings.mAngularDamping = 0.12;
    settings.mMotionQuality = Jolt.EMotionQuality_LinearCast;
    settings.mOverrideMassProperties = Jolt.EOverrideMassProperties_CalculateInertia;
    settings.mMassPropertiesOverride.mMass = mass;

    const createdId = this.bodyInterface.CreateAndAddBody(settings, Jolt.EActivation_Activate);
    const bodyId = new Jolt.BodyID(createdId.GetIndexAndSequenceNumber());
    this.bodyInterface.SetFriction(bodyId, 0.62);
    this.bodyInterface.SetRestitution(bodyId, 0);
    this.bodyInterface.SetLinearVelocity(
      bodyId,
      new Jolt.Vec3(velocity.x, velocity.y, velocity.z),
    );
    this.projectileBodies.set(this.getBodyKey(bodyId), { mass });
    return bodyId;
  }

  removeBody(bodyId) {
    if (!bodyId) return;
    const bodyKey = this.getBodyKey(bodyId);
    this.projectileBodies.delete(bodyKey);
    this.projectileContacts = this.projectileContacts.filter(
      (contact) => contact.projectileBodyKey !== bodyKey,
    );
    this.bodyInterface.RemoveBody(bodyId);
    this.bodyInterface.DestroyBody(bodyId);
  }

  syncBody(mesh, bodyId) {
    const p = this.bodyInterface.GetPosition(bodyId);
    const q = this.bodyInterface.GetRotation(bodyId);
    mesh.position.set(p.GetX(), p.GetY(), p.GetZ());
    mesh.quaternion.set(q.GetX(), q.GetY(), q.GetZ(), q.GetW());
  }

  initializeClusters(shards, pairs, anchorIndices, options = {}) {
    this.shards = shards.map((shard, index) => ({
      index,
      center: shard.center.clone(),
      half: shard.half.clone(),
      points: this.extractHullPoints(shard.geometry),
      mass: Math.max(0.15, shard.mass),
      layer: shard.layer ?? 'core',
      y: shard.center.y,
    }));
    this.anchorShards = new Set(anchorIndices);
    this.initialAnchorCount = this.anchorShards.size;
    this.jointType = options.jointType ?? 'locked';
    this.jointSoftness = options.jointSoftness ?? 0;

    this.bonds = pairs.map((pair, index) => ({
      index,
      a: pair.a,
      b: pair.b,
      distance: pair.distance,
      strength: pair.strength ?? 'core',
      active: true,
      load: 0,
      loadRatio: 0,
      capacity: Infinity,
      overloadTime: 0,
      baselineLoad: 0,
    }));

    const components = this.partitionInitialClusters(Math.max(1, options.clusterSize ?? 12));
    for (const indices of components) this.createCluster(indices);
    this.rebuildPhysicalConstraints();
  }

  partitionInitialClusters(maxSize) {
    const adjacency = this.makeBondAdjacency();
    const unassigned = new Set(this.shards.map((shard) => shard.index));
    const orderedSeeds = this.shards
      .map((shard) => shard.index)
      .sort((a, b) => this.shards[a].y - this.shards[b].y);
    const components = [];

    for (const seed of orderedSeeds) {
      if (!unassigned.has(seed)) continue;
      const component = [];
      const queued = new Set([seed]);
      const queue = [seed];

      while (queue.length > 0 && component.length < maxSize) {
        const current = queue.shift();
        queued.delete(current);
        if (!unassigned.has(current)) continue;
        unassigned.delete(current);
        component.push(current);

        const neighbors = (adjacency.get(current) ?? [])
          .map((edge) => edge.neighbor)
          // Keep clusters pure to one layer: a compound body mixing core and
          // shell shards would put overlapping shell pieces on the core
          // collision layer, so they fight the joints and the structure shakes.
          .filter((neighbor) => unassigned.has(neighbor) && !queued.has(neighbor)
            && this.shards[neighbor].layer === this.shards[seed].layer)
          .sort((a, b) => this.shards[current].center.distanceToSquared(this.shards[a].center)
            - this.shards[current].center.distanceToSquared(this.shards[b].center));
        for (const neighbor of neighbors) {
          queue.push(neighbor);
          queued.add(neighbor);
        }
      }

      components.push(component);
    }

    return components;
  }

  // Deduplicated local vertices of a shard, used to build a convex-hull collider
  // that matches the Voronoi cell instead of a coarse box.
  extractHullPoints(geometry) {
    const pos = geometry?.attributes?.position;
    if (!pos) return null;
    const seen = new Set();
    const points = [];
    for (let v = 0; v < pos.count; v++) {
      const x = pos.getX(v);
      const y = pos.getY(v);
      const z = pos.getZ(v);
      const key = `${Math.round(x * 1000)},${Math.round(y * 1000)},${Math.round(z * 1000)}`;
      if (seen.has(key)) continue;
      seen.add(key);
      points.push(x, y, z);
    }
    return points.length >= 12 ? points : null;
  }

  // Convex hull matching the shard's real shape, so detached pieces rest against
  // each other instead of passing through. Falls back to a box if the hull is
  // degenerate.
  makeShardShape(shard) {
    const Jolt = this.Jolt;
    if (shard.points) {
      const hull = new Jolt.ConvexHullShapeSettings();
      hull.mMaxConvexRadius = 0.001;
      for (let i = 0; i < shard.points.length; i += 3) {
        hull.mPoints.push_back(new Jolt.Vec3(
          shard.points[i] * HULL_SCALE,
          shard.points[i + 1] * HULL_SCALE,
          shard.points[i + 2] * HULL_SCALE,
        ));
      }
      const result = hull.Create();
      if (result.IsValid()) return result.Get();
    }
    const half = new Jolt.Vec3(
      Math.max(0.03, shard.half.x * 0.9),
      Math.max(0.03, shard.half.y * 0.9),
      Math.max(0.03, shard.half.z * 0.9),
    );
    return new Jolt.BoxShape(half, 0.01, null);
  }

  createCluster(shardIndices, inherited = null) {
    const Jolt = this.Jolt;
    const quaternion = inherited?.quaternion?.clone() ?? new THREE.Quaternion();
    const worldPositions = new Map();
    let totalMass = 0;
    const center = new THREE.Vector3();

    for (const index of shardIndices) {
      const shard = this.shards[index];
      const world = inherited?.worldPositions?.get(index)?.clone() ?? shard.center.clone();
      worldPositions.set(index, world);
      center.addScaledVector(world, shard.mass);
      totalMass += shard.mass;
    }
    center.multiplyScalar(1 / Math.max(totalMass, 1e-5));

    const inverse = quaternion.clone().invert();
    const localOffsets = new Map();
    const compound = new Jolt.StaticCompoundShapeSettings();
    const identity = Jolt.Quat.prototype.sIdentity();

    for (const index of shardIndices) {
      const shard = this.shards[index];
      const local = worldPositions.get(index).clone().sub(center).applyQuaternion(inverse);
      localOffsets.set(index, local);
      const shape = this.makeShardShape(shard);
      compound.AddShapeShape(
        new Jolt.Vec3(local.x, local.y, local.z),
        identity,
        shape,
        index,
      );
    }

    const shapeResult = compound.Create();
    if (!shapeResult.IsValid()) {
      throw new Error(`Jolt compound shape failed: ${shapeResult.GetError()}`);
    }
    const shape = shapeResult.Get();
    const anchored = shardIndices.some((index) => this.anchorShards.has(index));
    const clusterMass = shardIndices.reduce(
      (sum, index) => sum + this.shards[index].mass,
      0,
    );
    const motion = anchored ? Jolt.EMotionType_Static : Jolt.EMotionType_Dynamic;
    const isShell = shardIndices.every((index) => this.shards[index].layer === 'shell');
    const layer = isShell ? LAYER_SHELL : LAYER_CORE;
    const settings = new Jolt.BodyCreationSettings(
      shape,
      new Jolt.RVec3(center.x, center.y, center.z),
      new Jolt.Quat(quaternion.x, quaternion.y, quaternion.z, quaternion.w),
      motion,
      layer,
    );
    settings.mLinearDamping = 0.32;
    settings.mAngularDamping = 0.9;
    settings.mMaxAngularVelocity = 3;
    settings.mOverrideMassProperties = Jolt.EOverrideMassProperties_CalculateInertia;
    settings.mMassPropertiesOverride.mMass = Math.max(10, clusterMass * 10);

    const createdId = this.bodyInterface.CreateAndAddBody(
      settings,
      anchored ? Jolt.EActivation_DontActivate : Jolt.EActivation_Activate,
    );
    const bodyId = new Jolt.BodyID(createdId.GetIndexAndSequenceNumber());
    this.bodyInterface.SetFriction(bodyId, 0.96);
    this.bodyInterface.SetRestitution(bodyId, 0);

    if (!anchored && inherited) {
      const offset = center.clone().sub(inherited.position);
      const linear = inherited.linear.clone().add(inherited.angular.clone().cross(offset));
      this.bodyInterface.SetLinearAndAngularVelocity(
        bodyId,
        new Jolt.Vec3(linear.x, linear.y, linear.z),
        new Jolt.Vec3(inherited.angular.x, inherited.angular.y, inherited.angular.z),
      );
    }

    const cluster = {
      id: this.nextClusterId++,
      bodyId,
      shardIndices: new Set(shardIndices),
      localOffsets,
      anchored,
      layer,
      shape,
      center,
    };
    this.clusters.set(cluster.id, cluster);
    this.clusterBodies.set(this.getBodyKey(bodyId), cluster.id);
    for (const index of shardIndices) this.shardCluster.set(index, cluster.id);
    return cluster;
  }

  removeCluster(cluster) {
    if (!cluster) return;
    this.clusterBodies.delete(this.getBodyKey(cluster.bodyId));
    this.bodyInterface.RemoveBody(cluster.bodyId);
    this.bodyInterface.DestroyBody(cluster.bodyId);
    this.clusters.delete(cluster.id);
    for (const index of cluster.shardIndices) this.shardCluster.delete(index);
  }

  clearPhysicalConstraints() {
    for (const link of this.constraints) {
      if (link.line) link.line.visible = false;
      this.system.RemoveConstraint(link.constraint);
    }
    this.constraints = [];
    this.jointRevision++;
  }

  rebuildPhysicalConstraints() {
    this.clearPhysicalConstraints();
    const groups = new Map();

    for (const bond of this.bonds) {
      if (!bond.active) continue;
      const clusterA = this.shardCluster.get(bond.a);
      const clusterB = this.shardCluster.get(bond.b);
      if (clusterA === clusterB || clusterA == null || clusterB == null) continue;
      const low = Math.min(clusterA, clusterB);
      const high = Math.max(clusterA, clusterB);
      const key = `${low}:${high}`;
      if (!groups.has(key)) groups.set(key, { a: low, b: high, bonds: [] });
      groups.get(key).bonds.push(bond);
    }

    for (const group of groups.values()) {
      const clusterA = this.clusters.get(group.a);
      const clusterB = this.clusters.get(group.b);
      if (!clusterA || !clusterB || (clusterA.anchored && clusterB.anchored)) continue;
      this.addClusterConstraint(clusterA, clusterB, group.bonds);
    }
    this.jointRevision++;
  }

  addClusterConstraint(clusterA, clusterB, bonds) {
    const Jolt = this.Jolt;
    const positionA = this.getPosition(clusterA.bodyId);
    const positionB = this.getPosition(clusterB.bodyId);
    const distance = positionA.distanceTo(positionB);
    let settings;

    if (this.jointType === 'distance') {
      settings = new Jolt.DistanceConstraintSettings();
      settings.mSpace = Jolt.EConstraintSpace_WorldSpace;
      settings.mPoint1 = new Jolt.RVec3(positionA.x, positionA.y, positionA.z);
      settings.mPoint2 = new Jolt.RVec3(positionB.x, positionB.y, positionB.z);
      const slack = Math.max(0.004, distance * this.jointSoftness);
      settings.mMinDistance = Math.max(0, distance - slack);
      settings.mMaxDistance = distance + slack;
    } else if (this.jointType === 'locked') {
      settings = new Jolt.SixDOFConstraintSettings();
      settings.mSpace = Jolt.EConstraintSpace_WorldSpace;
      const midpoint = positionA.clone().add(positionB).multiplyScalar(0.5);
      const point = new Jolt.RVec3(midpoint.x, midpoint.y, midpoint.z);
      settings.mPosition1 = point;
      settings.mPosition2 = point;
      const slack = Math.max(0, distance * this.jointSoftness);
      for (const axis of [
        Jolt.SixDOFConstraintSettings_EAxis_TranslationX,
        Jolt.SixDOFConstraintSettings_EAxis_TranslationY,
        Jolt.SixDOFConstraintSettings_EAxis_TranslationZ,
      ]) {
        if (slack <= 1e-5) settings.MakeFixedAxis(axis);
        else settings.SetLimitedAxis(axis, -slack, slack);
      }
      for (const axis of [
        Jolt.SixDOFConstraintSettings_EAxis_RotationX,
        Jolt.SixDOFConstraintSettings_EAxis_RotationY,
        Jolt.SixDOFConstraintSettings_EAxis_RotationZ,
      ]) settings.MakeFixedAxis(axis);
    } else {
      settings = new Jolt.FixedConstraintSettings();
      settings.mSpace = Jolt.EConstraintSpace_WorldSpace;
      settings.mAutoDetectPoint = true;
    }

    settings.mNumVelocityStepsOverride = 24;
    settings.mNumPositionStepsOverride = 12;
    const constraint = this.bodyInterface.CreateConstraint(
      settings,
      clusterA.bodyId,
      clusterB.bodyId,
    );
    this.system.AddConstraint(constraint);
    this.constraints.push({
      constraint,
      a: clusterA.bodyId,
      b: clusterB.bodyId,
      aCluster: clusterA.id,
      bCluster: clusterB.id,
      bonds,
      line: null,
    });
  }

  makeBondAdjacency(indices = null) {
    const allowed = indices ? new Set(indices) : null;
    const adjacency = new Map();
    const keys = allowed ?? this.shards.map((shard) => shard.index);
    for (const index of keys) adjacency.set(index, []);
    for (const bond of this.bonds) {
      if (!bond.active) continue;
      if (allowed && (!allowed.has(bond.a) || !allowed.has(bond.b))) continue;
      adjacency.get(bond.a)?.push({ bond, neighbor: bond.b });
      adjacency.get(bond.b)?.push({ bond, neighbor: bond.a });
    }
    return adjacency;
  }

  getClusterSnapshot(cluster) {
    const position = this.getPosition(cluster.bodyId);
    const q = this.bodyInterface.GetRotation(cluster.bodyId);
    const quaternion = new THREE.Quaternion(q.GetX(), q.GetY(), q.GetZ(), q.GetW());
    const linearRaw = this.bodyInterface.GetLinearVelocity(cluster.bodyId);
    const angularRaw = this.bodyInterface.GetAngularVelocity(cluster.bodyId);
    const linear = new THREE.Vector3(linearRaw.GetX(), linearRaw.GetY(), linearRaw.GetZ());
    const angular = new THREE.Vector3(angularRaw.GetX(), angularRaw.GetY(), angularRaw.GetZ());
    const worldPositions = new Map();
    for (const index of cluster.shardIndices) {
      const world = cluster.localOffsets.get(index).clone().applyQuaternion(quaternion).add(position);
      worldPositions.set(index, world);
    }
    return { position, quaternion, linear, angular, worldPositions };
  }

  findComponents(indices) {
    const adjacency = this.makeBondAdjacency(indices);
    const remaining = new Set(indices);
    const components = [];

    while (remaining.size > 0) {
      const seed = remaining.values().next().value;
      const component = [];
      const queue = [seed];
      remaining.delete(seed);
      for (let cursor = 0; cursor < queue.length; cursor++) {
        const current = queue[cursor];
        component.push(current);
        for (const edge of adjacency.get(current) ?? []) {
          if (!remaining.has(edge.neighbor)) continue;
          remaining.delete(edge.neighbor);
          queue.push(edge.neighbor);
        }
      }
      components.push(component);
    }
    return components;
  }

  splitClusters(clusterIds) {
    const work = [];
    for (const id of clusterIds) {
      const cluster = this.clusters.get(id);
      if (!cluster) continue;
      const components = this.findComponents(cluster.shardIndices);
      if (components.length <= 1) continue;
      work.push({ cluster, components, inherited: this.getClusterSnapshot(cluster) });
    }

    if (work.length === 0) {
      this.rebuildPhysicalConstraints();
      return 0;
    }

    this.clearPhysicalConstraints();
    let created = 0;
    for (const item of work) {
      this.removeCluster(item.cluster);
      for (const component of item.components) {
        this.createCluster(component, item.inherited);
        created++;
      }
    }
    this.rebuildPhysicalConstraints();
    return created;
  }

  damageShards(indices) {
    const targets = new Set(indices);
    const affectedClusters = new Set();
    let broken = 0;

    for (const index of targets) {
      const clusterId = this.shardCluster.get(index);
      if (clusterId != null) affectedClusters.add(clusterId);
    }
    for (const bond of this.bonds) {
      if (!bond.active || (!targets.has(bond.a) && !targets.has(bond.b))) continue;
      bond.active = false;
      bond.load = 0;
      bond.loadRatio = 0;
      broken++;
      const aCluster = this.shardCluster.get(bond.a);
      const bCluster = this.shardCluster.get(bond.b);
      if (aCluster != null) affectedClusters.add(aCluster);
      if (bCluster != null) affectedClusters.add(bCluster);
    }

    if (broken > 0) this.splitClusters(affectedClusters);
    return broken;
  }

  // Force needed to snap a bond of each layer. A weak hit only clears the shell,
  // a stronger one also tears the skin off the core, and a hard one breaks the
  // structural core itself.
  static BREAK_FORCE = { shell: 4, attach: 16, core: 40 };

  // Break only the bonds (touching `indices`) that the applied force can
  // overcome, so hit strength selects how deep the damage goes.
  damageShardsByForce(indices, force) {
    const targets = new Set(indices);
    const affectedClusters = new Set();
    let broken = 0;

    for (const bond of this.bonds) {
      if (!bond.active || (!targets.has(bond.a) && !targets.has(bond.b))) continue;
      if (force < (JoltPhysics.BREAK_FORCE[bond.strength] ?? JoltPhysics.BREAK_FORCE.core)) continue;
      bond.active = false;
      bond.load = 0;
      bond.loadRatio = 0;
      broken++;
      const aCluster = this.shardCluster.get(bond.a);
      const bCluster = this.shardCluster.get(bond.b);
      if (aCluster != null) affectedClusters.add(aCluster);
      if (bCluster != null) affectedClusters.add(bCluster);
    }

    if (broken > 0) this.splitClusters(affectedClusters);
    return broken;
  }

  damageShard(index) {
    return this.damageShards([index]);
  }

  detachShard(index) {
    const broken = this.damageShard(index);
    const cluster = this.clusters.get(this.shardCluster.get(index));
    if (!cluster || cluster.shardIndices.size <= 1) return broken;

    const inherited = this.getClusterSnapshot(cluster);
    const remaining = [...cluster.shardIndices].filter((shardIndex) => shardIndex !== index);
    const components = this.findComponents(remaining);

    this.clearPhysicalConstraints();
    this.removeCluster(cluster);
    this.createCluster([index], inherited);
    for (const component of components) this.createCluster(component, inherited);
    this.rebuildPhysicalConstraints();
    return broken;
  }

  countShardBonds(index) {
    return this.bonds.filter((bond) => bond.active && (bond.a === index || bond.b === index)).length;
  }

  releaseShard(index) {
    if (this.anchorShards.delete(index)) this.refreshClusterMotion(this.shardCluster.get(index));
  }

  refreshClusterMotion(clusterId) {
    const cluster = this.clusters.get(clusterId);
    if (!cluster) return;
    const anchored = [...cluster.shardIndices].some((index) => this.anchorShards.has(index));
    if (cluster.anchored === anchored) return;

    if (!anchored) {
      const inherited = this.getClusterSnapshot(cluster);
      const indices = [...cluster.shardIndices];
      this.clearPhysicalConstraints();
      this.removeCluster(cluster);
      this.createCluster(indices, inherited);
      this.rebuildPhysicalConstraints();
      return;
    }

    cluster.anchored = anchored;
    this.bodyInterface.SetMotionType(
      cluster.bodyId,
      this.Jolt.EMotionType_Static,
      this.Jolt.EActivation_DontActivate,
    );
    // Keep the body on its own collision layer (core/shell); only the motion type
    // changes when it re-anchors.
    this.bodyInterface.SetObjectLayer(cluster.bodyId, cluster.layer);
  }

  addImpulseToShard(index, impulse, point = null) {
    const cluster = this.clusters.get(this.shardCluster.get(index));
    if (!cluster || cluster.anchored) return;
    const jImpulse = new this.Jolt.Vec3(impulse.x, impulse.y, impulse.z);
    if (point) {
      this.bodyInterface.AddImpulse(
        cluster.bodyId,
        jImpulse,
        new this.Jolt.RVec3(point.x, point.y, point.z),
      );
    } else {
      this.bodyInterface.AddImpulse(cluster.bodyId, jImpulse);
    }
  }

  getClusterIdForShard(index) {
    return this.shardCluster.get(index);
  }

  calibrateStructuralLoads(safetyFactor = 2.75) {
    this.calculateStructuralLoads();
    const totalWeight = this.shards.reduce((sum, shard) => sum + shard.mass * 9.8, 0);
    const rerouteReserve = totalWeight * 0.025 * safetyFactor;
    for (const bond of this.bonds) {
      const a = this.shards[bond.a];
      const b = this.shards[bond.b];
      const localWeight = (a.mass + b.mass) * 9.8;
      bond.capacity = Math.max(localWeight * 1.2, rerouteReserve, bond.load * safetyFactor);
      bond.baselineLoad = bond.load;
      bond.overloadTime = 0;
    }
  }

  updateStructuralLoads(dt, failureDelay = 0.35) {
    if (this.anchorShards.size === 0 || this.bonds.length === 0) return 0;
    this.calculateStructuralLoads();
    const overloaded = [];

    for (const bond of this.bonds) {
      if (!bond.active) continue;
      bond.loadRatio = bond.load / Math.max(bond.capacity, 1e-5);
      if (bond.loadRatio > 1) {
        bond.overloadTime += dt * Math.min(4, bond.loadRatio);
      } else if (bond.loadRatio < 0.72) {
        bond.overloadTime = Math.max(0, bond.overloadTime - dt * 1.5);
      }
      if (bond.overloadTime >= failureDelay) overloaded.push(bond);
    }

    overloaded.sort((a, b) => b.loadRatio - a.loadRatio);
    const failed = overloaded.slice(0, 6);
    if (failed.length === 0) return 0;

    const affectedClusters = new Set();
    for (const bond of failed) {
      bond.active = false;
      const aCluster = this.shardCluster.get(bond.a);
      const bCluster = this.shardCluster.get(bond.b);
      if (aCluster != null) affectedClusters.add(aCluster);
      if (bCluster != null) affectedClusters.add(bCluster);
    }
    this.splitClusters(affectedClusters);
    return failed.length;
  }

  calculateStructuralLoads() {
    const adjacency = this.makeBondAdjacency();
    for (const bond of this.bonds) bond.load = 0;

    const depth = new Map();
    const queue = [];
    for (const index of this.anchorShards) {
      depth.set(index, 0);
      queue.push(index);
    }
    for (let cursor = 0; cursor < queue.length; cursor++) {
      const index = queue[cursor];
      const nextDepth = depth.get(index) + 1;
      for (const edge of adjacency.get(index) ?? []) {
        if (depth.has(edge.neighbor)) continue;
        depth.set(edge.neighbor, nextDepth);
        queue.push(edge.neighbor);
      }
    }

    const accumulated = new Map();
    for (const shard of this.shards) accumulated.set(shard.index, shard.mass * 9.8);
    const ordered = [...depth.keys()].sort((a, b) => depth.get(b) - depth.get(a));

    for (const index of ordered) {
      const currentDepth = depth.get(index);
      if (currentDepth === 0) continue;
      const shard = this.shards[index];
      const supporters = (adjacency.get(index) ?? [])
        .filter((edge) => depth.get(edge.neighbor) === currentDepth - 1);
      if (supporters.length === 0) continue;

      const weights = supporters.map((edge) => {
        const neighbor = this.shards[edge.neighbor];
        return 1 + Math.max(0, shard.y - neighbor.y) * 2;
      });
      const totalWeight = weights.reduce((sum, weight) => sum + weight, 0);
      const bodyLoad = accumulated.get(index) ?? 0;

      for (let i = 0; i < supporters.length; i++) {
        const edge = supporters[i];
        const share = bodyLoad * weights[i] / totalWeight;
        edge.bond.load += share;
        accumulated.set(edge.neighbor, (accumulated.get(edge.neighbor) ?? 0) + share);
      }
    }
  }

  getShardLoadRatios() {
    const ratios = new Map(this.shards.map((shard) => [shard.index, 0]));
    for (const bond of this.bonds) {
      if (!bond.active) continue;
      ratios.set(bond.a, Math.max(ratios.get(bond.a) ?? 0, bond.loadRatio));
      ratios.set(bond.b, Math.max(ratios.get(bond.b) ?? 0, bond.loadRatio));
    }
    return ratios;
  }

  releaseUnsupported() {
    const collapseThreshold = Math.max(1, Math.floor(this.initialAnchorCount * 0.2));
    if (this.anchorShards.size > 0 && this.anchorShards.size <= collapseThreshold) {
      const affected = new Set();
      for (const index of [...this.anchorShards]) {
        this.anchorShards.delete(index);
        affected.add(this.shardCluster.get(index));
      }
      for (const clusterId of affected) this.refreshClusterMotion(clusterId);
    }

    const adjacency = this.makeBondAdjacency();
    const supported = new Set(this.anchorShards);
    const queue = [...this.anchorShards];
    for (let cursor = 0; cursor < queue.length; cursor++) {
      const index = queue[cursor];
      for (const edge of adjacency.get(index) ?? []) {
        if (supported.has(edge.neighbor)) continue;
        supported.add(edge.neighbor);
        queue.push(edge.neighbor);
      }
    }

    let released = 0;
    const clustersToRelease = [];
    for (const cluster of this.clusters.values()) {
      const hasSupport = [...cluster.shardIndices].some((index) => supported.has(index));
      if (hasSupport || !cluster.anchored) continue;
      clustersToRelease.push(cluster.id);
      released += cluster.shardIndices.size;
    }
    for (const clusterId of clustersToRelease) this.refreshClusterMotion(clusterId);
    return { released, anchors: this.anchorShards.size };
  }

  step(dt, substeps = 2) {
    this.stepDuration = Math.min(dt, 1 / 20);
    this.jolt.Step(this.stepDuration, substeps);
  }

  syncShard(mesh, index) {
    const cluster = this.clusters.get(this.shardCluster.get(index));
    if (!cluster) return;
    const p = this.bodyInterface.GetPosition(cluster.bodyId);
    const q = this.bodyInterface.GetRotation(cluster.bodyId);
    const quaternion = new THREE.Quaternion(q.GetX(), q.GetY(), q.GetZ(), q.GetW());
    const offset = cluster.localOffsets.get(index).clone().applyQuaternion(quaternion);
    mesh.position.set(p.GetX() + offset.x, p.GetY() + offset.y, p.GetZ() + offset.z);
    mesh.quaternion.copy(quaternion);
  }

  getPosition(id) {
    const p = this.bodyInterface.GetPosition(id);
    return new THREE.Vector3(p.GetX(), p.GetY(), p.GetZ());
  }

  getClusterCount() {
    return this.clusters.size;
  }
}
