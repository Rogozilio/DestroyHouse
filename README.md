# Jolt Fracture Lab

Browser demo for fracturing `assets/Home.fbx` with Three.js and Jolt Physics.

## Run

```powershell
node server.mjs
```

Open `http://127.0.0.1:5173/`.

## GitHub Pages

1. Push the project to a public GitHub repository.
2. Open `Settings` -> `Pages`.
3. Under `Build and deployment`, choose `Deploy from a branch`.
4. Select the `main` branch and the `/(root)` folder, then save.

The site will be published at `https://<user>.github.io/<repository>/`.

## Included

- FBX triangle extraction and noise-driven Voronoi assignment.
- Four fracture fields: `voronoi`, `ridge`, `bands`, and `turbulence`.
- Adaptive surface tessellation produces 24-320 local shards even from low-poly source meshes.
- Neighboring intact shards are packed into Jolt compound rigid clusters.
- Logical per-shard bonds replace internal physics joints and split only the damaged cluster.
- Jolt `fixed`, rotation-locked `SixDOF`, and `distance` constraints connect rigid clusters.
- Zero-slack `locked` joints fix all six axes and use increased solver iterations to avoid rubbery clusters.
- Structural support propagation lets tethered shards sag and collapses islands that lose their foundation path.
- Calibrated graph loads transfer shard weight to anchors; overloaded joints fail after a delay with 100%/72% hysteresis.
- A direct hit removes every logical bond attached to the selected shard and rebuilds its local compound body.
- Mouse impulses with local, strength-limited joint breaking.
- Runtime controls for shard count, noise, joints, force, radius, and anchors.
- `smoke.html` for checking the local ES module import graph.

## UI note

Dear ImGui (`ocornut/imgui`) is a native C++ library and needs an Emscripten
backend to run in a browser. The old `imgui-js` npm package is unpublished, so
this demo uses a small immediate-mode canvas panel with ImGui-like controls.
