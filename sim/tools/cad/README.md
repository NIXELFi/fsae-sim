# CAD pipeline: team SolidWorks exports -> data/car.glb, steering-wheel.glb, dash.glb

The three outputs are the team's CAD and are **not in this repo** (it is
public) and **not in any build** unless `FSAE_SIM_BUNDLE_CAD=1` is set
(src-tauri/build.rs). The simulator draws its procedural car without them.

Sources live in the team vault (Helios, `SDM27/Helios/Sim`):
`chassis_and_suspension.glb`, `aero_and_chassis.glb`,
`26-08-SW-SWMA-ASM-U_V4.glb`, `AIMXSStrada1.2.STEP`.

1. `fetch.mjs` -- pull the GLBs from the vault into `raw/` (needs the
   service key in the environment, see ~/.helios-publish.env).
2. `npm i @gltf-transform/core@4 @gltf-transform/functions@4 meshoptimizer occt-import-js`
3. `node build.mjs 0.0015 0.04 out/car.glb` -- the car: hardware dropped,
   six parts SolidWorks mirrored through the ground reflected back, finishes
   fixed (mirror copies take their twin's, right-side default parts the
   left's, machined parts aluminium, springs and frame black), the side body
   wing and its missing rib rebuilt symmetric, 1.5 mm simplification, and
   the suspension split into `rig:<corner>:<role>` nodes with the OptimumK
   hardpoints (from sdm26_team_data.json) in the scene extras, for
   src/render/suspensionRig.js.
4. `node build_sw.mjs 0.0002` -- steering wheel, 0.2 mm.
5. `node step2glb.mjs <sha> out/strada.glb` then `node build_dash.mjs` -- the
   Strada from its STEP (single-part GLB exports from SolidWorks come out
   empty; export parts as STEP or wrap them in an assembly).

`inspect.mjs` / `scan.mjs` dump a GLB's tree, sizes and misplaced parts.
Paths in the scripts point at the scratch folder they were written in.
