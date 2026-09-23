# CAD pipeline: team SolidWorks exports -> data/car.glb, steering-wheel.glb, dash.glb

The three outputs are the team's CAD and are **not in this repo** (it is
public) and **not in any build** unless `FSAE_SIM_BUNDLE_CAD=1` is set
(src-tauri/build.rs). The simulator draws its procedural car without them.

Sources live in the team vault (Helios, `SDM27/Helios/Sim`):
`chassis_and_suspension.glb`, `aero_and_chassis.glb`,
`chassis_and_engine.glb`, `chassis_and_driver_interface.glb`,
`26-08-SW-SWMA-ASM-U_V4.glb`, `AIMXSStrada1.2.STEP`.

1. `fetch.mjs` -- pull the GLBs from the vault into `raw/` (needs the
   service key in the environment, see ~/.helios-publish.env).
2. `npm i @gltf-transform/core@4 @gltf-transform/functions@4 meshoptimizer occt-import-js`
3. `node build.mjs 0.0015 0.03 out/car.glb` -- the car: every fastener
   dropped (McMaster numbers, screws, nuts, rivets, Dzus, bolts...),
   six parts SolidWorks mirrored through the ground reflected back, finishes
   fixed (mirror copies take their twin's, right-side default parts the
   left's, machined parts aluminium, springs and frame black), the side body
   wing and its missing rib rebuilt symmetric, 1.5 mm simplification, and
   the suspension split into `rig:<corner>:<role>` nodes with the OptimumK
   hardpoints (from sdm26_team_data.json) in the scene extras, for
   src/render/suspensionRig.js. The engine bay (simplified at 2x, it is
   out of sight) and the driver interface (pedals, floor, seat, head
   restraint, firewall, column, rack, dash panel) come in static; their
   metals are toned to what the sim's base-colour shading reads as metal,
   the firewall black and the exhaust titanium. Where that assembly mounts
   the wheel and the Strada goes in the scene extras as `cockpit`, and the
   renderer puts the moving wheel and the live dash there. Normals are
   stored as bytes and indices as 16-bit (7.9 MB for 324k triangles).
   The brake pedal (with the master cylinders on its arm), the throttle
   pedal and the hand clutch lever are split out as `ctl:<name>` nodes with
   their pivots and travel curves in the extras (`controls`); the renderer
   turns them by the driver's inputs (src/vehicle/clutchLever.js for the
   clutch, which has no control of its own in the sim).
4. `node build_sw.mjs 0.0002` -- steering wheel, 0.2 mm.
5. `node step2glb.mjs <sha> out/strada.glb` then `node build_dash.mjs` -- the
   Strada from its STEP (single-part GLB exports from SolidWorks come out
   empty; export parts as STEP or wrap them in an assembly).

`inspect.mjs` / `scan.mjs` dump a GLB's tree, sizes and misplaced parts.
Paths in the scripts point at the scratch folder they were written in.
