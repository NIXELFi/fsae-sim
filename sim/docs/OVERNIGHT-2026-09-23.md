# Overnight 2026-09-23: leaderboard triage, 4-wheel launch

Two fresh-eyes agents triaged everything. One read the code and data; the other rendered the real leaderboard from all 194 `sim.runs` rows and drove the sim. Leaderboard issues were fixed and released. Vehicle-dynamics findings are listed under **Your call** and nothing there was applied.

## Shipped

- **Simulator 0.7.5**: Windows and macOS are both on the feed. The Mac build came from GitHub.
  - `--model bicycle|4wheel`, so Helios can launch straight into either car.
  - An off-course lap no longer voids the whole run. In 0.7.2 to 0.7.4, one off in an endurance stint took every time in that run off the board as "modified".
  - Illegal setups are now caught however they arrive: a slot load from the menu, staging card or Numpad5, or a `.hset` import. All of these used to skip the legality check. The check now runs in `pushParams`, which every path goes through.
  - A run's header follows the car up to the green. Fixing the car on the staging card no longer files the run as modified.
  - A model swap mid-lap taints that lap.
  - The browser build stamps runs as bicycle, because that is what it actually drives.
  - Skidpad: no theoretical best. Lap start times are correct, so replay jump-to-lap, chase and sector sync work there.
  - Per-course notes for skidpad, accel and MIS.
  - The 4-wheel physics fingerprint now uses a tolerance instead of rounding. It was 0.0006 g from a rounding edge.
- **Helios 5.12.2**: tagged, and the release workflow was building at the time of writing.
  - "Launch 4-wheel" on every course's 4-wheel board.
  - A "Car model" choice on the Launch tab.
  - Physics eras are picked per course.
  - "N not ranked" explains why when you hover it.
  - Roomier tables.
  - The skidpad no longer shows a ~24 s "perfect lap".
  - The trophy banner only counts lapped courses.
  - "Time found", record holders and telemetry retention all work per model and era. Before this, 4-wheel laps could have deleted your bicycle personal-best telemetry.
  - A run with a null sector no longer vanishes.
  - Per-sector cone counts now reach the shared board.
  - Rows from simulator 0.7.x that lost their model in sharing are unranked with a reason, instead of counting as bicycle.
- **Helios 5.12.3**: PR #47.
  - Rows shared without sector cones are re-shared once with them.
  - A theoretical best slower than a real lap shows "—".
  - Tables fit a 1280 window.
- **Database**: 5 rows had lost their model. Telemetry showed `sim.vehicle_model = 2` on every sample, so I stamped them `vehicleModel: 2`:
  - Edgar's skidpad o3hq.
  - Ralf's autocross runs jsqi, aahr, nj88 and 2uiw.

## How the board reads now (live rows)

**Autocross, bicycle**

| # | Driver | Time |
|---|---|---|
| 1 | Josh | 38.317 |
| 2 | Edgar | 38.545 |
| 3 | Ralf | 39.928 |
| 4 | Daniel | 40.032 |
| 5 | Nick | 41.853 |

**Endurance, bicycle**

| # | Driver | Time |
|---|---|---|
| 1 | Edgar | 1:57.102 |
| 2 | Ralf | 1:59.110 |
| 3 | Nick | 2:00.233 |

**Skidpad, bicycle:** Edgar 4.983, the only ranked run.

**Every 4-wheel board is empty.** All of the runs there were marked not-counted by hand.

## Your call (not applied)

1. **The bicycle stayed "rev 1" through changes that move lap times.**
   - Since 0.6.13 the golden drive has been regenerated for brakes, slip and drag (999e4b8), and for the launch clutch (e9933c0).
   - Josh's 38.317 and Edgar's 38.545 are both pre-0.7.0.
   - The bicycle fingerprint is the golden drive, which starts at 15 m/s with light braking. It cannot see a launch change or a tyre fall-off change. I'd add limit fingerprints to it: a standing 75 m, a full-pedal stop and a steer ramp.
   - Bumping the bicycle to rev 2 would move today's autocross board into the archive. That's your decision.
2. **Clutch lock throws energy away.** On a 2→3 upshift the engine drops 11065 → 10407 rpm. Conserving momentum would give about 10887, so roughly 850 J is lost per shift (`powertrain.rs:737`, and the JS mirror). This is a VD fix, so I didn't touch it.
3. **The launch depends on step size.** The standing 75 m takes 4.870 s at 500 Hz and 4.823 s at 1 kHz. The rig runs at 1 kHz; the tests and fingerprints run at 500 Hz. This was already true before tonight.
4. **Edgar's hand-edited rows will change when his Helios updates.**
   - His local run files say those runs counted, so Helios re-shares them.
   - The 4-wheel ones land on the 4-wheel **rev 1** board, which is archived and not the default. You said that was fine.
   - Any bicycle ones, likely some accel runs, go back on the bicycle board, because they are legitimate.
   - If you want hard moderation, the fix is a server-side admin column that client pushes can't overwrite.
5. **Smaller known issues:**
   - Two machines can re-share each other's imported run back and forth.
   - Runs from before 0.7.2 that were modified after the green still rank; only the flags at the green are read.
   - The ghost picker offers runs from the other model without labelling them.
   - The Runs table has no model column.
   - The accel off-course card says "+20 s" (D.9 makes it a DNF).
   - The front grip scale does nothing above 1.25, but the UI allows 1.4.
   - The "4-wheel β setup" group mixes legal and illegal parameters.
6. **Hygiene:**
   - The sim CI (`build.yml`) runs no `cargo test`.
   - Sim releases are tagged from `fix/physics-review-0922`, and `main` is far behind.
   - There is no sim CHANGELOG.
   - `STATUS-2026-09-23.md` is stale: it says 0.7.1 and has the wrong macOS feed row.
7. **Still open from earlier:** the toe-test mismatch, the rear toe sign, the roll discrepancy and the steering-torque scale. See `HANDOFF-physics-2026-09-22-b.md`.

**Rotate the Supabase management token** (`sbp_…`) now that tonight's DB work is done. It sits in `~/.helios-sbp.env`.
