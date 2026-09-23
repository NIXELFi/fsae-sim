# SDM26 roll and understeer gradients from test logs, compared with the sim's 4-wheel model

2026-09-22. Read-only analysis. Nothing in `C:\Users\nick5\fsae-sim` was changed. The sim runs used a snapshot of commit `8080ae3` exported to the scratchpad; see §6 for why.

## Summary

| Quantity | Real car (logs) | Sim, double-track (β) | Verdict |
|---|---|---|---|
| Roll from the springs only (shock pots), **front** | **0.51 deg/g** (95 % CI 0.49–0.53, 9 sessions) | 0.40 deg/g | Real car rolls about 27 % more |
| Roll from the springs only, **rear** | **0.64 deg/g** (95 % CI 0.61–0.66) | 0.37 deg/g | Real car rolls about 70 % more |
| Roll from the springs only, mean of both axles | 0.57 ± 0.02 deg/g | 0.39 deg/g | |
| Body roll to ground, tyres included, tyre rate 91 N/mm | front ≈ 0.82, rear ≈ 0.98 deg/g (estimate) | 0.67 deg/g (0.66 set) | Real car is higher, by 25–45 % |
| Body roll to ground, tyres included, tyre rate 52.5 N/mm | front ≈ 1.05, rear ≈ 1.24 deg/g (estimate) | — | |
| Understeer gradient K, 0.2–1.3 g, 7–12 m/s | **Not pinned down.** Robust fits give ≈ 0.0 deg/g (95 % CI −0.5 to +0.7). Ordinary least squares gives 1.45 deg/g (95 % CI 0.8–2.1). | 0.55–1.1 deg/g steady state at 8–14 m/s. The same log pipeline run on sim-driven data gives 0.75–1.0. | Inside the spread of the data. The logs can't show the sim's K is wrong. |

**Roll.** The roll result is solid: it's linear to 1.2 g and consistent across drivers and sessions. The sim's roll comes from the ARB calculator's axle stiffnesses (about 1660 N·m/deg from springs plus bars). The logs say the springs and bars together gave about 1110 N·m/deg on these days. The coil springs alone give 1126 N·m/deg. I did not find the ARB settings for 4/16 and 4/19, so this report doesn't say why the two differ (see §3.4).

**Understeer.** The understeer gradient is small and the logs can't resolve it. The data do show that the yaw gyro works. It logs millidegrees per second, and its kinematic scale checks out to within ±10 % (§4.2).

---

## 1. Data used

### 1.1 What is in the Drive "SDM26 Test Log Packs" folder, and why it wasn't used

The folder is `1KGbGN-r2-ks13JXusbrtTIO6Ho4B8O26`, with sub-folders 4.26, 4.28 and 4.30. Its `.llgx` files and CSV exports are **Link ECU internal datalogs**.

- I downloaded `4.28 CSVs/Failed Launch.csv`. It has 56 ECU channels: engine, lambda, gear, "GP Speed 1 - TransSpeed" and similar. It has no shock pots, no steering and no IMU.
- The local copies of the 5/3 session in the same format show the same thing: `Downloads\VAULT\CSVS\SDM26 (5.3.x) *.csv` and their `.llgx`. I checked the channel table inside `SDM26 (5.3.3) Matt Skidpad.llgx` and found 63 ECU channels and no chassis sensors.
- The Drive connector caps downloads at 10 MB. Because of that I couldn't open the 4.26 CSVs (8.7–29 MB) or the 4.30 CSVs (72–141 MB). Their headers are unverified, but they sit beside `.llgx` ECU logs of the same kind.
- The llgx parser the handoff mentions isn't on this machine.

### 1.2 Where the chassis channels are

The channels that "SDM26 Data Channels Review.xlsx" lists are the **DAQ logger's** channels, exported from MoTeC `.ld` to CSV at 100 Hz: FR/FL/RR/RL Shock, Steering, Lat_Accel and so on. Local copies are at `C:\Users\nick5\Downloads\VAULT\SDM26\MOTEC\testing_data\`. I copied them to `scratchpad\logs\`.

| File (scratchpad\logs) | Date / event | Driver (folder name) | Length | Used for |
|---|---|---|---|---|
| driver_tryout_4_16__43 | 4/16 driver selection | Diego + Matt.R | 236 s | roll, K |
| driver_tryout_4_16__73 | 4/16 | Matt.M | 635 s | roll, K |
| driver_tryout_4_16__34 / 36 / 37 | 4/16 | Matt.S | 262 / 59 / 57 s | roll, K |
| driver_tryout_4_16__66 / 69 / 70 | 4/16 | Sarah | 211 / 77 / 144 s | roll (66), K (66, 70). 69 and 70 carry too little lateral g for roll. |
| endurance3 | 4/19 mock endurance | Diego | 244 s | roll, K |
| driver_tryout_4_16__222 / 224 | 4/19 mock endurance | Josh | 123 / 190 s | roll, K |
| temp264 | 4/25 aero validation | — | 105 s | Not used: the car is static |

### 1.3 Channels

| Channel | Used as | Notes |
|---|---|---|
| `FLSHOCK FRSHOCK RLSHOCK RRSHOCK` (mm) | Damper length | Extension reads positive on both axles: under longitudinal g the fronts and rears move opposite ways, as pitch requires. Front (FL−FR) and rear (RL−RR) respond to lateral g with **opposite signs**, so one axle's L/R labels (or wiring) are swapped. This doesn't affect magnitudes. |
| `IMU_X_ACCEL` (g) | Lateral acceleration | Correlates at 0.94–0.98 with the shock-pot roll. `IMU_Y_ACCEL` is longitudinal. The ECU accelerometer `ENG_IMU_Y` is also lateral (inverted) and reads 0.83–0.99 of the IMU. |
| `IMU_Z_GYRO` | Yaw rate | Raw values are exact multiples of 17.5, which is the ±500 dps sensitivity of an LSM6-type gyro in **mdps**. Divided by 1000 this gives deg/s, which is plausible (up to about 125 deg/s). It checks out kinematically (§4.2). The Channels Review's "huge numbers, not useful" is a units issue, not a dead sensor. |
| `STEERING` (deg) | Steering-wheel (rim) angle | Plateaus at 121–124 deg in several runs (the known channel cap), so any \|rim\| > 115 deg is excluded. I removed a per-session offset: the median rim angle when \|ay\| < 0.04 g and \|r\| < 0.03 rad/s. It is −0.6 to −1.8 deg in most sessions and −7 deg in 34/36. |
| `GPS_SPD`, `GPS_LAT/LON` | — | Not used. One update per log and a constant value (31 or 33). Unusable. |
| `GP_SPEED` (ECU trans speed, km/h) | — | Not used for the fits. Updates at 20 Hz, has spikes to 250 km/h, and reads 1.2–1.3× the kinematic speed in low-g turns (§4.2). |
| Wheel RPM | — | Saturate at 600 / 272. Unusable, as the review says. |

## 2. Conversions and where they come from

- **Shock travel to wheel travel.** The motion ratio, damper travel per wheel travel, is **1.143 front and 1.054 rear**. I computed it from the OptimumK hardpoints (`SDM26 V1.4.6.xlsx`, transcribed in `sdm26-assetto-corsa/data/sdm26_team_data.json`). The method was a small-displacement solve of the double wishbone, pushrod and rocker, with the rocker axis normal to its three points (`scratchpad\mr.py`). It matches:
  - the Ride Roll sheet's 1.14 front, and
  - the sim's own rear wheel rate: 249.833 / 225 = 1.110 = 1.054².

  The json file lists OptimumK "motion_ratio_heave 0.88 / 0.943" and labels them "spring travel / wheel travel". Those numbers are the reciprocals (1/1.143 = 0.875, 1/1.054 = 0.949), so the **label is inverted**. `sim/tools/team_data.py` §2 squares them as spring/wheel for the aero-squat estimate, which only affects that printout. If the 0.88 / 0.943 values were used here as labelled, the roll numbers would come out 1.30× (front) and 1.11× (rear) higher.
- **Roll angle per axle.** φ = atan((ΔL / MR) / track), with the sim's tracks: 1.207 m front, 1.194 m rear.
- **Road-wheel steer.** Rim angle is mapped through `STEER_MAP_ROAD_DEG` (vehicle.rs, PCHIP through the 37 measured points, `scratchpad\common.py`). This is the mean of both road wheels, so static toe and Ackermann average out as they do in the sim.
- **Wheelbase.** L = 1.53 m (vehicle.rs).
- **IMU tilt.** A body-mounted accelerometer reads ay + g·sin(φ_body). With body roll of about 0.9 deg/g it reads about 1.6 % high, so the roll slopes are multiplied by 1.016. K barely moves.

## 3. Roll gradient

### 3.1 Method

1. Low-pass all channels at 3 Hz (2nd-order Butterworth, zero phase).
2. Keep quasi-steady samples: 0.3 s rolling standard deviation of ay < 0.04 g and of roll < 0.05 deg. This drops damper-velocity lag, kerb strikes and transients.
3. Fit roll = k·ay + c per session. The intercept absorbs static pot offsets and L/R preload.
4. Report the mean of the per-session slopes, with a t-interval across sessions. Pooling samples would overstate confidence, because samples within a session are autocorrelated.

### 3.2 Results (springs and dampers only, deg/g, tilt-corrected)

| Session | Samples | 99th-pct \|ay\| (g) | Front | Rear |
|---|---|---|---|---|
| 4_16__43 | 3599 | 0.96 | 0.491 | 0.639 |
| 4_16__73 | 9165 | 1.03 | 0.508 | 0.658 |
| 4_16__34 | 4640 | 0.65 | 0.505 | 0.631 |
| 4_16__36 | 883 | 0.91 | 0.466 | 0.608 |
| 4_16__37 | 532 | 0.98 | 0.485 | 0.651 |
| 4_16__66 | 3385 | 1.02 | 0.533 | 0.713 |
| endurance3 (4/19) | 2810 | 1.14 | 0.495 | 0.604 |
| 4_16__222 (4/19) | 1223 | 1.18 | 0.528 | 0.622 |
| 4_16__224 (4/19) | 1300 | 1.28 | 0.568 | 0.611 |
| **Mean (95 % CI)** | | | **0.509 (0.486–0.532)** | **0.637 (0.611–0.663)** |

Session 70 (0.53 / 0.64) is left out because its lateral g stays below 0.2.

- **Linear to 1.2 g.** Slopes below and above 0.6 g agree within ±0.05 in every session, and there's no sign of bump-stop contact.
- **The fit is tight.** The correlation between roll and ay is 0.94–0.99 per session.

![roll](png/roll_fit.png)

### 3.3 Tyre contribution

The pots see only suspension roll. The sim's 0.66 deg/g is body roll to the ground, tyres included. The tyres add roll equal to ΔFz(outer − inner) / (k_tyre · track).

For ΔFz I used the sim's own steady-state transfer: 594 N per g front and 660 N per g rear, from the constant-speed sweep, `simfit/sweep.csv`. These are consistent with m·ay·h/t.

| Tyre vertical rate | Tyre roll, front | Tyre roll, rear | Real body roll to ground, front | Real body roll to ground, rear |
|---|---|---|---|---|
| 91 N/mm (520 lb/in, Ride Roll sheet; what the sim uses) | 0.31 deg/g | 0.35 deg/g | **0.82 deg/g** | **0.98 deg/g** |
| 52.5 N/mm (Brakes Calculator) | 0.54 deg/g | 0.60 deg/g | **1.05 deg/g** | **1.24 deg/g** |

These to-ground numbers are estimates, not measurements. They take the logged spring roll and add a tyre term that uses the sim's lateral load transfer distribution and a tyre rate the team hasn't settled.

A model detail, for information only: in the sim, the tyre part of `roll_deg` comes from the elastic transfer alone. The springs' share (0.602 front, 0.559 rear) is applied to the gradient, so the front tyre part is (1 − 0.602) × 0.668 = 0.27 deg/g. Tyre squash from the geometric (roll-centre) transfer is not in `roll_deg`. With the full ΔFz it would be 0.31 deg/g front. The difference is small.

### 3.4 What the roll result implies about roll stiffness

Spring roll stiffness = m_s·g·arm / φ_spring. The sim values are m_s = 236.34 kg and arm = 0.2735 m, so m_s·g·arm = 634 N·m per g.

| Source | Roll stiffness, springs and bars, no tyres |
|---|---|
| Logs, axle-mean 0.57 deg/g | **≈ 1110 N·m/deg** (front-based 1245, rear-based 995) |
| Coil springs alone: 200 / 225 lb/in at MR 1.143 / 1.054 → wheel rates 45.8 / 43.8 N/mm → k·t²/2 | 582 + 545 = **1126 N·m/deg** |
| Springs + ARB at 1-1 / 1-1 (measured ARB wheel rates 24.5 / 34.2 N/mm, "Meaasured vs actual Spring rate" sheet) | ≈ 1860 N·m/deg |
| What the sim uses (ARB calculator, ≈ 47 % RSD, ~746 + 914) | ≈ 1660 N·m/deg |

The logged stiffness matches the coil springs alone to within 2 %. I did **not** find a setup sheet with the ARB settings or links for 4/16 or 4/19. `DIGITAL SDM26SETUPSHEET.xlsx` is only a template and records 200 / 225 lb/in springs. So from these data I can't tell apart three explanations:

- the bars were disconnected or slack on these days;
- ARB or chassis compliance absorbs most of the bar rate;
- the shock-pot scaling is off by the same factor.

This needs the setup record for 4/16 and 4/19, or a controlled test (Next #5 in the handoff).

The front and rear axles roll by different amounts (0.51 vs 0.64). On a rigid body both axles would read the same angle, so the gap points to frame torsional compliance, a roll motion ratio that differs from the heave one, or a pot calibration difference. The data can't separate these.

## 4. Understeer gradient

### 4.1 Method

- **Model.** On quasi-steady samples, fit δ_road = c·(L/R) + K·ay + δ₀ + δ_side·sign(ay).
  - The 0.4 s rolling standard deviations must be under 0.05 g (ay), 0.04 rad/s (r) and 3 deg of rim.
  - 0.2 g < \|ay\| and \|r\| > 0.15 rad/s. The signs of ay, r and steer must agree.
- **Radius without speed.** Steady state means V = ay/r, so **R = ay/r²**, using only the accelerometer and the yaw gyro. The real data span R = 3–40 m (median 13 m) and V = 6–16 m/s (median 9.5 m/s).
- **Blocking.** Samples are collapsed to one median point per quasi-steady segment: 222 segments from 10 sessions. Bootstrap CIs resample whole segments.
- **Estimators.** I used three:
  1. OLS on raw points;
  2. Huber-robust regression on segment medians;
  3. Theil–Sen on segment medians.

  Each was run once with c fixed at 1 and once with c free. A free c soaks up any gyro-scale error.

### 4.2 Checking the speed and radius sources

- **Low-g turns.** In turns with 0.1–0.5 g and more than 4 deg of road steer, kinematic steering gives R ≈ L/δ, so V = √(ay·L/δ) and r = V/R.
  - The measured gyro yaw rate matches this r with a median ratio of **0.96** (IQR 0.93–1.04 per session; 305 points).
  - `GP_SPEED` reads a median **1.28×** (IQR 1.18–1.65) of that V. So the gyro is trustworthy and GP_SPEED is not.
- **High-g fits.** With c free, the fits return **c = 1.00 (95 % CI 0.90–1.09)**. The gyro-derived L/R term agrees with the steering geometry across 3–40 m radii.
- **Estimator check on the sim.** I drove the sim's double-track model through random steer holds at 7–12 m/s (`simfit/src/bin/drive.rs`) and ran the identical pipeline. It gives K = 0.97 (OLS), 0.99 [0.94, 1.05] (Huber) and 0.94 (Theil–Sen), with c = 1.05. The sim's true constant-speed steady state in that speed range is 0.6–1.1 deg/g (§4.4). So on noise-free data the estimator is not badly biased.

### 4.3 Real-car K

| Estimator | ay range | K (deg/g) | 95 % CI |
|---|---|---|---|
| OLS, raw points, c = 1 | 0.2–1.4 g | 1.45 | 0.79 – 2.13 |
| OLS, raw points, c free (c = 1.07) | 0.2–1.4 g | 0.83 | 0.03 – 1.72 |
| Huber, segment medians, c = 1 | 0.2–1.4 g | 0.00 | −0.53 – 0.66 |
| Theil–Sen, segment medians | 0.2–1.4 g | 0.05 | −0.54 – 0.61 |
| Huber, c free (c = 1.00) | 0.2–1.4 g | −0.02 | −0.65 – 0.74 |
| Huber, c = 1 | 0.2–1.0 g | −0.43 | −1.16 – 0.44 |

Binned medians of δ − L/R (deg):

| ay (g) | 0.2–0.3 | 0.3–0.4 | 0.4–0.5 | 0.5–0.6 | 0.6–0.7 | 0.7–0.8 | 0.8–0.9 | 0.9–1.0 | 1.0–1.1 | 1.1–1.2 | 1.2–1.3 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| δ − L/R | 0.20 | 0.29 | 0.30 | 0.00 | 0.18 | −0.21 | 0.31 | 0.37 | 0.27 | 0.71 | 0.78 |

Individual points scatter by ±2–3 deg. The per-session OLS slopes run from −1 to +10 deg/g, and sessions 36 and 43 are outliers. Robust and least-squares estimators disagree by more than their CIs, so the result depends on the method. The honest statement is:

**The real K over 0.2–1.2 g is small. It lies between about −0.5 and +2 deg/g, and the medians rise by only about 0.5 deg between 0.3 g and 1.2 g. These logs can't pin it down more tightly.**

![understeer](png/understeer.png)

### 4.4 The sim's K, same definition

Source: constant-speed, constant-steer steady states on the double-track model at HEAD `8080ae3`, default SDM26 params (`simfit/src/bin/sweep.rs` → `simfit/sweep.csv`). K is fitted over 0.2–1.0 g.

| Speed | 6 m/s | 8 m/s | 10 m/s | 12 m/s | 14 m/s |
|---|---|---|---|---|---|
| DT K (deg/g) | 2.9* | 1.10 | 0.73 | 0.58 | 0.54 |
| Bicycle K, for reference | −0.06* | 0.55 | 0.61 | 0.61 | 0.62 |

\* At 6 m/s, 0.2–1.0 g needs R = 4–18 m. The small-angle L/R and the large-steer geometry dominate there, so these values aren't comparable.

The double-track model's median δ − L/R at 10–12 m/s is 0.45 at 0.5 g and 0.7–0.8 at 0.9 g. That is close to the real bin medians (0.0–0.3 at 0.5 g, 0.3–0.4 at 0.9 g, 0.7–0.8 at 1.2 g). The DT model's peak steady-state ay in these sweeps is 1.40–1.60 g.

Throwaway `simfit/src/main.rs` also ran a constant-radius grid with a closed-loop yaw controller. It oscillated and gave K ≈ 2.6. That output is **not valid**, has been superseded by the open-loop sweep, and should not be used.

## 5. Caveats

- **Setup unknown.** The ARB settings, tyre pressures and diff preload for 4/16 and 4/19 aren't in the files I could find. The drivers are mixed. The surface is a lot or course whose layout I don't know.
- **Sensor calibration assumed.** The shock-pot scaling (mm) and the IMU lateral scale weren't checked independently. The ECU accelerometer reads 0.83–0.99 of the IMU. If the ECU is right, every gradient here rises by that factor's inverse.
- **Real K includes steering compliance.** It also includes kinematic and compliance steer and any rack slop. The sim's steering is rigid apart from the measured map.
- **Quasi-steady ≠ steady.** Corners on these courses last about 0.5–2 s. The windows reject most transients, and the sim check shows the pipeline recovers the sim's K. But lap-average data isn't a constant-radius test.
- **Steering offset.** Offsets per session come from straight-line medians. Sessions 34 and 36 show −7 deg of rim (about 1.3 deg of road). A signed-offset term is fitted too, but an error here moves K.
- **Tyre roll is not measured.** It is modelled from the sim's ΔFz and a tyre rate the team hasn't settled (91 vs 52.5 N/mm).
- **The Drive log packs could not be fully checked.** The 4.26 and 4.30 CSVs are over the connector's 10 MB limit. The one 4.28 CSV I opened, and the local 5.3 exports, are ECU-only.

## 6. Reproduction (all in the scratchpad)

The scratchpad is `C:\Users\nick5\AppData\Local\Temp\claude\C--Users-nick5\2ece2bc5-afba-4c8a-9053-8e62680c927c\scratchpad\`.

| File | What it does |
|---|---|
| `logs\` | Copied DAQ CSVs, plus `Failed Launch.csv` from the Drive 4.28 pack |
| `common.py` | Loader, filters and the steering map |
| `roll.py` → `roll_results.json` | Per-session roll fits |
| `us.py` → `us_points.csv` | Quasi-steady points and OLS K |
| `robust.py` → `us_robust.json` | Huber and Theil–Sen fits, segment bootstrap, and the sim-drive check |
| `final.py` | Summary statistics and plots (`png\roll_fit.png`, `png\understeer.png`, `png\overview.png`) |
| `mr.py` | Motion ratios from the hardpoints |
| `accel.py` → `accel_runs.pkl` | ECU accel-run extraction; plots `png/accel_runs.png` and `png/accel_vs_sim.png` |
| `simfit\` | Throwaway Rust crate: `sweep` for steady states, `drive` for the estimator check, `probe` for diagnostics, `accel` for the 75 m harness. Run with `cargo run --release --bin sweep`. |
| `simcore_head\` | `git archive 8080ae3 native/crates/sim-core`, plus a stub workspace `Cargo.toml` |

Why the snapshot: while this ran, the repo's working tree picked up uncommitted edits in `native/crates/sim-core/src/{solver/mod.rs,vehicle.rs}` from another session, and it didn't compile (a `Telemetry` initializer was missing fields). I built against the HEAD snapshot and left the working tree untouched. For the double-track steady states, the HEAD build matches my first run on the working tree.

## 7. 75 m acceleration: real speed trace vs the sim (added on request)

### 7.1 Data

The DAQ logs contain no dedicated acceleration run. The acceleration runs are in the **Link ECU logs** of 5/3:

- `Downloads\VAULT\CSVS\SDM26 (5.3.1) Accel.csv` and `SDM26 (5.3.2) Accel.csv`, copied to `scratchpad\logs\`. Their `.llgx` are in `VAULT\Link Log Files`.
- Logging is at 1 kHz.
- The ECU log has **no accelerometer**. There is no GPS either: the DAQ logger's GPS is dead (§1.3) and wasn't logging.
- The only speed source is `GP Speed 1 - TransSpeed`. The `Driving Wheel Speed` and `Driven Wheel Speed` channels are identical copies of it. It is a **gearbox-output (driven-wheel) speed**. It does not saturate (it reaches 168 km/h in these files), but it reads wheel speed, not vehicle speed, so it includes wheelspin.

The Drive 4.28 pack has `8k Launch 4 runs.csv` (14.5 MB, over the connector limit) and `9k Launch 1 run Failed shift.csv`. I didn't use either.

### 7.2 Runs found

A candidate run is a stretch with TPS > 70 % for more than 2 s, reaching more than 50 km/h.

| Run | Launch rpm | Shifts: time (s), rpm at shift | What it is |
|---|---|---|---|
| **5.3.1 @ file time 127.5 s** | 8000 (launch control) | 1→2 at 0.75 s / 11 400; 2→3 at 2.65 s / 11 170; 3→4 at 3.75 s / 11 460 | **The only clean full run.** |
| 5.3.2 @ 91.4 s | 8000 | 1→2 at 0.68 s; 2→3 at 2.96 s | Speed stalls at 40 km/h from 1.5 to 3 s and rpm drops to 5000–6000, so it was lifted or bogged. 75 m in 5.5 s at 85 km/h. Not representative. |
| 5.3.2 @ 48 s, 5.3.1 @ 161 s and 251 s | about 2400–5800 | — | Roll-outs or aborted runs. |

About gear numbering: the ECU gear channel runs 0, 1, 2, 3 through the run and changes at the rpm drops, so channel 0 is 1st gear.

Speed-channel calibration: in-gear engine rpm per km/h of trans speed is about 212, 155, ~126 and 114. The stock CBR600RR ratios the sim uses (primary 2.111; gears 2.75 / 2.0 / 1.667 / 1.444; final drive 36/12) with a 0.2032 m tyre radius give 227, 165, 138 and 119. So the channel reads about **1.05–1.07× the rpm-implied wheel speed** (1.08× with a 0.198 m loaded radius). Both measure the same shaft, so this is calibration, not slip.

### 7.3 The clean run (5.3.1 @ 127.5 s)

![accel](png/accel_vs_sim.png)

- **0 to 0.6 s.** The trace climbs to 55 km/h at an apparent 2.7–2.9 g, and rpm goes 8000 → 13 000 in 1st. No 2WD FSAE car does 2.8 g, so this is **wheelspin** off the launch-control dump.
- **Around 1.3 s.** After the 1–2 shift the trace peaks at 67 km/h, then falls back to **55 km/h at 1.67 s** as the tyres hook up. The true vehicle speed at 1.67 s is therefore at most about 55 km/h raw, or about 52 km/h rpm-consistent. That is an average of at least about 0.9 g from rest.
- **From 1.67 s to 3.9 s.** The trace rises smoothly to 97 km/h at about 0.47 g. It is power-limited here, so slip should be small.
- **Around 4.6 s.** The driver lifts at 104 km/h raw (99 km/h rpm-consistent).

**Speed at fixed times** (km/h, raw channel ÷ 1.055):

| Time | Real, wheel speed | Sim bicycle, vehicle speed | Sim DT, vehicle speed |
|---|---|---|---|
| 0.5 s | 43 (wheelspin) | 7.5 | 7.5 |
| 1.0 s | 58 (wheelspin) | 16.1 | 16.1 |
| 1.67 s | **52** (hooked up) | 36.4 | 36.0 |
| 3.0 s | 75 | 66.7 | 66.3 |
| 3.9 s | 92 | 82.8 | 82.3 |

**75 m time and trap speed.** Distance comes from integrating speed, and the first 1.7 s aren't known exactly, so I bracket them:

- *Upper-distance bound:* integrate the spinning wheel trace. This overstates distance.
- *Lower bound:* uniform acceleration from rest to the hooked-up speed at 1.67 s. A traction-limited launch is front-loaded, so this understates distance.

| Speed scale | Distance at 1.67 s | t(75 m) | Trap at 75 m (km/h) |
|---|---|---|---|
| Raw channel | 12.8 – 22.7 m | 4.13 – 4.48 s | 97 – 103 |
| ÷ 1.055 (rpm-consistent, r = 0.2032 m) | 12.1 – 21.5 m | **4.28 – 4.63 s** | **94 – 99** |
| ÷ 1.08 (r = 0.198 m) | 11.9 – 21.0 m | 4.35 – 4.69 s | 93 – 96 |

The team's 4.2–4.4 s sits at the fast end of this bracket. That means most of the launch-phase wheel-speed distance was real, but it can't be confirmed without a vehicle-speed source. The trap speeds are wheel speed, which reads a few percent high under power.

### 7.4 The sim, same harness as `tests/double_track.rs` (HEAD 8080ae3)

The harness is: shift up above 12 000 rpm, throttle modulated on rear slip ratio 0.13 (`simfit/src/bin/accel.rs` → `simfit/sim_accel.csv`).

- **Results.** Bicycle: 75 m in **5.06 s at 97.7 km/h**. DT: **5.10 s at 95.9 km/h**.
- **Launch.** The sim leaves from about 2000 rpm, dips to 1100 rpm at 0.2 s, and reaches 16 km/h at 1 s and 36 km/h at 1.67 s. Its 52 km/h point comes at **2.18 s and 13.3 m**. That is about 0.5 s later than the real car's hooked-up 52 km/h at 1.67 s. The real car also spent at most 1.67 s and 12–21 m to get there.
- **Shifting.** The sim goes 1st→2nd→3rd within 0.1 s at 2.7–2.8 s. The loop shifts again while rpm is still above 12 000, which is a harness artifact. It also runs to about 14 400 rpm.

### 7.5 Power-limited phase, compared like for like

| Stretch | Real (÷1.055) | Sim bicycle | Sim DT |
|---|---|---|---|
| 52 → 88 km/h | **2.10 s (0.48 g)** | 2.06 s (0.49 g) | 2.23 s (0.46 g) |
| Trap at 75 m | 94 – 99 km/h (wheel speed) | 97.7 km/h | 95.9 km/h |

**Reading.** Above about 50 km/h the real car accelerates no harder than the sim, so the trace **doesn't indicate more power** than the 46 kW chassis-dyno curve. Real trap speed is about the same as the sim's, and slightly lower once the channel calibration is applied. The whole 0.5–0.8 s gap sits in the first ~1.7 s:

- the real car launches at 8000 rpm with launch control, spins the tyres, and has about 50 km/h by 1.7 s;
- the sim harness launches from about idle and needs 2.2 s to get there.

That is a launch, clutch or harness difference, not a power or top-end one.

**Caveats for this section.**

- It rests on one clean run and one driver.
- There is no accelerometer or vehicle-speed channel, so the launch-phase distance is bracketed, not measured.
- The speed channel's calibration was inferred from rpm and the stock gear ratios, not checked on a roll-out.
- I only checked the sim's `tests/double_track.rs` launch harness. The rig's launch path (launch control, clutch model) may behave differently.
