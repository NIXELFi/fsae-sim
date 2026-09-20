# Autocross setup notes, 2026-09-20

What the logs say about the stock SDM26 setup on the 2026 autocross course,
and what the vehicle model says a change would buy. Evidence first, advice
after. Every number below came out of the archived runs in
`%LOCALAPPDATA%\Helios\sim-runs` or out of `sim/src/vehicle` driven offline;
none is recalled.

## The runs

| run | driver | lap | build | notes |
|---|---|---|---|---|
| 2026-09-19 17:57 | Nick Murray | **37.627** | 0.4.0 | Nick's best, stock setup (RSD-F 51 %, bias 72 %) |
| 2026-09-20 17:49 | Nick Murray | 38.301 | 0.5.6 | Nick's best on the current build, same setup |
| 2026-09-19 23:56 | Ralf Petitt | **36.639** | shared via Helios | Ralf's best; the shared manifest carries no setup block, so it is assumed stock |
| 2026-09-19 21:01 | Ralf Petitt | 37.169 | shared via Helios | |

Both drivers on the MOZA-class wheel profile, no assists, manual shift. All
four laps are clean (no cones, no off-course).

## Where the 0.99 s goes

Nick's best against Ralf's best, resampled on distance, 20 m bins. Positive
is time Nick loses in that bin.

| distance | Nick loses | what is happening |
|---|---|---|
| 80-100 m (fast sweep) | +0.13 | Ralf 95 % throttle at 1.49 g; Nick 72 % at 1.11 g |
| 160-180 m (fast entry) | +0.13 | Ralf 21 % throttle, no brake, 1.16 g; Nick brakes 11 % and lifts to 59 % |
| 360-380 m (slowest corner) | +0.20 | both at 8-9 m/s; Nick's rear at 0.83 utilisation on 63 % throttle, Ralf's at 0.89 but he carries 4.5 km/h more |
| 440-460 m | +0.12 | Ralf 82 % throttle, Nick 66 % |
| 460-520 m (fast sweeper) | +0.42 | Ralf 93 % / 60 % / 40 % throttle at 1.26 / 1.50 / 1.66 g; Nick 75 % / 39 % / 64 % at 1.14 / 1.12 / 1.17 g |
| 280-300, 340-360, 400-440 m | -0.40 | Nick is quicker: later braking into 340, better exit at 280 and 400 |

Whole-lap numbers say the same thing:

| | Nick best | Ralf best |
|---|---|---|
| mean lateral g while cornering | 1.32 | 1.41 |
| 95th percentile lateral g | 1.64 | 1.77 |
| lap distance under brakes | 19.6 % | 11.9 % |
| brake and throttle overlapped | 11.4 % | 0.2 % |
| peak braking g | 1.43 | 1.38 |
| rear wheelspin on throttle in corners | 8.4 % of samples | 5.8 % |

**Six tenths of the gap is in three fast sections where Ralf stays at
93-100 % throttle and 1.5-1.87 g and Nick lifts to 60-75 % and 1.1-1.2 g.**
The car's own limit at that radius is 1.87 g (model, below), and Ralf's
peak on the sweeper is 1.87 g: he is driving it at the limit and the car is
not the reason Nick is not. Nick also brakes for two-thirds more of the lap
than Ralf for the same peak deceleration, and trail-brakes into corners
Ralf rolls into.

## What the car is doing at the limit

Cornering samples (|lat g| > 0.8) from Nick's best lap, by speed and by
corner phase. Utilisation is the tyre's share of its own peak force;
"US" is road-wheel angle minus the kinematic angle for the curvature the
car is actually on, so positive is understeer.

| | n | lat g | front util | rear util | slip F / R (deg) | US (deg) |
|---|---|---|---|---|---|---|
| slow corners, < 12 m/s | 84 | 1.31 | **0.92** | 0.89 | 6.5 / 4.3 | **+2.4** |
| mid, 12-18 m/s | 305 | 1.38 | 0.66 | 0.66 | 4.7 / 3.4 | +1.2 |
| fast, > 18 m/s | 363 | 1.20 | 0.33 | 0.40 | 2.3 / 1.9 | +0.1 |
| entry (on the brakes) | 163 | 1.30 | **0.72** | 0.53 | 4.6 / 3.3 | +0.9 |
| mid (neither pedal) | 147 | 1.52 | **0.78** | 0.49 | 5.7 / 3.2 | +1.3 |
| exit (> 60 % throttle) | 442 | 1.21 | 0.38 | **0.60** | 2.8 / 2.4 | +0.6 |

Of the samples where either axle is above 92 % utilisation, 45 % are
front-limited and 55 % rear-limited, and **97 % of the rear-limited ones are
on more than half throttle**. Ralf's lap reads the same (47 / 53, 97 %).

So: the car pushes in the slow and mid corners (front past its 7.3 deg peak
in 11 % of cornering samples, 2.4 deg of understeer at 9 m/s) and it is
traction-limited on the way out. Steady-state cornering never runs out of
rear grip; only the throttle does that. In the fast stuff the front is not
even near the limit for Nick, and neutral for Ralf (+0.3 deg).

Braking: the front axle is the one closer to locking on Nick's best lap
(front slip ratio 95th percentile 0.071 vs rear 0.057 under braking, front
util 0.56 vs rear 0.42) because he brakes and turns at the same time. On the
0.5.6 lap the rear reaches 0.22 slip under braking in 10 % of braking
samples, but 70 % of those are within 3 m of a downshift at ~6900 rpm at
18 m/s: that is engine braking through the downshift, not the brake bias.

## What the model says a change buys

Constant-radius test with the JS vehicle model (the same one `validate.js`
uses for the skidpad): hold the circle with the yaw-rate controller, raise
speed until it will not hold, report the lateral g at that speed. Radii
chosen from the course: 9 m is the 360-400 m hairpin, 18 m the mid-speed
corners, 27 m the fast sweeper at 460-520 m (1.5 g at 20 m/s).

| setup | 9 m | 18 m | 27 m |
|---|---|---|---|
| stock (RSD-F 51 %, aero 52.4 % front) | 1.249 g | 1.589 g | 1.869 g |
| **RSD-F 46 %** | **1.309 (+4.8 %)** | **1.637 (+3.0 %)** | **1.911 (+2.2 %)** |
| RSD-F 42 % | 1.370 (+9.7 %) | 1.734 (+9.1 %) | 1.786 (-4.4 %) |
| RSD-F 56 % | 1.190 (-4.7 %) | 1.495 (-5.9 %) | 1.786 (-4.4 %) |
| aero 56 % front | 1.249 | 1.637 (+3.0 %) | 1.911 (+2.2 %) |
| aero 48 % front | 1.249 | 1.542 (-3.0 %) | 1.786 (-4.4 %) |
| diff preload 10 / 35 N.m | 1.249 | 1.589 | 1.869 |
| diff power lock 0.42 | 1.249 | 1.589 | 1.869 |

Understeer at 92 % of the limit is 5-7 deg in every case: the model is
front-limited in steady state everywhere, which is exactly what the logs
show. Anything that gives the front grip raises the limit; the diff does
nothing to steady-state cornering (as it should not).

## Recommendations, in order

1. **Front roll stiffness 51 % -> 46 %.** This is the team's own skidpad
   blade setting (1-1/1-1), so it is a known configuration, not an
   experiment. Model: +3-5 % lateral limit in the 9-18 m corners where the
   log shows the front saturating, +2 % on the sweeper. On Nick's lap the
   car spends ~24 s above 0.8 g; 3 % more cornering speed over that is
   worth roughly 0.3 s, and the slow hairpin at 360-400 m (where Nick loses
   0.3 s to Ralf) is the corner it helps most. 42 % is faster still in the
   slow and mid corners but gives back 4 % on the sweeper and makes the rear
   the limiting axle on the throttle; do not go past 46 % without the diff
   channel below.
2. **Brake bias 72 % -> 70 %, for Nick only, on the pre-0.5.x-style braking
   he actually does.** He trail-brakes 11 % of the lap and the front is the
   axle nearer lock on entry while the car understeers (+0.9 deg). Two
   points rearward balances the axles in that phase and lets the car rotate
   on entry. Ralf does not overlap the pedals and would not feel it. Watch
   the 0.5.6 downshift slip first: with no auto-blip, downshifting at
   6900 rpm and 18 m/s puts 0.22 of slip into the rears, and moving bias
   rearward on top of that is asking for a snap. Downshift later, or one
   gear less, before touching the bias.
3. **Leave the aero balance.** The model rewards more front downforce
   because it is front-limited in steady state, but the fast-corner
   samples in both laps are slightly rear-limited on throttle (front minus
   rear utilisation -0.03 to -0.07 above 18 m/s) and Ralf reads neutral
   there (+0.3 deg). The two cancel; the lever to touch first is the roll
   stiffness.
4. **Differential: no evidence either way yet.** Until 0.5.7 the log's
   `sim.diff_locked_nm` channel recorded the clutch-locked flag and the
   desktop build sent one rear wheel's slip ratio for both, so every run to
   date shows zero diff torque and identical rear wheels. 0.5.7 records the
   real transfer torque and both rear wheels; the next session's logs can
   say whether the 8 % of exit samples with wheelspin are one wheel or two,
   which is the question the ramp angle and preload answer.
5. **The other 0.6 s is throttle in the fast sections.** The car will do
   1.87 g on the 460-520 m sweeper and Ralf takes it there at 93 % throttle;
   the same car under Nick sees 1.17 g at 64 %. No setup change delivers
   that. What might help the confidence is item 1, which also takes
   understeer out of the slow corners where the front currently gives up
   first.

## Method notes

- Distance alignment: both laps resampled on `sim.track_s_m` at 0.5 m; the
  delta is time-at-distance. Sector splits agree with the manifests.
- Speed bands and phases from `drivetrain.vehicle_speed`, `engine.aps`,
  `brake.driver_load`, utilisation from `sim.util_front` / `sim.util_rear`,
  slip from `sim.slip_front_deg` / `sim.slip_rear_deg`, understeer from
  `sim.road_wheel_deg` against `atan(L * sim.curvature)`.
- Scripts: `ax_analysis.py`, `ax_compare.py` and `setup_sweep.mjs` in the
  session scratchpad (2fba54c3). `setup_sweep.mjs` imports the vehicle
  model through `file:///` URLs and needs no build.
- The throttle-step half of the sweep (full throttle from 85 % of the limit)
  spun the model every time and is not reported; a 0.65 step from 80 % was
  too state-dependent to rank the diff settings. That needs a lap-following
  driver, which the harness does not have.
