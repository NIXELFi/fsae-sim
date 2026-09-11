# Tyre model files reviewed for the force feedback (2026-09-10)

Five OptimumT `.tir` exports were supplied for the Hoosier 10" tyres. None are
in the repository. Evaluated at 14 psi over 300-1500 N with a straight MF5.2 /
MF6.1.2 implementation, pure slip, zero camber.

| File | Fit | Aligning (Mz) | Verdict |
|---|---|---|---|
| MF5-2_R1 10psi Hzr16x75-10 R20 - 7in Rim | MF5.2 | all zero | Fy only. Peak mu 3.4 raw. |
| MF5-2_R1 14psi Hzr16x75-10 R20 - 7in Rim | MF5.2 | all zero | Fy only. Peak mu 3.0 raw. |
| MF5-2_R1 14psi ... (1) | MF5.2 | all zero | Different fit of the same data (mu 2.6). |
| MF612-Hoosier 16x7_5-10 R20 7in Rim | MF6.1.2 | present | See below. Not usable for trail. |
| MF62- Hoosier 18x6-10 | MF6.1.2 | all zero | Wrong tyre size for SDM26 anyway. |

All five share the same header defects: `FNOMIN = -6000` (or +6000) N when the
tyre operates at 200-1200 N, so every `dfz` term extrapolates a long way from
the fit's nominal; `UNLOADED_RADIUS = 0.34` m on a 0.2 m tyre; `WIDTH = 0`;
`NOMPRES = 180 kPa` against a 97 kPa test pressure. Friction peaks are raw
TTC belt values (2.6-3.4) with no road scaling.

The MF6.1.2 R20 file is the only one with an Mz fit, and the fit is broken
rather than merely rough. Evaluated straight from MF6.1 (4.E26 onward):

- `PKY4 = 38.7` (physically ~2), so `Kya = PKY1 Fz0 sin(PKY4 atan(Fz / (PKY2 Fz0)))`
  crosses zero at about 1640 N even at the fit's nominal pressure, and at
  14 psi `(1 + PPY2 dpi) = -0.64` flips the sign of the whole argument. The
  cornering stiffness therefore has the wrong sign over most of the range and
  changes sign inside it; Fy itself is wrong above ~900 N, which is the loaded
  outside front at 1.5 g. Whether Fz is stored negative (SAE) changes nothing:
  every term is a ratio to `FNOMIN`.
- The aligning block is worse: `SHt = QHZ1 + QHZ2 dfz` is a horizontal shift
  of -56 to -66 DEGREES at 300-1500 N, `Dt` is negative (-15 to -65 mm), and
  the `Et` polynomial runs from -33 to +31 and is clamped on one side only.
  Any "trail" read off this curve is garbage output, not a property of the
  tyre.
- `Dt` also scales with the 0.34 m `UNLOADED_RADIUS`, 1.7x the real tyre.

So the simulator keeps its own trail: a brush-model shape, 20 mm at 700 N
scaling with the square root of load, gone at 1.25x the peak slip. Those are
estimates and are labelled as such in `tire.js`. The right next step is a
direct Mz-vs-alpha fit from the TTC round 8 raw data at 12-14 psi, not from
these exports.

## Update 2026-09-11: trail fitted from the raw TTC data

The raw Calspan data for this exact tyre is on the team's Drive (FSAE TTC
Round 9, `B2356run6.mat`: Hoosier 43075 16x7.5-10 R20 on the 7 in rim), and
`sim/tools/ttc_trail.py` now fits the simulator's trail model straight to its
Mz channel at 12 psi, zero camber and 25 mph, no .tir in between. Trail is
taken sample by sample as -Mz/Fy where |Fy| is clear of the noise floor.

Near-zero-slip trail by load: 16 / 21 / 27 / 35 / 43 mm at 222 / 445 / 667 /
890 / 1112 N. It falls to about a fifth of that by 8-9 deg and is gone
between 12 and 13 deg.

Least squares of `t = t0 (Fz/700)^n (1 - min(s/s0, 1))^2`, with `s`
normalised to the model's 8.5 deg peak, over 7359 samples: `t0 = 39.2 mm`
with the model's square-root load law (a free exponent fits 0.60; the rms is
the same to 0.03 mm), `s0 = 1.845` (zero trail at 15.4 deg). The rms is 8 mm,
which is the (1 - x)^2 shape being an approximation, not noise: the fit sits
~15% above the data in the 0-2 deg linear range and matches from 3 deg out.

So the 20 mm / 1.25 estimate is replaced by 39.2 mm / 1.845 in `tire.js` and
`tyre.rs`. The measured trail is nearly double the guess, and it survives
well past the force peak, so the wheel goes light more gradually than the
model previously assumed. The .mat files stay off the repo (consortium data);
only the fitted constants are committed.
