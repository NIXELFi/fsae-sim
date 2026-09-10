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
