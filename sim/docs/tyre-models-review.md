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

The MF6.1.2 R20 file is the only one with an Mz fit, and it is not a shape
anyone should feel through a wheel:

- cornering stiffness (`Kya`) changes sign between 1000 and 1500 N, so Fy
  itself is wrong above ~900 N, which is the loaded outside front at 1.5 g;
- pneumatic trail barely falls with slip (`Ct` ~ 1.27 with a tiny `Bt`), so
  the wheel would not go light as the front slides -- the one thing FFB is for;
- `Dt` scales with the 0.34 m radius, inflating the trail 1.7x;
- at 300-700 N the low-slip trail evaluates to 6-12 mm, which after the radius
  correction is 4-7 mm. Raw TTC Mz/Fy for a 10" R20 is nearer 15-25 mm.

So the simulator keeps its own trail: a brush-model shape, 20 mm at 700 N
scaling with the square root of load, gone at 1.25x the peak slip. Those are
estimates and are labelled as such in `tire.js`. The right next step is a
direct Mz-vs-alpha fit from the TTC round 8 raw data at 12-14 psi, not from
these exports.
