"""Derive simulator parameters from the team's own SDM26 measurements.

Input is the transcription of the Sun Devil Motorsports Drive that the
Assetto Corsa mod keeps (sdm26-assetto-corsa/data/sdm26_team_data.json,
transcribed 2026-09-04; every block names its source document). Nothing here
is invented: this script only turns the team's numbers into the form
`sim/src/vehicle/params.js` wants and prints the working, so each value in
params.js can cite the line that made it.

    python3 sim/tools/team_data.py [path/to/sdm26_team_data.json]

Derivations:
  1. Aero at nominal ride height from the 2026 'Ride Height Data (BW)' CFD
     map (lbf at 15.65 m/s, rho 1.225): ClA, CdA, front downforce share.
  2. Aero balance AT SPEED: the car squats on its measured springs under its
     own downforce, so the ride heights move off nominal and the map is read
     where the car actually sits. Fixed-point iteration per speed.
  3. Brake torque share from calliper piston areas, effective radii, pad mu
     and the 54% bias bar; torque per MPa of master-cylinder pressure.
  4. Front mechanical trail: the OptimumK figure against caster x tyre radius,
     giving the kingpin-offset term the sim adds to the caster trail.
  5. Tyre load sensitivity from the PAC2002 TTC fit (PDY2 / PDY1).
  6. Pitch gradient from the measured spring rates and motion ratios with
     OptimumK's anti-dive / anti-squat. Camera only in the sim.
"""

import json
import math
import os
import sys

# The team workbook export. $SDM26_TEAM_DATA overrides; the default is where
# it lives on the machine this was written on.
DEFAULT = os.path.expanduser(os.environ.get("SDM26_TEAM_DATA", "~/Developer/sdm26-assetto-corsa/data/sdm26_team_data.json"))
IN = 0.0254
LBF = 4.448222
G = 9.81
RHO_MAP = 1.225   # the CFD map's own density (stated in the sheet's header)

# Simulator constants the derivation needs (params.js TEAM block).
MASS_KG = 267.0
WD_FRONT = 0.485
CG_M = 0.2845
WHEELBASE_M = 1.53
TYRE_R_M = 0.20
RHO_SIM = 1.162   # params.js airDensityKgM3 (Tempe, hot day)


def bilinear(grid, rows, cols, r, c):
    """grid[i][j] indexed [rows[i]][cols[j]]; rows descend, cols ascend."""
    def bracket(axis, x):
        pts = sorted(range(len(axis)), key=lambda k: axis[k])
        vals = [axis[k] for k in pts]
        x = min(max(x, vals[0]), vals[-1])
        for k in range(len(vals) - 1):
            if vals[k] <= x <= vals[k + 1]:
                t = (x - vals[k]) / (vals[k + 1] - vals[k])
                return pts[k], pts[k + 1], t
        return pts[-1], pts[-1], 0.0
    i0, i1, ti = bracket(rows, r)
    j0, j1, tj = bracket(cols, c)
    g = grid
    top = g[i0][j0] * (1 - tj) + g[i0][j1] * tj
    bot = g[i1][j0] * (1 - tj) + g[i1][j1] * tj
    return top * (1 - ti) + bot * ti


def main(path):
    with open(path, encoding="utf-8") as fh:
        d = json.load(fh)

    print(f"source: {path}")
    print(f"  {d['_about']}\n")

    # ---- 1. aero at nominal -------------------------------------------
    am = d["aero_maps"]
    bw = am["bw_2026_lbf"]
    cols = am["ride_height_delta_in"]          # front RH deltas, ascending
    rows = bw["rear_rh_rows"]                  # rear RH deltas, descending
    v_ref = am["reference_speed_m_s"]
    q_ref = 0.5 * RHO_MAP * v_ref * v_ref
    i0, j0 = rows.index(0), cols.index(0)
    df0, dr0, pf0 = bw["total_df"][i0][j0], bw["total_drag"][i0][j0], bw["pct_front"][i0][j0]
    cla0, cda0 = df0 * LBF / q_ref, dr0 * LBF / q_ref
    print("1. AERO, 2026 CFD ride-height map (BW), nominal ride height")
    print(f"   downforce {df0} lbf, drag {dr0} lbf at {v_ref} m/s, rho {RHO_MAP}")
    print(f"   ClA = {cla0:.3f} m^2   CdA = {cda0:.3f} m^2   front share {pf0/100:.3f}")
    m25 = am["map_2025"]
    print(f"   (2025 half-car sheet at nominal: Cl {m25['Cl'][i0][j0]}, Cd {m25['Cd'][i0][j0]}, "
          f"front {m25['pct_front'][i0][j0]}% -- what params.js used to carry as '2026')\n")

    # ---- 2. aero balance at speed --------------------------------------
    sd = d["springs_dampers"]
    km = d["kinematics_measured"]
    mr_f, mr_r = km["front"]["motion_ratio_heave"], km["rear"]["motion_ratio_heave"]
    kw_f = sd["spring_rate_front_N_per_mm"] * mr_f ** 2 * 1000   # N/m at the wheel
    kw_r = sd["spring_rate_rear_N_per_mm"] * mr_r ** 2 * 1000
    print("2. AERO BALANCE AT SPEED (car squats on its measured springs)")
    print(f"   wheel rates: front {kw_f/1000:.2f} N/mm (coil {sd['spring_rate_front_N_per_mm']} x MR {mr_f}^2), "
          f"rear {kw_r/1000:.2f} N/mm (coil {sd['spring_rate_rear_N_per_mm']} x MR {mr_r}^2)")
    print("   tyre vertical stiffness is not on Drive; springs only (tyre would add ~20% travel)")
    print("   speed  q(sim rho)  dz_f    dz_r    ClA    CdA   front")
    fracs = []
    for v in (10, 15, 20, 25, 30):
        q = 0.5 * RHO_SIM * v * v
        zf = zr = 0.0
        for _ in range(30):
            cla = bilinear(bw["total_df"], rows, cols, zr, zf) * LBF / q_ref
            pf = bilinear(bw["pct_front"], rows, cols, zr, zf) / 100
            down = q * cla
            zf = -down * pf / kw_f / IN          # inches, negative = lower
            zr = -down * (1 - pf) / kw_r / IN
        cda = bilinear(bw["total_drag"], rows, cols, zr, zf) * LBF / q_ref
        fracs.append(pf)
        print(f"   {v:>3} m/s  {q:7.1f} Pa  {zf:+.2f} in {zr:+.2f} in  {cla:.3f}  {cda:.3f}  {pf:.3f}")
    print(f"   mean front share 10-30 m/s: {sum(fracs)/len(fracs):.3f}; the map is flat to "
          f"+/-{(max(fracs)-min(fracs))/2:.3f} over the travel the car actually uses")
    pm = am["pitch_map_bw_lbf"]
    k = pm["pitch_deg"].index(0)
    print(f"   pitch map at 0 deg: {pm['pct_front'][k]}% front (same nominal, second sweep)\n")

    # ---- 3. brakes ------------------------------------------------------
    # Workbook BRAKES block plus the Drive 'SDM26 Brakes Calculator (Ideal
    # Brake Bias)' sheet: Brembo P4.24 front (4 x 24 mm pistons, 1809.6 mm^2
    # TOTAL, i.e. both sides), P2.24 rear (2 x 24 mm, 904.8 mm^2 total),
    # Tilton 78-625 master cylinders (15.875 mm, 197.9 mm^2), pedal ratio
    # 3.5, pedal efficiency 0.8, max working pressure 70 bar, bias bar 54%
    # front by force (OptimumK 'Brake Bias 54.0'; Tyler 2026-04-11: "54% fr").
    # Clamp force on the disc is mu x pressure x (total piston area), the
    # two faces being the two halves of that total.
    b = d["brakes"]
    p_max = 70e5                         # Pa, calculator's max working pressure
    bias = b["bias_bar_front_pressure_fraction"]
    a_mc = math.pi / 4 * 0.015875 ** 2   # m^2, Tilton 78-625
    pedal_ratio, pedal_eff = 3.5, 0.8
    per_pa_f = b["pad_mu"] * b["front"]["piston_area_mm2"] * 1e-6 * b["front"]["effective_radius_mm"] * 1e-3
    per_pa_r = b["pad_mu"] * b["rear"]["piston_area_mm2"] * 1e-6 * b["rear"]["effective_radius_mm"] * 1e-3
    # Bias bar splits pedal FORCE; both cylinders have the same bore, so the
    # line pressures split the same way.
    share_f = 2 * per_pa_f * bias / (2 * per_pa_f * bias + 2 * per_pa_r * (1 - bias))
    # Front line at its max working pressure, rear at (1-bias)/bias of it.
    t_max = 2 * per_pa_f * p_max + 2 * per_pa_r * p_max * (1 - bias) / bias
    f_pedal_max = p_max * a_mc / (bias * pedal_ratio * pedal_eff)
    per_n_pedal = pedal_ratio * pedal_eff / a_mc * (2 * per_pa_f * bias + 2 * per_pa_r * (1 - bias))
    print("3. BRAKES (workbook BRAKES block + Drive 'SDM26 Brakes Calculator (Ideal Brake Bias)')")
    print(f"   pad mu {b['pad_mu']}, front {b['front']['piston_area_mm2']} mm^2 total at r {b['front']['effective_radius_mm']} mm, "
          f"rear {b['rear']['piston_area_mm2']} mm^2 at r {b['rear']['effective_radius_mm']} mm")
    print(f"   bias bar {bias:.2f} front by force -> front TORQUE share = {share_f:.3f} "
          f"(calculator's own 'actual brake force distribution': 0.713 with r 78.1/70.1 mm)")
    print(f"   torque per N of pedal force = {per_n_pedal:.3f} N.m/N (ratio {pedal_ratio}, eff {pedal_eff}, 5/8 in cylinders)")
    print(f"   at the 70 bar max working pressure (front line): {t_max:.0f} N.m at the wheels, "
          f"{f_pedal_max:.0f} N = {f_pedal_max/4.448:.0f} lbf on the pedal")
    print(f"   calculator's 'common' pedal force 489 N -> {489*per_n_pedal:.0f} N.m; rules max 2000 N -> {2000*per_n_pedal:.0f} N.m")
    fz_f_1g = WD_FRONT + 1.0 * CG_M / WHEELBASE_M
    fz_f_15g = WD_FRONT + 1.5 * CG_M / WHEELBASE_M
    print(f"   ideal front share for lock-together: {fz_f_1g:.3f} at 1.0 g, {fz_f_15g:.3f} at 1.5 g "
          f"(48.5% static, CG {CG_M*1000:.1f} mm, wheelbase {WHEELBASE_M} m)")
    print(f"   all-four lock at 1.5 g needs {MASS_KG*G*1.5*TYRE_R_M:.0f} N.m = {MASS_KG*G*1.5*TYRE_R_M/per_n_pedal:.0f} N on the pedal")
    print("   the pedal force a driver actually reaches is not measured, so the sim's max torque is the 70 bar figure\n")

    # ---- 4. steering geometry ------------------------------------------
    f = km["front"]
    caster = f["caster_deg"]
    trail_ok = f["mech_trail_in"] * IN
    caster_trail = TYRE_R_M * math.tan(math.radians(caster))
    print("4. STEERING (OptimumK 'Designed vs Actual Kinematics', 2026-06-27, Actual column)")
    print(f"   caster {caster} deg, KPI {f['kpi_deg']} deg, scrub {f['scrub_radius_in']*IN*1000:.1f} mm, "
          f"mechanical trail {trail_ok*1000:.2f} mm, ratio {km['steering_ratio']}")
    print(f"   sim trail model: R tan(caster) = {caster_trail*1000:.2f} mm at R = {TYRE_R_M} m")
    print(f"   -> kingpinOffsetTrailM = {trail_ok - caster_trail:+.5f} m to land on OptimumK's trail")
    print(f"   rack: {d['steering']['rack_travel_in_per_deg']} in per rim deg; Ackermann {d['steering']['ackermann_deg']} deg")
    print("   full-lock road-wheel angle is not in the export (rack stops not measured)\n")

    # ---- 5. tyre load sensitivity ----------------------------------------
    t = d["tyre_pac2002"]
    lat, lon = t["lateral"], t["longitudinal"]
    sy, sx = -lat["PDY2"] / lat["PDY1"], -lon["PDX2"] / lon["PDX1"]
    print("5. TYRE (PAC2002 TTC fit, Hoosier 16x7.5-10 R20, FNOMIN 700 N)")
    print(f"   peak mu at 700 N: lateral {lat['PDY1']*lat['LMUY']:.3f}, longitudinal {lon['PDX1']*lon['LMUX']:.3f} (belt, unscaled)")
    print(f"   mu(Fz) = PDY1 + PDY2 dfz -> lateral falls {sy:.3f} per 100% load; longitudinal {sx:.3f}")
    print(f"   sim tireLoadSensitivity uses the same linear form: {sy:.2f} lateral (was 0.15 EST)")
    print(f"   peak slip angle by load (deg): {t['peak_slip_angle_deg_by_load']}")
    print("   no aligning-moment (Mz) coefficients in the block: pneumatic trail stays an estimate\n")

    # ---- 6. pitch gradient -----------------------------------------------
    a = WHEELBASE_M * (1 - WD_FRONT)
    bb = WHEELBASE_M * WD_FRONT
    m_pitch = MASS_KG * G * CG_M                          # N.m per g
    k_pitch = 2 * kw_f * a * a + 2 * kw_r * bb * bb       # N.m/rad, springs only
    raw = math.degrees(m_pitch / k_pitch)
    ad, asq = f["anti_dive_pct"] / 100, km["rear"]["anti_squat_pct"] / 100
    print("6. PITCH GRADIENT (springs + measured MRs, OptimumK anti-dive/anti-squat)")
    print(f"   {m_pitch:.0f} N.m per g over {k_pitch:.0f} N.m/rad = {raw:.3f} deg/g springs only")
    print(f"   anti-dive {ad:.3f} front, anti-squat {asq:.3f} rear -> braking dive ~{raw*(1-0.5*(ad+km['rear']['anti_lift_pct']/100)):.2f} deg/g")
    print("   tyre compliance would add ~15-20%; camera only in the sim\n")

    # ---- inertia and masses, straight from the workbook ---------------
    mi = d["mass_inertia"]
    print("MASS / INERTIA ('SDM26 Full-Vehicle Sim Parameters.xlsx' BODY block)")
    print(f"   Izz {mi['Izz_yaw_kg_m2']} kg.m^2 at {mi['mass_kg_as_listed']} kg listed (Ixx {mi['Ixx_roll_kg_m2']}, Iyy {mi['Iyy_pitch_kg_m2']})")
    ab = a * bb
    print(f"   dynamic index k^2/(ab) = {mi['Izz_yaw_kg_m2']/MASS_KG/ab:.3f} at {MASS_KG} kg")
    print(f"   unsprung per corner: front {sd['unsprung_mass_kg']['front']} kg, rear {sd['unsprung_mass_kg']['rear']} kg")
    print(f"   wheel spin inertia: front {sd['wheel_spin_inertia_kg_m2']['front']}, rear {sd['wheel_spin_inertia_kg_m2']['rear']} kg.m^2")
    print(f"   loaded wheel-centre height {sd['wheel_centre_height_loaded_mm']} mm (OptimumK hardpoints say 8.0 in = 203.2 mm)")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else DEFAULT)
