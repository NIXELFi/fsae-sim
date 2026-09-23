// Where the driver's hand clutch is, for drawing it.
//
// The sim has no clutch control: the drivetrain slips and locks the clutch
// itself (powertrain.clutchCapacity / sim-core `clutch_capacity`), and the
// 4-wheel engine does not report it. What any log does have is the engine
// speed and the rear wheels' speed, and the clutch's slip is the difference
// once the wheels are taken back up through the gearbox. On the real car the
// hand is on the lever exactly while that slip is there: pulled right in with
// the car stopped in gear, let out as it pulls away, out once the clutch
// locks. So the lever is drawn from the slip -- which shows a start without
// launch control as well as one with it -- and held in while launch control
// is armed (the clutch is in; the engine sits on the limiter).
//
// Renderer-only. Nothing the physics or the timing reads depends on it.
import { SDM26, totalReduction } from "./params.js";

/** Above this road speed the lever stays out: nobody clutch-shifts (the
 *  drivetrain opens the clutch for a shift itself, and that is not a hand). */
const MOVING_OFF_MPS = 12;

/**
 * @param rpm            engine speed
 * @param rearWheelRpm   mean of the two rear wheels
 * @param gear           1-based as logged; 0 is neutral
 * @param speed          road speed, m/s
 * @param launchHeld     launch control armed
 * @returns 0 (lever out, clutch engaged) .. 1 (pulled right in)
 */
export function clutchLever(rpm, rearWheelRpm, gear, speed, launchHeld, v = SDM26) {
  if (launchHeld) return 1;
  if (!(gear >= 1) || speed > MOVING_OFF_MPS || !(rpm > 0)) return 0;
  const ratio = totalReduction(v, Math.round(gear) - 1);
  if (!ratio) return 0;
  const slip = (rpm - Math.abs(rearWheelRpm) * ratio) / rpm;
  return Math.max(0, Math.min(1, slip));
}
