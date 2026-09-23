// The drawn driver controls: the hand clutch's position from the clutch's
// slip (src/vehicle/clutchLever.js), checked on the pull-away of a real
// 4-wheel run (xu5e, which launched without launch control).
import { clutchLever } from "../src/vehicle/clutchLever.js";

let fails = 0;
const ok = (name, cond, detail = "") => {
  if (!cond) fails++;
  console.log(`  [${cond ? "PASS" : "FAIL"}] ${name}${detail ? "  " + detail : ""}`);
};
// rpm, rear wheel rpm, gear, m/s -- rows from the run's log.
const pull = [
  [2150, 5.4, 1, 0.068], [2952, 27.7, 1, 0.516], [3974, 68.2, 1, 1.328],
  [5090, 150.0, 1, 2.851], [5575, 320.8, 1, 3.566], [7837, 450.7, 1, 5.003],
];
const lever = pull.map(([rpm, w, g, v]) => clutchLever(rpm, w, g, v, false));
ok("pulling away: in at the start, let out as the car picks up", lever[0] > 0.9 && lever.every((x, i) => i === 0 || x <= lever[i - 1] + 1e-9),
  lever.map((x) => x.toFixed(2)).join(" -> "));
ok("out once the clutch has locked (0.6 s in)", lever[4] < 0.02 && lever[5] < 0.02);
ok("stopped in gear with the engine running: pulled right in", clutchLever(1800, 0, 1, 0, false) === 1);
ok("launch control armed: pulled in", clutchLever(9000, 0, 1, 0, true) === 1);
ok("neutral: out", clutchLever(1800, 0, 0, 0, false) === 0);
ok("a shift at speed: out (nobody clutch-shifts)", clutchLever(9000, 400, 3, 25, false) === 0);
ok("rear wheelspin (wheels ahead of the engine): out", clutchLever(9000, 700, 1, 8, false) === 0);

console.log(fails ? `\n${fails} FAILED` : "\nALL CHECKS PASSED");
if (fails) process.exit(1);
