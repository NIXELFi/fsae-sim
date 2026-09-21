// Which old builds a publish retires.
//
//     node tools/test_publish_prune.mjs
//
// This is the one rule in the publisher that deletes things, and the bucket it
// deletes from is also where `feed.json` lives -- so the cases below are as
// much about what it must NOT touch as about what it must.

import { buildsToDelete, compareVersions, KEEP_BUILDS } from "./build_retention.mjs";

let failures = 0;
let checks = 0;
function ok(cond, what) {
  checks++;
  if (cond) return;
  failures++;
  console.error(`  FAIL  ${what}`);
}
const same = (a, b, what) => ok(JSON.stringify(a) === JSON.stringify(b),
  `${what}\n          got      ${JSON.stringify(a)}\n          expected ${JSON.stringify(b)}`);

const win = (versions) => versions.map((v) => `windows/${v}/fsae-sim.exe`);

ok(KEEP_BUILDS === 3, "keeps the current build and two");

// The newest three survive; everything older goes.
same(
  buildsToDelete(win(["0.1.0", "0.2.0", "0.5.7", "0.6.3", "0.6.4", "0.6.6"]), "windows"),
  win(["0.5.7", "0.2.0", "0.1.0"]),
  "retires everything but the newest three",
);

// Version order, not lexical. Sorted as strings, "0.9.0" beats "0.10.0" and
// the publisher would delete the build it had just published.
same(
  buildsToDelete(win(["0.9.0", "0.10.0", "0.11.0", "0.2.0"]), "windows"),
  win(["0.2.0"]),
  "compares versions numerically, not as strings",
);

// Nothing spare, nothing to do.
same(buildsToDelete(win(["0.6.4", "0.6.6"]), "windows"), [],
  "leaves a bucket that is already small enough alone");

// The feed is in the same bucket and is the one object that must never go:
// without it Helios cannot find any build at all.
const mixed = [...win(["0.1.0", "0.2.0", "0.5.7", "0.6.6"]), "macos/0.1.0/fsae-sim", "feed.json"];
same(buildsToDelete(mixed, "windows"), win(["0.1.0"]),
  "never touches the feed or another platform's builds");
ok(!buildsToDelete(mixed, "windows").includes("feed.json"), "feed.json is not deletable");
same(buildsToDelete(mixed, "macos"), [], "one macOS build is under the keep count");

// A version that is not a number sorts oldest rather than throwing, so a
// hand-uploaded directory cannot wedge a publish.
same(buildsToDelete([...win(["0.6.6", "0.6.4", "0.6.3"]), "windows/scratch/fsae-sim.exe"], "windows"),
  ["windows/scratch/fsae-sim.exe"],
  "an unparseable version sorts oldest and is retired first");

ok(compareVersions("0.6.6", "0.6.4") > 0, "0.6.6 is newer than 0.6.4");
ok(compareVersions("1.0", "0.99.99") > 0, "a major bump beats any minor");

console.log(failures
  ? `test_publish_prune: ${failures} of ${checks} checks FAILED`
  : `test_publish_prune: ok (${checks} checks)`);
process.exit(failures ? 1 : 0);
