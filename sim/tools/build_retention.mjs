// Which simulator builds stay in the bucket.
//
// Pure, and its own module, for two reasons. `publish_build.mjs` does its work
// at import time -- it hashes, uploads and rewrites the feed as soon as it is
// loaded -- so importing it from a test would publish a build. And this is the
// one rule in the publisher that DELETES things, which makes it the one that
// has to be testable without a network or a key.

/**
 * How many builds per platform stay in the bucket.
 *
 * The feed carries ONE entry per platform and Helios takes the first match for
 * its own (`available_build`), so it cannot be asked for an older version:
 * every build but the current one is already unreachable through the app. They
 * were kept anyway, all of them, and 18 Windows builds had quietly accumulated
 * -- 126 MB that nothing pointed at.
 *
 * Kept at all, and not zero, because rollback is republishing the feed against
 * an older url, and that works only while the object is still there. Three is
 * the current build and two to fall back to.
 */
export const KEEP_BUILDS = 3;

/** `0.10.0` is newer than `0.9.0`; anything not numeric sorts oldest. */
function versionKey(v) {
  const parts = String(v).split("-")[0].split(".").map((n) => Number.parseInt(n, 10));
  return parts.length && parts.every((n) => Number.isFinite(n)) ? parts : [-1];
}

/** Newest last, so `sort` with this puts the oldest first. */
export function compareVersions(a, b) {
  const [x, y] = [versionKey(a), versionKey(b)];
  for (let i = 0; i < Math.max(x.length, y.length); i++) {
    const d = (x[i] ?? 0) - (y[i] ?? 0);
    if (d) return d;
  }
  return 0;
}

/**
 * Which objects to retire, given everything in the bucket and one platform.
 *
 * Only ever `<platform>/<version>/<file>`. `feed.json` is not that shape and
 * neither is another platform's build, so neither can be caught by this even
 * if the pattern were to go wrong -- which matters, because deleting the feed
 * would make the simulator cease to exist for everybody at once.
 */
export function buildsToDelete(objectNames, platform) {
  const pattern = new RegExp(`^${platform}/([^/]+)/[^/]+$`);
  const mine = [];
  for (const name of objectNames) {
    const m = pattern.exec(name);
    if (m) mine.push({ name, version: m[1] });
  }
  // Newest first, then everything past the keep count.
  mine.sort((a, b) => compareVersions(b.version, a.version));
  return mine.slice(KEEP_BUILDS).map((b) => b.name);
}
