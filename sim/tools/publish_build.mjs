// Publish a simulator build so Helios can offer it as a download.
//
//   node sim/tools/publish_build.mjs --version 0.2.0 [--notes "..."] [--dry-run]
//
// Needs, in the environment:
//   SUPABASE_URL          https://<ref>.supabase.co
//   SUPABASE_SERVICE_KEY  a service-role key (storage writes are not public)
//
// What it does, in order:
//   1. hashes the built executable
//   2. uploads it to  sim/<platform>/<version>/fsae-sim.exe
//   3. rewrites  sim/feed.json  to point at it
//
// The feed is what Helios reads. It is deliberately a plain public JSON file
// with a SHA-256 in it rather than anything cleverer: Helios verifies the hash
// before it installs, so the transport only has to be reachable, not trusted.
//
// The bucket is created on the first publish if it is not there, public-read,
// so there is no dashboard step. Creating it needs the service key the upload
// already needs, and a bucket that exists is left alone.
//
// Decoupling this from the Helios release is the whole point. A new simulator
// is a new row in this feed, not a new Helios installer for fifty people.

import fs from "node:fs";
import path from "node:path";
import crypto from "node:crypto";
import { fileURLToPath } from "node:url";

const HERE = path.dirname(fileURLToPath(import.meta.url));
const REPO = path.join(HERE, "..", "..");

const argv = process.argv.slice(2);
const arg = (name, dflt = null) => {
  const i = argv.indexOf(`--${name}`);
  return i >= 0 && argv[i + 1] && !argv[i + 1].startsWith("--") ? argv[i + 1] : dflt;
};
const has = (name) => argv.includes(`--${name}`);

const VERSION = arg("version");
const NOTES = arg("notes", "");
const DRY = has("dry-run");
const BUCKET = arg("bucket", "sim");

if (!VERSION) {
  console.error("publish_build: --version is required (e.g. --version 0.2.0)");
  process.exit(1);
}
if (!/^[A-Za-z0-9._-]{1,64}$/.test(VERSION)) {
  console.error(`publish_build: "${VERSION}" is not usable as a version (it becomes a directory name)`);
  process.exit(1);
}

// Only Windows is built here today; the field exists so the feed can carry
// more than one platform when there is one.
const PLATFORM = process.platform === "win32" ? "windows"
  : process.platform === "darwin" ? "macos" : "linux";
const EXE_NAME = PLATFORM === "windows" ? "fsae-sim.exe" : "fsae-sim";
const exePath = path.join(REPO, "sim", "src-tauri", "target", "release", EXE_NAME);

if (!fs.existsSync(exePath)) {
  console.error(`publish_build: no build at ${exePath}`);
  console.error("  cargo build --release --manifest-path sim/src-tauri/Cargo.toml");
  process.exit(1);
}

const bytes = fs.statSync(exePath).size;
const sha256 = crypto.createHash("sha256").update(fs.readFileSync(exePath)).digest("hex");

const SUPABASE_URL = (process.env.SUPABASE_URL || "").replace(/\/+$/, "");
const KEY = process.env.SUPABASE_SERVICE_KEY || "";

const objectPath = `${PLATFORM}/${VERSION}/${EXE_NAME}`;
const publicUrl = `${SUPABASE_URL}/storage/v1/object/public/${BUCKET}/${objectPath}`;

const entry = {
  version: VERSION,
  platform: PLATFORM,
  url: publicUrl,
  sha256,
  bytes,
  notes: NOTES || null,
  published: new Date().toISOString(),
};

console.log(`build   ${exePath}`);
console.log(`size    ${(bytes / 1048576).toFixed(2)} MB`);
console.log(`sha256  ${sha256}`);
console.log(`target  ${BUCKET}/${objectPath}`);

if (DRY) {
  console.log("\n--dry-run: nothing uploaded. The feed entry would be:");
  console.log(JSON.stringify({ builds: [entry] }, null, 2));
  process.exit(0);
}

if (!SUPABASE_URL || !KEY) {
  console.error("\npublish_build: set SUPABASE_URL and SUPABASE_SERVICE_KEY, or pass --dry-run");
  process.exit(1);
}

/**
 * Make sure the bucket exists, and is public.
 *
 * There used to be a sentence in the README asking whoever published to go and
 * click through the Supabase dashboard first. That is a step that gets done
 * once, by one person, and is then a mystery to everybody else -- and the
 * failure when it has not been done is a 400 from an upload, which reads like
 * a broken script rather than a missing bucket.
 *
 * PUBLIC on purpose. What is in here is a build of a driving simulator, the
 * feed names its SHA-256, and Helios verifies that hash before a byte becomes
 * executable -- so the transport only has to be reachable, not trusted. A
 * private bucket would mean shipping a credential to every rig that wants to
 * download a game.
 */
async function ensureBucket() {
  const head = await fetch(`${SUPABASE_URL}/storage/v1/bucket/${BUCKET}`, {
    headers: { Authorization: `Bearer ${KEY}`, apikey: KEY },
  });
  if (head.ok) {
    const info = await head.json().catch(() => ({}));
    if (info.public === false) {
      throw new Error(
        `the "${BUCKET}" bucket exists but is private; Helios downloads it without ` +
        `credentials, so make it public in the dashboard (Storage -> ${BUCKET} -> ` +
        `Settings -> Public bucket)`,
      );
    }
    console.log(`bucket  ${BUCKET} (exists, public)`);
    return;
  }
  if (head.status !== 404 && head.status !== 400) {
    throw new Error(`could not check the "${BUCKET}" bucket: ${head.status} ${await head.text()}`);
  }
  const made = await fetch(`${SUPABASE_URL}/storage/v1/bucket`, {
    method: "POST",
    headers: {
      Authorization: `Bearer ${KEY}`,
      apikey: KEY,
      "Content-Type": "application/json",
    },
    body: JSON.stringify({ id: BUCKET, name: BUCKET, public: true }),
  });
  if (!made.ok) {
    const body = await made.text();
    // Two publishes at once, or a bucket that was there after all.
    if (made.status === 409) {
      console.log(`bucket  ${BUCKET} (already there)`);
      return;
    }
    throw new Error(
      `could not create the "${BUCKET}" bucket: ${made.status} ${body}\n` +
      `  (this needs a SERVICE-ROLE key, not the anon key)`,
    );
  }
  console.log(`bucket  ${BUCKET} (created, public)`);
}

async function upload(objPath, body, contentType) {
  const url = `${SUPABASE_URL}/storage/v1/object/${BUCKET}/${objPath}`;
  const res = await fetch(url, {
    method: "POST",
    headers: {
      Authorization: `Bearer ${KEY}`,
      "Content-Type": contentType,
      // Replace rather than fail when the same version is published twice.
      "x-upsert": "true",
    },
    body,
  });
  if (!res.ok) {
    throw new Error(`upload ${objPath}: ${res.status} ${await res.text()}`);
  }
}

/**
 * The feed as it stands, or an explanation of why it could not be read.
 *
 * The distinction matters more than it looks. The feed is REWRITTEN below with
 * this platform's entry replaced and every other platform's carried over, so
 * "the feed is empty" and "I could not read the feed" have to be different
 * answers: a flaky network, a 500 from storage or a half-written feed.json all
 * used to come back as an empty feed, and publishing a Windows build would
 * then silently delete the macOS and Linux entries. The person publishing sees
 * a successful Windows release; a Mac user sees the simulator cease to exist.
 *
 * "The feed has not been written yet" is the one honest empty, and Supabase
 * does NOT say that with a 404. A missing object in a public bucket comes back
 * as HTTP 400 carrying `{"statusCode":"404","error":"not_found",...}` in the
 * body -- so testing the HTTP status alone made the very first publish into a
 * brand-new bucket look like a read failure, and refuse itself. The body is
 * where the real answer is; the status is a wrapper.
 */
async function readFeed() {
  let res;
  try {
    res = await fetch(`${SUPABASE_URL}/storage/v1/object/public/${BUCKET}/feed.json`);
  } catch (err) {
    return { error: `could not reach storage: ${err?.message ?? err}` };
  }
  if (!res.ok) {
    const body = await res.text().catch(() => "");
    let inner = null;
    try { inner = JSON.parse(body); } catch { /* not JSON; fall through */ }
    const missing = res.status === 404 ||
      String(inner?.statusCode) === "404" ||
      inner?.error === "not_found" ||
      inner?.code === "NoSuchKey";
    if (missing) return { feed: { builds: [] }, fresh: true };
    return {
      error: `feed.json came back ${res.status} ${res.statusText}${body ? `: ${body.slice(0, 200)}` : ""}`,
    };
  }
  let json;
  try {
    json = await res.json();
  } catch (err) {
    return { error: `feed.json is not valid JSON: ${err?.message ?? err}` };
  }
  if (!Array.isArray(json?.builds)) return { error: "feed.json has no builds array" };
  return { feed: json };
}

// Before the feed is even read: on a fresh project there is no bucket, and
// every call below would come back as a 400 that reads like a broken script
// rather than like a missing bucket.
await ensureBucket();

const read = await readFeed();
if (read.error) {
  console.error(`\npublish_build: ${read.error}`);
  if (!has("replace-feed")) {
    console.error("  Refusing to publish: rewriting the feed from here would drop every");
    console.error("  other platform's build. Fix the read, or pass --replace-feed if you");
    console.error("  really do mean to publish a feed containing only this build.");
    process.exit(1);
  }
  console.error("  --replace-feed given: publishing a feed containing ONLY this build.");
  console.error("  Any build for another platform that was in the old feed is now gone.\n");
}
const feed = read.feed ?? { builds: [] };
if (read.fresh) console.log("feed    none yet -- this will create it");
else if (!read.error) console.log(`feed    ${feed.builds.length} existing build(s)`);
// One entry per platform: Helios asks "what is there for me", not "what is
// there". History lives in the bucket, which keeps every version's object.
const carried = feed.builds.filter((b) => b.platform !== PLATFORM);
for (const b of carried) console.log(`        carrying over ${b.platform} ${b.version}`);
feed.builds = [entry, ...carried];

console.log("\nuploading the executable...");
await upload(objectPath, fs.readFileSync(exePath), "application/octet-stream");
console.log("uploading the feed...");
await upload("feed.json", JSON.stringify(feed, null, 2), "application/json");

console.log(`\npublished ${VERSION} for ${PLATFORM}`);
console.log(`feed: ${SUPABASE_URL}/storage/v1/object/public/${BUCKET}/feed.json`);
console.log("Helios will offer it the next time somebody opens the Sim module without one installed.");
