// Pull the vault's GLBs (stored compressed under their sha256) and unpack them.
import fs from "node:fs";
import zlib from "node:zlib";
const OUT = "C:/Users/nick5/AppData/Local/Temp/claude/C--Users-nick5/2ece2bc5-afba-4c8a-9053-8e62680c927c/scratchpad/cad/raw/";
fs.mkdirSync(OUT, { recursive: true });
const files = {
  "SWMA-ASM-U_V4.glb": "38d0c1a08dc5a5a558512a9f5e8e5c5823c969e3f840abc10199eae116acb2bf",
  "SWMA-ASM-U_V4_no_QR.glb": "329ec328ea9d69208f60c648185b340046d0cb6f2f87fc25f42a4a47d67d8063",
  "SDM26_Chassis_FINAL.glb": "6a859ed1bd766032d44db001253c369695cd706d7b9251ffa78e11ec1c8863e4",
  "chassis_and_suspension.glb": "691f571f0ea126405e756f02f26eabd70e3b57854302d1ffa9d6b15f6f79001e",
  "aero_and_chassis.glb": "6091f976a66d39aa0dc87d780314f56f030d3f6f14949709519d416e54eb8b2c",
};
for (const [name, sha] of Object.entries(files)) {
  const url = `${process.env.SUPABASE_URL}/storage/v1/object/vault-objects/${sha.slice(0, 2)}/${sha}`;
  const res = await fetch(url, { headers: { Authorization: `Bearer ${process.env.SUPABASE_SERVICE_KEY}`, apikey: process.env.SUPABASE_SERVICE_KEY } });
  if (!res.ok) { console.log(name, "HTTP", res.status, await res.text()); continue; }
  let buf = Buffer.from(await res.arrayBuffer());
  const head = buf.subarray(0, 4).toString("hex");
  // Unpack whatever the vault compressed it with.
  if (head === "28b52ffd") buf = zlib.zstdDecompressSync ? zlib.zstdDecompressSync(buf) : (() => { throw new Error("zstd unsupported") })();
  else if (head.startsWith("1f8b")) buf = zlib.gunzipSync(buf);
  fs.writeFileSync(OUT + name, buf);
  console.log(name, "stored-magic", head, "->", buf.length, "bytes, magic", buf.subarray(0, 4).toString());
}
