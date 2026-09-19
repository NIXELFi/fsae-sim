// Shader sources are JavaScript template literals, so a backtick anywhere
// inside one -- including inside a GLSL comment -- silently terminates the
// string and turns the rest of the shader into broken JavaScript. The failure
// is a syntax error a long way from the cause, and it has now happened twice.
//
//   node sim/tools/check_shaders.mjs
//
// Compiles nothing: this is a lint, not a driver. Three passes, cheapest last:
//
//   1. import() every module. A stray backtick makes the file unparseable, so
//      the engine's own parser is the most reliable detector there is -- far
//      better than counting braces and hoping.
//   2. extract every shader literal by scanning to the first UNESCAPED
//      backtick, which is where JavaScript itself ends the literal, and check
//      that what follows looks like the end of a statement. A literal that
//      ended early is one whose "end" is in the middle of a line of GLSL.
//   3. brace balance and #version placement on the body that pass 2 found.
//
// Any file that fails to parse, and any run that finds no shaders at all, is a
// failure: a lint that silently checks nothing is worse than no lint, because
// it reports success.

import fs from "node:fs";
import path from "node:path";
import { fileURLToPath, pathToFileURL } from "node:url";

// Resolved against this file, not the working directory. Run from anywhere --
// a hook, another repo, CI with a different checkout root -- and it still looks
// at the sim's shaders instead of finding nothing and exiting green.
const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "..");
const DIRS = ["src/render", "src/game"].map((d) => path.join(ROOT, d));

const problems = [];
/** Modules that threw something other than a SyntaxError on import. Reported,
 *  because a module the parser never got through is a module this lint did not
 *  check -- and a top-level `await` that never settles would hang it. */
const sideEffects = [];
let scanned = 0;
let parsed = 0;

/**
 * The body of the template literal that starts at `from` (the index of its
 * opening backtick), ending where JavaScript ends it: at the first backtick
 * that is not escaped.
 *
 * The regex this replaces asked for a backtick followed by a semicolon, so
 * when a literal was cut short by a stray backtick the match simply ran on to
 * the next candidate that did have one -- the exact mistake it was written to
 * catch was the one it skipped over.
 */
function literalBody(src, from) {
  let i = from + 1;
  while (i < src.length) {
    const c = src[i];
    if (c === "\\") { i += 2; continue; }
    if (c === "`") return { body: src.slice(from + 1, i), end: i };
    i++;
  }
  return null;
}

function walk(dir) {
  if (!fs.existsSync(dir)) return [];
  return fs
    .readdirSync(dir)
    .filter((n) => n.endsWith(".js"))
    .map((n) => path.join(dir, n));
}

const files = DIRS.flatMap(walk);

for (const file of files) {
  const rel = path.relative(ROOT, file).replace(/\\/g, "/");
  const src = fs.readFileSync(file, "utf8");

  // Pass 1 -- the parser. A module that imports the DOM or WebGL at load time
  // is not a lint failure; only a SyntaxError is. But a module that threw for
  // some OTHER reason was not parsed by this tool either, so counting it as
  // parsed overstated what pass 1 had actually verified.
  try {
    await import(pathToFileURL(file).href);
    parsed++;
  } catch (err) {
    if (err instanceof SyntaxError) {
      problems.push(`${rel}  does not parse: ${err.message}`);
      continue;
    }
    sideEffects.push(`${rel} (${err?.message ?? err})`);
  }

  // Pass 2 -- find the literals.
  //
  // Any `= \`` or `: \`` or `(\``, not just a `const` declaration: a shader
  // living in an object property or passed straight to a function is exactly
  // as breakable and was previously skipped in silence.
  const decl = /(?:(?:const|let|var)\s+(\w+)\s*=|(\w+)\s*:|\(\s*)\s*`/g;
  let m;
  while ((m = decl.exec(src)) !== null) {
    const open = m.index + m[0].length - 1;
    const lit = literalBody(src, open);
    if (!lit) {
      problems.push(`${rel}  ${m[1]}: template literal is never closed`);
      break;
    }
    decl.lastIndex = lit.end + 1;

    const name = m[1] || m[2] || "(inline)";
    const body = lit.body;
    const isShader =
      /_(VS|FS|GLSL)$/.test(name) ||
      body.includes("#version 300 es") ||
      /\b(gl_Position|gl_FragCoord)\b/.test(body);
    if (!isShader) continue;
    scanned++;
    const line = src.slice(0, m.index).split("\n").length;

    // Pass 3 -- what the literal looks like from the outside, and from within.
    // What follows the literal. A stray backtick that happened to land at the
    // end of a line left `after` as whitespace, which the old test accepted --
    // so the one arrangement most likely to occur in a commented shader was
    // the one arrangement that slipped through. Look at the next
    // NON-WHITESPACE character instead, and only accept end-of-file.
    const rest = src.slice(lit.end + 1);
    const after = rest.replace(/^\s+/, "").slice(0, 2);
    if (after !== "" && !/^[;,)\]}.+`]/.test(after)) {
      problems.push(
        `${rel}:${line}  ${name}: the literal ends mid-statement (next: ${JSON.stringify(after)})` +
          " -- a stray backtick inside the GLSL closed it early",
      );
    }
    const opens = (body.match(/\{/g) || []).length;
    const closes = (body.match(/\}/g) || []).length;
    if (opens !== closes) {
      problems.push(`${rel}:${line}  ${name}: ${opens} '{' vs ${closes} '}'`);
    }
    if (body.includes("#version") && !body.trimStart().startsWith("#version")) {
      problems.push(`${rel}:${line}  ${name}: #version is not the first line`);
    }
  }
}

console.log(`parsed ${parsed}/${files.length} module(s), checked ${scanned} shader source(s)`);
for (const m of sideEffects) console.log(`  note: did not import cleanly, so only text-checked: ${m}`);
for (const p of problems) console.error(`  ${p}`);
if (problems.length) process.exit(1);
if (scanned === 0) {
  console.error(`  no shader sources found under ${DIRS.join(", ")} -- this lint checked nothing`);
  process.exit(1);
}
console.log("all balanced");
