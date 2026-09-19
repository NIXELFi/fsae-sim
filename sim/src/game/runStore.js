// Saving and loading recorded runs from the frontend.
//
// On the desktop this is a thin wrapper over the four Tauri commands in
// `src-tauri/src/runs.rs`. In a browser there is no filesystem to write to, so
// a save turns into two downloads instead -- which is still useful, because it
// is how you get a run out of the browser dev loop and into Helios.

import { isDesktop } from "./desktop.js";

function invoke(cmd, args) {
  const core = window.__TAURI__?.core;
  if (!core?.invoke) return Promise.reject(new Error("not running in the desktop shell"));
  return core.invoke(cmd, args);
}

/**
 * Run id: a sortable timestamp, the course, and four random characters so two
 * runs started in the same second on two machines cannot collide when the runs
 * directory is on a shared drive.
 */
export function newRunId(track, when = new Date()) {
  const p = (n, w = 2) => String(n).padStart(w, "0");
  const stamp =
    `${when.getFullYear()}${p(when.getMonth() + 1)}${p(when.getDate())}` +
    `-${p(when.getHours())}${p(when.getMinutes())}${p(when.getSeconds())}`;
  const slug = String(track || "run").toLowerCase().replace(/[^a-z0-9]+/g, "-").slice(0, 24);
  const rand = Math.random().toString(36).slice(2, 6);
  return `${stamp}-${slug}-${rand}`;
}

/**
 * Write one run. Returns what the shell did with it, or throws.
 *
 * @param runId     from `newRunId`
 * @param manifest  the object from `Recorder.toManifest()`
 * @param csv       the string from `Recorder.toCsv()`
 */
export async function saveRun(runId, manifest, csv) {
  const manifestText = JSON.stringify(manifest, null, 2);
  if (!isDesktop) {
    downloadText(`${runId}.run.json`, manifestText, "application/json");
    downloadText(`${runId}.telemetry.csv`, csv, "text/csv");
    return { runId, dir: "(browser download)", bytes: manifestText.length + csv.length };
  }
  return invoke("save_run", { runId, manifest: manifestText, telemetry: csv });
}

/**
 * Read a run back. `run` may be an id, a directory, or a manifest path.
 *
 * In a browser there is no shell to ask, so runs are served as static files
 * from `sim/runs/<id>/`. That is not a fallback for users -- it is the dev
 * loop: drop a run directory in beside the course data and the replay can be
 * worked on with a page reload instead of a two-minute relink.
 */
export async function loadRun(run) {
  if (!isDesktop) {
    const id = String(run).replace(/[^A-Za-z0-9._-]/g, "");
    const [m, t] = await Promise.all([
      fetch(`./runs/${id}/run.json`).then(okOrThrow),
      fetch(`./runs/${id}/telemetry.csv`).then(okOrThrow),
    ]);
    return { runId: id, manifest: JSON.parse(await m.text()), telemetry: await t.text() };
  }
  const res = await invoke("load_run", { run });
  return {
    runId: res.runId,
    manifest: JSON.parse(res.manifest),
    telemetry: res.telemetry,
  };
}

function okOrThrow(res) {
  if (!res.ok) throw new Error(`${res.status} ${res.url}`);
  return res;
}

/** Every recorded run, newest first, manifests only. */
export async function listRuns(limit = 200) {
  if (!isDesktop) {
    // `runs/index.json` is a plain array of run ids, written by whoever put
    // the runs there. Absent is the normal case in a browser.
    try {
      const res = await fetch("./runs/index.json");
      if (!res.ok) return [];
      const ids = await res.json();
      const out = [];
      for (const id of ids.slice(0, limit)) {
        try {
          const m = await fetch(`./runs/${id}/run.json`).then(okOrThrow);
          out.push({ runId: id, manifest: JSON.parse(await m.text()) });
        } catch { /* a run that is not there is not an error */ }
      }
      return out;
    } catch {
      return [];
    }
  }
  const rows = await invoke("list_runs", { limit });
  return rows
    .map((r) => {
      try {
        return { runId: r.runId, manifest: JSON.parse(r.manifest) };
      } catch {
        return null; // a half-written manifest is skipped, not fatal
      }
    })
    .filter(Boolean);
}

/** Where runs are being written. Shown on the launch screen. */
export async function runsDirectory() {
  if (!isDesktop) return "(browser: runs download instead)";
  return invoke("runs_directory");
}

function downloadText(name, text, mime) {
  try {
    const url = URL.createObjectURL(new Blob([text], { type: mime }));
    const a = document.createElement("a");
    a.href = url;
    a.download = name;
    a.click();
    setTimeout(() => URL.revokeObjectURL(url), 10000);
  } catch (err) {
    console.warn("could not download", name, err);
  }
}

/**
 * Parse a telemetry CSV back into columns.
 *
 * The file the recorder writes is plain: one header line, then rows of plain
 * numbers with no quoting, no missing cells and no locale formatting. This
 * reader assumes exactly that and is about twenty times faster than a general
 * CSV parser on a 100 Hz endurance log, which matters because it runs while
 * the driver is waiting to watch a replay.
 */
export function parseTelemetry(text) {
  const nl = text.indexOf("\n");
  if (nl < 0) throw new Error("telemetry is empty");
  const headers = text.slice(0, nl).trim().split(",");
  const nCols = headers.length;
  // Count rows first so the arrays are allocated once at the right size.
  let rows = 0;
  for (let i = nl + 1; i < text.length; i++) {
    if (text.charCodeAt(i) === 10) rows++;
  }
  if (text.charCodeAt(text.length - 1) !== 10) rows++;
  const cols = headers.map(() => new Float64Array(rows));

  let r = 0;
  let i = nl + 1;
  const n = text.length;
  while (i < n && r < rows) {
    let col = 0;
    let start = i;
    while (i <= n) {
      const ch = i < n ? text.charCodeAt(i) : 10;
      if (ch === 44 || ch === 10 || ch === 13) {
        if (col < nCols) cols[col][r] = +text.slice(start, i) || 0;
        col++;
        start = i + 1;
        if (ch !== 44) {
          // End of line. Swallow a CRLF's LF and stop this row.
          if (ch === 13 && text.charCodeAt(i + 1) === 10) { i++; start = i + 1; }
          i++;
          break;
        }
      }
      i++;
    }
    // A trailing blank line produces a row of one empty cell; drop it.
    if (col > 1) r++;
  }

  const byId = new Map();
  headers.forEach((h, c) => byId.set(h, cols[c]));
  return { headers, rows: r, columns: cols, byId, time: cols[0] };
}
