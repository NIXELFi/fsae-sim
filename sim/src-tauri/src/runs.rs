// Where a recorded run goes, and how it gets read back.
//
// Runs are written into a directory that Helios also knows about, because the
// point of recording them is that Helios launches the sim, lists every run it
// has ever produced, ranks them per course, and replays them. The two programs
// agree on a path rather than on a protocol: the sim only ever writes a run
// directory, Helios only ever reads one, and neither has to be running for the
// other to work.
//
//   %LOCALAPPDATA%\Helios\sim-runs\<runId>\    Windows
//   ~/.local/share/Helios/sim-runs/<runId>/    Linux
//   ~/Library/Application Support/Helios/sim-runs/<runId>/   macOS
//
// `FSAE_SIM_RUNS_DIR` overrides it, which is what the tests use and what a
// team running off a shared drive would set.
//
// Each run directory holds exactly two files:
//   run.json        the manifest: who, what, where, the lap times, the events
//   telemetry.csv   the channel log, Helios-canonical headers, 100 Hz
//
// Each file is written to a `.tmp` name and renamed into place, and the
// TELEMETRY is committed before the MANIFEST. The manifest is what every
// reader looks for -- `sim_list_runs` skips a directory without one -- so
// committing it last means a crash between the two writes leaves a directory
// that is simply not a run yet, rather than a run whose telemetry is missing.

use serde::Serialize;
use std::fs;
use std::io::Write;
use std::path::{Path, PathBuf};

const RUNS_SUBDIR: &str = "sim-runs";
const MANIFEST: &str = "run.json";
const TELEMETRY: &str = "telemetry.csv";

/// Root directory for every recorded run.
pub fn runs_dir() -> PathBuf {
    if let Ok(dir) = std::env::var("FSAE_SIM_RUNS_DIR") {
        if !dir.trim().is_empty() {
            return PathBuf::from(dir);
        }
    }
    helios_data_dir().join(RUNS_SUBDIR)
}

/// The `Helios` application-data directory this platform uses. Kept in step
/// with the desktop app's own resolution (it writes `bridge.json` and
/// `shell-state.json` in the same place) so both programs land on one folder
/// without either importing the other.
fn helios_data_dir() -> PathBuf {
    #[cfg(windows)]
    {
        if let Ok(local) = std::env::var("LOCALAPPDATA") {
            if !local.trim().is_empty() {
                return PathBuf::from(local).join("Helios");
            }
        }
    }
    #[cfg(target_os = "macos")]
    {
        if let Ok(home) = std::env::var("HOME") {
            if !home.trim().is_empty() {
                return PathBuf::from(home)
                    .join("Library")
                    .join("Application Support")
                    .join("Helios");
            }
        }
    }
    #[cfg(all(unix, not(target_os = "macos")))]
    {
        if let Ok(xdg) = std::env::var("XDG_DATA_HOME") {
            if !xdg.trim().is_empty() {
                return PathBuf::from(xdg).join("Helios");
            }
        }
        if let Ok(home) = std::env::var("HOME") {
            if !home.trim().is_empty() {
                return PathBuf::from(home).join(".local").join("share").join("Helios");
            }
        }
    }
    std::env::temp_dir().join("Helios")
}

/// A run id is a directory name, so it must not be able to climb out of the
/// runs directory. Ids the sim generates are `<stamp>-<track>-<rand>`; this
/// is the guard for ids that arrive from the frontend or a command line.
fn is_safe_id(id: &str) -> bool {
    !id.is_empty()
        && id.len() <= 128
        && id
            .chars()
            .all(|c| c.is_ascii_alphanumeric() || c == '-' || c == '_' || c == '.')
        && !id.starts_with('.')
        && id != ".."
}

fn run_path(id: &str) -> Result<PathBuf, String> {
    if !is_safe_id(id) {
        return Err(format!("not a run id: {id}"));
    }
    Ok(runs_dir().join(id))
}

#[derive(Serialize, Clone, Debug)]
#[serde(rename_all = "camelCase")]
pub struct SavedRun {
    pub run_id: String,
    pub dir: String,
    pub manifest_path: String,
    pub telemetry_path: String,
    pub bytes: u64,
}

/// Write one run. Called once, at the end of a drive.
///
/// `async` on a plain fn puts the call on Tauri's blocking thread pool: a
/// multi-megabyte telemetry write must not sit on the thread that answers
/// `rig_frame`, or the webview draws the same snapshot until the disk is
/// done and the rig's watchdog holds the car. Same for the two readers.
#[tauri::command(async)]
pub fn save_run(run_id: String, manifest: String, telemetry: String) -> Result<SavedRun, String> {
    let dir = run_path(&run_id)?;
    fs::create_dir_all(&dir).map_err(|e| format!("create {}: {e}", dir.display()))?;
    // Telemetry first: the manifest is what makes the directory a run, so it
    // is the last thing committed.
    write_atomic(&dir.join(TELEMETRY), telemetry.as_bytes())?;
    write_atomic(&dir.join(MANIFEST), manifest.as_bytes())?;
    Ok(SavedRun {
        run_id,
        dir: dir.display().to_string(),
        manifest_path: dir.join(MANIFEST).display().to_string(),
        telemetry_path: dir.join(TELEMETRY).display().to_string(),
        bytes: (manifest.len() + telemetry.len()) as u64,
    })
}

/// Keep a run that is still being driven on disk, so a crash or a killed
/// window loses at most the last few seconds instead of the whole drive.
///
/// `telemetry` is the next slice of CSV rows: it starts the file (header
/// included) when `fresh`, and is appended otherwise. The manifest, when
/// given, replaces the one on disk atomically and says `finishedReason:
/// "interrupted"` until `save_run` writes the real one over it -- so a run
/// cut off mid-drive lists, replays and ranks on the laps it completed.
#[tauri::command(async)]
pub fn checkpoint_run(run_id: String, telemetry: String, fresh: bool, manifest: Option<String>) -> Result<(), String> {
    let dir = run_path(&run_id)?;
    fs::create_dir_all(&dir).map_err(|e| format!("create {}: {e}", dir.display()))?;
    let path = dir.join(TELEMETRY);
    let mut f = fs::OpenOptions::new()
        .create(true)
        .write(true)
        .append(!fresh)
        .truncate(fresh)
        .open(&path)
        .map_err(|e| format!("open {}: {e}", path.display()))?;
    f.write_all(telemetry.as_bytes()).map_err(|e| format!("append {}: {e}", path.display()))?;
    f.flush().map_err(|e| format!("flush {}: {e}", path.display()))?;
    if let Some(m) = manifest {
        write_atomic(&dir.join(MANIFEST), m.as_bytes())?;
    }
    Ok(())
}

/// Write to `<name>.tmp` then rename. A reader either sees the previous file
/// or the new one, never a partial line.
fn write_atomic(path: &Path, bytes: &[u8]) -> Result<(), String> {
    let tmp = path.with_extension(format!(
        "{}tmp",
        path.extension()
            .and_then(|e| e.to_str())
            .map(|e| format!("{e}."))
            .unwrap_or_default()
    ));
    {
        let mut f = fs::File::create(&tmp).map_err(|e| format!("create {}: {e}", tmp.display()))?;
        f.write_all(bytes).map_err(|e| format!("write {}: {e}", tmp.display()))?;
        f.flush().map_err(|e| format!("flush {}: {e}", tmp.display()))?;
    }
    // `fs::rename` on Windows is `MoveFileExW` with `MOVEFILE_REPLACE_EXISTING`
    // and DOES replace an existing file, so the `remove_file` that used to be
    // here bought nothing and cost the guarantee this function is named for:
    // between the remove and the rename the run had no manifest at all, which
    // is exactly the state a crash must not be able to leave behind.
    fs::rename(&tmp, path).map_err(|e| format!("rename into {}: {e}", path.display()))
}

#[derive(Serialize, Clone, Debug)]
#[serde(rename_all = "camelCase")]
pub struct LoadedRun {
    pub run_id: String,
    pub manifest: String,
    pub telemetry: String,
}

/// Read a run back for replay. Accepts either a run id or a path to the run
/// directory / its manifest, so `--replay` can take whatever a user pastes.
#[tauri::command(async)]
pub fn load_run(run: String) -> Result<LoadedRun, String> {
    let dir = resolve_run_dir(&run)?;
    let manifest = fs::read_to_string(dir.join(MANIFEST))
        .map_err(|e| format!("read {}: {e}", dir.join(MANIFEST).display()))?;
    let telemetry = fs::read_to_string(dir.join(TELEMETRY))
        .map_err(|e| format!("read {}: {e}", dir.join(TELEMETRY).display()))?;
    let run_id = dir
        .file_name()
        .and_then(|n| n.to_str())
        .unwrap_or(&run)
        .to_string();
    Ok(LoadedRun { run_id, manifest, telemetry })
}

/// Turn whatever the caller has -- an id, a directory, a manifest path -- into
/// the run directory.
///
/// A path is accepted only if it is INSIDE the runs directory. It used to be
/// accepted unconditionally: `is_file()` returned the parent and `is_dir()`
/// returned itself, so `load_run("C:/Users/someone/private")` read
/// `private/run.json` and `private/telemetry.csv` and handed both to the
/// webview. `is_safe_id` was consulted only on the by-id fallback, which is
/// not where the danger was.
pub fn resolve_run_dir(run: &str) -> Result<PathBuf, String> {
    let trimmed = run.trim().trim_matches('"');
    if trimmed.is_empty() {
        return Err("no run given".into());
    }
    let root = runs_dir();
    let as_path = PathBuf::from(trimmed);
    if as_path.is_absolute() {
        // A manifest path names its directory; a directory names itself.
        let dir = if as_path.is_file() {
            as_path.parent().map(Path::to_path_buf)
        } else if as_path.is_dir() {
            Some(as_path.clone())
        } else {
            None
        };
        if let Some(dir) = dir {
            // Canonicalise both sides so `..` cannot walk out of the root.
            let inside = match (dir.canonicalize(), root.canonicalize()) {
                (Ok(d), Ok(r)) => d.starts_with(&r),
                _ => false,
            };
            if !inside {
                return Err(format!("that is not in the runs directory: {trimmed}"));
            }
            return Ok(dir);
        }
    }
    let by_id = run_path(trimmed)?;
    if by_id.is_dir() {
        return Ok(by_id);
    }
    Err(format!("no such run: {trimmed}"))
}

/// Manifest-only listing, newest first. The sim uses it for the replay picker;
/// Helios does its own richer listing natively.
#[derive(Serialize, Clone, Debug)]
#[serde(rename_all = "camelCase")]
pub struct RunSummary {
    pub run_id: String,
    pub manifest: String,
}

#[tauri::command(async)]
pub fn list_runs(limit: Option<usize>) -> Result<Vec<RunSummary>, String> {
    let root = runs_dir();
    let Ok(entries) = fs::read_dir(&root) else {
        return Ok(Vec::new()); // nothing recorded yet is not an error
    };
    let mut ids: Vec<String> = entries
        .filter_map(|e| e.ok())
        .filter(|e| e.path().is_dir())
        .filter_map(|e| e.file_name().to_str().map(str::to_string))
        .filter(|n| is_safe_id(n))
        .collect();
    // Ids lead with a sortable timestamp, so a reverse lexical sort is
    // newest-first without stat-ing every directory.
    ids.sort_unstable_by(|a, b| b.cmp(a));
    ids.truncate(limit.unwrap_or(200));
    let mut out = Vec::with_capacity(ids.len());
    for id in ids {
        if let Ok(manifest) = fs::read_to_string(root.join(&id).join(MANIFEST)) {
            out.push(RunSummary { run_id: id, manifest });
        }
    }
    Ok(out)
}

/// Where runs are being written, so the frontend can show it and Helios can
/// be pointed at the same place.
#[tauri::command]
pub fn runs_directory() -> String {
    runs_dir().display().to_string()
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::{Mutex, MutexGuard};

    /// `FSAE_SIM_RUNS_DIR` is process-global and Rust runs tests in parallel
    /// threads of one process, so a test that sets it and one that clears it
    /// will trip over each other. Every test that touches it takes this first.
    static ENV_LOCK: Mutex<()> = Mutex::new(());

    fn lock_env() -> MutexGuard<'static, ()> {
        // A panicking test poisons the lock; the variable is set on the way in
        // regardless, so taking a poisoned lock is safe.
        ENV_LOCK.lock().unwrap_or_else(|e| e.into_inner())
    }

    #[test]
    fn rejects_ids_that_climb_out() {
        assert!(!is_safe_id(".."));
        assert!(!is_safe_id("../../etc"));
        assert!(!is_safe_id("a/b"));
        assert!(!is_safe_id("a\\b"));
        assert!(!is_safe_id(""));
        assert!(!is_safe_id(".hidden"));
        assert!(is_safe_id("20260918-142233-autocross-9f3a"));
    }

    #[test]
    fn env_override_wins() {
        let _guard = lock_env();
        let dir = std::env::temp_dir().join("fsae-runs-test");
        std::env::set_var("FSAE_SIM_RUNS_DIR", &dir);
        assert_eq!(runs_dir(), dir);
        std::env::remove_var("FSAE_SIM_RUNS_DIR");
    }

    #[test]
    fn a_path_outside_the_runs_directory_is_refused() {
        let _guard = lock_env();
        let dir = std::env::temp_dir().join(format!("fsae-outside-{}", std::process::id()));
        std::env::set_var("FSAE_SIM_RUNS_DIR", &dir);
        let _ = fs::create_dir_all(&dir);
        let elsewhere = std::env::temp_dir().join(format!("fsae-private-{}", std::process::id()));
        let _ = fs::create_dir_all(&elsewhere);
        fs::write(elsewhere.join(MANIFEST), "{}").unwrap();
        fs::write(elsewhere.join(TELEMETRY), "time_s
0
").unwrap();

        let err = resolve_run_dir(elsewhere.to_str().unwrap()).unwrap_err();
        assert!(err.contains("not in the runs directory"), "{err}");
        let err = resolve_run_dir(elsewhere.join(MANIFEST).to_str().unwrap()).unwrap_err();
        assert!(err.contains("not in the runs directory"), "{err}");

        // A run that IS inside resolves normally, by id and by path.
        let id = "20260918-142233-autocross-9f3a";
        save_run(id.into(), "{}".into(), "time_s
0
".into()).unwrap();
        assert!(resolve_run_dir(id).is_ok());
        assert!(resolve_run_dir(dir.join(id).to_str().unwrap()).is_ok());

        std::env::remove_var("FSAE_SIM_RUNS_DIR");
        let _ = fs::remove_dir_all(&dir);
        let _ = fs::remove_dir_all(&elsewhere);
    }

    #[test]
    fn a_checkpointed_run_is_on_disk_and_the_final_save_replaces_it() {
        let _g = lock_env();
        let tmp = std::env::temp_dir().join(format!("fsae-ckpt-{}", std::process::id()));
        std::env::set_var("FSAE_SIM_RUNS_DIR", &tmp);
        let id = "20260923-120000-autocross-ck01".to_string();
        checkpoint_run(id.clone(), "time_s,a\n0,1\n".into(), true, None).unwrap();
        checkpoint_run(id.clone(), "1,2\n".into(), false, Some("{\"finishedReason\":\"interrupted\"}".into())).unwrap();
        let dir = tmp.join(&id);
        assert_eq!(fs::read_to_string(dir.join(TELEMETRY)).unwrap(), "time_s,a\n0,1\n1,2\n");
        assert!(fs::read_to_string(dir.join(MANIFEST)).unwrap().contains("interrupted"));
        save_run(id.clone(), "{}".into(), "final\n".into()).unwrap();
        assert_eq!(fs::read_to_string(dir.join(TELEMETRY)).unwrap(), "final\n");
        std::env::remove_var("FSAE_SIM_RUNS_DIR");
        let _ = fs::remove_dir_all(&tmp);
    }

    #[test]
    fn round_trips_a_run() {
        let _guard = lock_env();
        let dir = std::env::temp_dir().join(format!("fsae-runs-rt-{}", std::process::id()));
        let _ = fs::remove_dir_all(&dir);
        std::env::set_var("FSAE_SIM_RUNS_DIR", &dir);
        let id = "20260918-142233-autocross-9f3a".to_string();
        let saved = save_run(id.clone(), "{\"a\":1}".into(), "time_s,x\n0,1\n".into())
            .expect("save");
        assert!(PathBuf::from(&saved.manifest_path).is_file());
        let back = load_run(id.clone()).expect("load");
        assert_eq!(back.manifest, "{\"a\":1}");
        assert_eq!(back.telemetry, "time_s,x\n0,1\n");
        let listed = list_runs(None).expect("list");
        assert_eq!(listed.len(), 1);
        assert_eq!(listed[0].run_id, id);
        // Re-saving the same id overwrites rather than failing.
        save_run(id.clone(), "{\"a\":2}".into(), "time_s,x\n0,2\n".into()).expect("resave");
        assert_eq!(load_run(id).expect("load2").manifest, "{\"a\":2}");
        std::env::remove_var("FSAE_SIM_RUNS_DIR");
        let _ = fs::remove_dir_all(&dir);
    }
}
