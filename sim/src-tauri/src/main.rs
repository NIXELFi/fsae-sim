// SDM26 Driver-in-Loop desktop shell.
//
// Rendering, input mapping, audio and the courses live in the embedded
// frontend. This process gives it a native window, carries the static files
// inside the executable, and runs the rig: the vehicle model, the steering
// wheel and the force feedback on one native thread at 1 kHz (`rig.rs`).

// Release builds are a GUI app: no console window behind the game.
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use serde::Serialize;
use tauri::{Emitter, Manager};

mod rig;
mod runs;
mod wheel;

/// What a launcher (Helios, a shortcut, a script) can ask for on the command
/// line. Everything is optional; the launch screen fills in the rest.
///
/// ```text
/// fsae-sim [--track autocross|endurance|mis|gen-ax-SEED|gen-en-SEED] [--profile keyboard|gamepad-xbox|gamepad-ps|wheel]
///          [--tc on|off] [--abs on|off] [--auto-shift on|off]
///          [--driver NAME] [--driver-id ID] [--replay RUN] [--ghost RUN]
///          [--reference RUN]
///          [--setup FILE.hset]
///          [--no-record]
///          [--autostart] [--fullscreen] [--windowed] [--version]
///
/// A bare argument ending in `.hset` is a setup file too: that is what the
/// shell passes when somebody double-clicks a file associated with the app.
///
/// By default the window comes up borderless and filling the screen (the
/// game window most people expect); `--windowed` keeps a decorated 1600x900
/// window centred on the screen instead.
/// ```
#[derive(Clone, Debug, Default, Serialize)]
#[serde(rename_all = "camelCase")]
struct LaunchOptions {
    track: Option<String>,
    profile: Option<String>,
    traction: Option<bool>,
    abs: Option<bool>,
    auto_shift: Option<bool>,
    autostart: bool,
    fullscreen: bool,
    /// Decorated 1600x900 window instead of borderless full-screen.
    windowed: bool,
    /// Who is driving. Stamped into every run this launch records, so a
    /// leaderboard can be a leaderboard rather than a list of files.
    driver: Option<String>,
    /// The launcher's ID for that person -- a Helios account id. A run
    /// carrying one was started by somebody Helios had signed in; a run
    /// without one was somebody typing a name into the simulator, and the
    /// leaderboard treats the two differently.
    driver_id: Option<String>,
    /// Open straight into the replay of a recorded run (id or path) instead
    /// of the launch screen.
    replay: Option<String>,
    /// A second run to draw alongside the replay as a ghost.
    ghost: Option<String>,
    /// A recorded run whose best lap becomes the live delta's reference, so
    /// the driver is chasing a real lap from the first corner instead of
    /// waiting for lap two.
    reference: Option<String>,
    /// Drive without writing a run. Off by default: the whole point is that
    /// every lap the team drives is kept.
    no_record: bool,
    /// A label for the session -- test day, setup change, driver coaching --
    /// carried into the manifest so runs can be grouped later.
    session: Option<String>,
    /// A Helios setup file (`.hset`) to offer on the Vehicle tab. The page
    /// reads it through `read_text_file` and shows the import summary; the
    /// driver still has to press Apply.
    setup: Option<String>,
    /// Arguments the parser did not understand, reported rather than ignored.
    unknown: Vec<String>,
}

const SETUP_EXT: &str = ".hset";

fn is_setup_path(a: &str) -> bool {
    !a.starts_with("--") && a.to_ascii_lowercase().ends_with(SETUP_EXT)
}

fn on_off(v: Option<&String>) -> Option<bool> {
    match v.map(|s| s.to_ascii_lowercase()).as_deref() {
        Some("on") | Some("1") | Some("true") | Some("yes") => Some(true),
        Some("off") | Some("0") | Some("false") | Some("no") => Some(false),
        _ => None,
    }
}

fn parse_args<I: IntoIterator<Item = String>>(args: I) -> LaunchOptions {
    let args: Vec<String> = args.into_iter().collect();
    let mut o = LaunchOptions::default();
    let mut i = 0;
    while i < args.len() {
        let a = args[i].as_str();
        // `--key=value` and `--key value` both work.
        let (key, inline) = match a.split_once('=') {
            Some((k, v)) => (k, Some(v.to_string())),
            None => (a, None),
        };
        let mut take = || -> Option<String> {
            if inline.is_some() {
                return inline.clone();
            }
            i += 1;
            args.get(i).cloned()
        };
        match key {
            "--track" => o.track = take(),
            "--profile" => o.profile = take(),
            "--tc" | "--traction" => o.traction = on_off(take().as_ref()),
            "--abs" => o.abs = on_off(take().as_ref()),
            "--auto-shift" | "--auto" => o.auto_shift = on_off(take().as_ref()),
            "--driver" => o.driver = take(),
            "--driver-id" => o.driver_id = take(),
            "--replay" => o.replay = take(),
            "--ghost" => o.ghost = take(),
            "--reference" | "--ref" => o.reference = take(),
            "--session" => o.session = take(),
            "--setup" => o.setup = take(),
            "--no-record" | "--norecord" => o.no_record = true,
            "--autostart" | "--start" => o.autostart = true,
            "--fullscreen" => o.fullscreen = true,
            "--windowed" | "--window" => o.windowed = true,
            // Double-clicking an associated file launches `fsae-sim <path>`.
            _ if is_setup_path(a) => o.setup = Some(a.to_string()),
            _ => o.unknown.push(a.to_string()),
        }
        i += 1;
    }
    o
}

#[tauri::command]
fn launch_options(state: tauri::State<'_, LaunchOptions>) -> LaunchOptions {
    state.inner().clone()
}

/// Cap on a setup file. A real one is a few kilobytes; anything near this is
/// not a setup and is not worth handing to the webview.
const SETUP_MAX_BYTES: u64 = 2 * 1024 * 1024;

/// Read a Helios setup file for the page. Deliberately NOT a general file
/// reader: only a `.hset` is accepted, so a `--setup` argument (or a
/// double-clicked file) cannot be turned into a way to read arbitrary text
/// off the machine into the webview.
#[tauri::command(async)]
fn read_text_file(path: String) -> Result<String, String> {
    let trimmed = path.trim().trim_matches('"');
    if !is_setup_path(trimmed) {
        return Err(format!("not a {SETUP_EXT} file: {trimmed}"));
    }
    let p = std::path::Path::new(trimmed);
    let len = std::fs::metadata(p).map_err(|e| format!("read {}: {e}", p.display()))?.len();
    if len > SETUP_MAX_BYTES {
        return Err(format!("{} is {len} bytes, too large for a setup file", p.display()));
    }
    std::fs::read_to_string(p).map_err(|e| format!("read {}: {e}", p.display()))
}

/// Where exported setups go: `sim-setups` beside the runs directory, so a
/// setup and the runs driven on it live in the same Helios data folder.
fn setups_dir() -> std::path::PathBuf {
    runs::runs_dir().with_file_name("sim-setups")
}

#[derive(Serialize)]
#[serde(rename_all = "camelCase")]
struct SavedSetup {
    path: String,
    bytes: usize,
}

/// Write an exported setup. `name` is a filename the page already made safe
/// (`setupFilename` in `setupFile.js`); it is checked again here because the
/// webview is not the trust boundary. Overwrites a setup of the same name,
/// which is what "export again" should do.
#[tauri::command(async)]
fn save_setup_file(name: String, text: String) -> Result<SavedSetup, String> {
    // A name, not a path: refused rather than rewritten, so a caller that
    // sends `../x` finds out instead of getting a file called `x`.
    let trimmed = name.trim();
    let is_name = !trimmed.is_empty()
        && !trimmed.contains("..")
        && trimmed.chars().all(|c| c.is_ascii_alphanumeric() || matches!(c, '.' | '_' | '-' | ' '));
    let safe = trimmed.trim_matches('.').trim().to_string();
    if !is_name || safe.is_empty() {
        return Err(format!("bad setup file name: {name:?}"));
    }
    let file = if safe.to_ascii_lowercase().ends_with(SETUP_EXT) { safe } else { format!("{safe}{SETUP_EXT}") };
    if text.len() as u64 > SETUP_MAX_BYTES {
        return Err("setup text is too large".into());
    }
    let dir = setups_dir();
    std::fs::create_dir_all(&dir).map_err(|e| format!("create {}: {e}", dir.display()))?;
    let path = dir.join(file);
    std::fs::write(&path, text.as_bytes()).map_err(|e| format!("write {}: {e}", path.display()))?;
    Ok(SavedSetup { path: path.display().to_string(), bytes: text.len() })
}

fn main() {
    let argv: Vec<String> = std::env::args().skip(1).collect();
    if argv.iter().any(|a| a == "--version" || a == "-V") {
        println!("fsae-sim {}", env!("CARGO_PKG_VERSION"));
        return;
    }
    let options = parse_args(argv);
    if !options.unknown.is_empty() {
        eprintln!("fsae-sim: ignoring unknown arguments: {}", options.unknown.join(" "));
    }

    tauri::Builder::default()
        .manage(options)
        .plugin(tauri_plugin_single_instance::init(|app, argv, _cwd| {
            // Already running: focus the window and hand it the new request.
            let opts = parse_args(argv.into_iter().skip(1));
            if let Some(w) = app.get_webview_window("main") {
                let _ = w.unminimize();
                let _ = w.set_focus();
                let _ = w.emit("launch-options", &opts);
            }
        }))
        // The rig: vehicle model, steering wheel and force feedback on one
        // native thread at 1 kHz. The reason this shell has any code at all.
        .manage(rig::Rig::new())
        .setup(|app| {
            // The window is created hidden (tauri.conf.json) so this happens
            // before the first paint: no decorated window flashing up and
            // then jumping to the borderless one.
            let windowed = app.state::<LaunchOptions>().windowed;
            if let Some(w) = app.get_webview_window("main") {
                if !windowed {
                    let _ = w.set_decorations(false);
                    let _ = w.maximize(); // the screen's usable area, on every platform
                }
                let _ = w.show();
                let _ = w.set_focus();
            }
            Ok(())
        })
        .invoke_handler(tauri::generate_handler![

            launch_options,
            read_text_file,
            save_setup_file,
            runs::save_run,
            runs::load_run,
            runs::list_runs,
            runs::runs_directory,
            rig::rig_status,
            rig::rig_start,
            rig::rig_stop,
            rig::rig_frame,
            rig::rig_command,
        ])
        // Closing the window must release the wheel (torque off, effect
        // stopped, device unacquired) before the process goes away.
        .on_window_event(|window, event| {
            if let tauri::WindowEvent::CloseRequested { .. } = event {
                window.state::<rig::Rig>().stop();
            }
        })
        .run(tauri::generate_context!())
        .expect("failed to start SDM26 Driver-in-Loop");
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Split a command line the way a shell would: on whitespace, but keeping
    /// a double-quoted run together as one argument.
    ///
    /// Plain `split_whitespace` does not, and the session test below passes
    /// `--session "setup B"` -- which tokenized to `--session`, `"setup`, `B"`,
    /// so `session` came out as `Some("\"setup")` and `B"` landed in
    /// `unknown`. The test passed only because it asserted on neither. A
    /// launcher really does pass a session name with a space in it.
    fn args(s: &str) -> Vec<String> {
        let mut out = Vec::new();
        let mut cur = String::new();
        let mut in_quotes = false;
        let mut started = false;
        for c in s.chars() {
            match c {
                '"' => {
                    in_quotes = !in_quotes;
                    started = true;
                }
                c if c.is_whitespace() && !in_quotes => {
                    if started {
                        out.push(std::mem::take(&mut cur));
                        started = false;
                    }
                }
                c => {
                    cur.push(c);
                    started = true;
                }
            }
        }
        if started {
            out.push(cur);
        }
        out
    }

    #[test]
    fn the_test_helper_keeps_a_quoted_argument_together() {
        assert_eq!(
            args("--session \"setup B\" --track mis"),
            vec!["--session", "setup B", "--track", "mis"],
        );
        assert_eq!(args("  --a   --b  "), vec!["--a", "--b"]);
        assert_eq!(args("--empty \"\""), vec!["--empty", ""]);
    }

    #[test]
    fn parses_a_helios_style_launch() {
        let o = parse_args(args("--track mis --profile wheel --tc off --abs=on --autostart --fullscreen"));
        assert_eq!(o.track.as_deref(), Some("mis"));
        assert_eq!(o.profile.as_deref(), Some("wheel"));
        assert_eq!(o.traction, Some(false));
        assert_eq!(o.abs, Some(true));
        assert_eq!(o.auto_shift, None);
        assert!(o.autostart && o.fullscreen);
        assert!(!o.windowed);
        assert!(o.unknown.is_empty());
    }

    #[test]
    fn parses_a_helios_recording_launch() {
        let o = parse_args(args(
            "--track autocross --driver Nick --session \"setup B\" --autostart",
        ));
        assert_eq!(o.driver.as_deref(), Some("Nick"));
        assert_eq!(o.track.as_deref(), Some("autocross"));
        // The whole point of the quoted argument: a session name with a space
        // in it has to arrive as one value.
        assert_eq!(o.session.as_deref(), Some("setup B"));
        assert!(o.autostart);
        assert!(!o.no_record);
        assert!(o.replay.is_none());
        assert!(o.unknown.is_empty(), "unexpected leftovers: {:?}", o.unknown);
    }

    #[test]
    fn parses_an_authenticated_driver() {
        let o = parse_args(args("--driver Nick --driver-id 8f14e45f-ceea-467a-9c1e-1b2c3d4e5f60"));
        assert_eq!(o.driver.as_deref(), Some("Nick"));
        assert_eq!(o.driver_id.as_deref(), Some("8f14e45f-ceea-467a-9c1e-1b2c3d4e5f60"));
    }

    #[test]
    fn parses_a_reference_lap() {
        let o = parse_args(args("--track autocross --reference 20260918-142233-autocross-9f3a"));
        assert_eq!(o.reference.as_deref(), Some("20260918-142233-autocross-9f3a"));
        assert!(o.replay.is_none(), "a reference is for a DRIVE, not a replay");
        assert!(o.unknown.is_empty());
    }

    #[test]
    fn parses_a_replay_launch() {
        let o = parse_args(args("--replay 20260918-142233-autocross-9f3a --ghost other-run"));
        assert_eq!(o.replay.as_deref(), Some("20260918-142233-autocross-9f3a"));
        assert_eq!(o.ghost.as_deref(), Some("other-run"));
        assert!(o.unknown.is_empty());
    }

    #[test]
    fn parses_a_setup_flag() {
        let o = parse_args(args("--setup \"C:\\Setups\\Nick MIS.hset\" --track mis"));
        assert_eq!(o.setup.as_deref(), Some("C:\\Setups\\Nick MIS.hset"));
        assert_eq!(o.track.as_deref(), Some("mis"));
        assert!(o.unknown.is_empty());
    }

    #[test]
    fn a_double_clicked_setup_file_is_a_setup() {
        // What Explorer passes for an associated file: the bare path, nothing else.
        let o = parse_args(args("\"C:\\Users\\nick\\Downloads\\Quali B.HSET\""));
        assert_eq!(o.setup.as_deref(), Some("C:\\Users\\nick\\Downloads\\Quali B.HSET"));
        assert!(o.unknown.is_empty(), "the path must not be reported as unknown");
        // Any other bare argument is still unknown.
        let o = parse_args(args("notes.txt"));
        assert_eq!(o.setup, None);
        assert_eq!(o.unknown, vec!["notes.txt".to_string()]);
    }

    #[test]
    fn read_text_file_only_reads_setups() {
        let err = read_text_file("C:/Windows/System32/drivers/etc/hosts".into()).unwrap_err();
        assert!(err.contains("not a .hset file"), "{err}");
        let err = read_text_file("does-not-exist.hset".into()).unwrap_err();
        assert!(err.starts_with("read "), "{err}");
    }

    #[test]
    fn save_setup_file_rejects_paths_and_empty_names() {
        assert!(save_setup_file("../escape".into(), "{}".into()).is_err());
        assert!(save_setup_file("".into(), "{}".into()).is_err());
        assert!(save_setup_file("///".into(), "{}".into()).is_err());
    }

    #[test]
    fn unknown_flags_are_reported_not_swallowed() {
        let o = parse_args(args("--bogus --track endurance"));
        assert_eq!(o.unknown, vec!["--bogus".to_string()]);
        assert_eq!(o.track.as_deref(), Some("endurance"));
    }

    #[test]
    fn a_trailing_flag_with_no_value_is_none() {
        let o = parse_args(args("--track"));
        assert_eq!(o.track, None);
    }
}
