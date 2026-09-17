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
mod wheel;

/// What a launcher (Helios, a shortcut, a script) can ask for on the command
/// line. Everything is optional; the launch screen fills in the rest.
///
/// ```text
/// fsae-sim [--track autocross|endurance|mis] [--profile keyboard|gamepad-xbox|gamepad-ps|wheel]
///          [--tc on|off] [--abs on|off] [--auto-shift on|off]
///          [--autostart] [--fullscreen] [--windowed] [--version]
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
    /// Arguments the parser did not understand, reported rather than ignored.
    unknown: Vec<String>,
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
            "--autostart" | "--start" => o.autostart = true,
            "--fullscreen" => o.fullscreen = true,
            "--windowed" | "--window" => o.windowed = true,
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

    fn args(s: &str) -> Vec<String> {
        s.split_whitespace().map(String::from).collect()
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
