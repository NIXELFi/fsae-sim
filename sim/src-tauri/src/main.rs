// SDM26 Driver-in-Loop desktop shell.
//
// The whole simulator is the embedded frontend -- physics, rendering, input and
// audio all live in the webview. This process exists to give it a native window
// and to carry the static files inside the executable, so there is no dev
// server and nothing to install alongside it.

// Release builds are a GUI app: no console window behind the game.
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

mod rig;
mod wheel;

fn main() {
    tauri::Builder::default()
        // The rig: vehicle model, steering wheel and force feedback on one
        // native thread at 1 kHz. The reason this shell has any code at all.
        .manage(rig::Rig::new())
        .invoke_handler(tauri::generate_handler![
            rig::rig_status,
            rig::rig_start,
            rig::rig_stop,
            rig::rig_frame,
            rig::rig_command,
        ])
        .run(tauri::generate_context!())
        .expect("failed to start SDM26 Driver-in-Loop");
}
