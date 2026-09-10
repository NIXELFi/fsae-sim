// SDM26 Driver-in-Loop desktop shell.
//
// The whole simulator is the embedded frontend -- physics, rendering, input and
// audio all live in the webview. This process exists to give it a native window
// and to carry the static files inside the executable, so there is no dev
// server and nothing to install alongside it.

// Release builds are a GUI app: no console window behind the game.
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

mod ffb;

fn main() {
    tauri::Builder::default()
        // Steering-wheel force feedback. The only thing the webview cannot do
        // itself, and the reason this shell has any code at all.
        .manage(ffb::Ffb::new())
        .invoke_handler(tauri::generate_handler![
            ffb::ffb_status,
            ffb::ffb_start,
            ffb::ffb_stop,
            ffb::ffb_update,
        ])
        .run(tauri::generate_context!())
        .expect("failed to start SDM26 Driver-in-Loop");
}
