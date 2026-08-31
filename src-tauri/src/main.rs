// SDM26 Driver-in-Loop desktop shell.
//
// The whole simulator is the embedded frontend -- physics, rendering, input and
// audio all live in the webview. This process exists to give it a native window
// and to carry the static files inside the executable, so there is no dev
// server and nothing to install alongside it.

// Release builds are a GUI app: no console window behind the game.
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

fn main() {
    tauri::Builder::default()
        .run(tauri::generate_context!())
        .expect("failed to start SDM26 Driver-in-Loop");
}
