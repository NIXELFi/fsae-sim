// SDM26 Driver-in-Loop desktop shell.
//
// Rendering, input mapping, audio and the courses live in the embedded
// frontend. This process gives it a native window, carries the static files
// inside the executable, and runs the rig: the vehicle model, the steering
// wheel and the force feedback on one native thread at 1 kHz (`rig.rs`).

// Release builds are a GUI app: no console window behind the game.
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use tauri::Manager;

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
