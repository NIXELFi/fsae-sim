//! Replay video export: the file side.
//!
//! The webview renders and encodes the MP4 (WebCodecs + mp4-muxer, see
//! src/game/videoExport.js) and streams the bytes here in chunks, each with
//! the byte offset it belongs at -- the muxer patches the `mdat` size at the
//! front once it knows it, so the writes are positional, not an append.
//!
//! Files go to the user's Videos folder (`FSAE Sim` inside it; `Movies` on
//! macOS), named by the caller but never pathed by it.

use std::collections::HashMap;
use std::fs::File;
use std::io::{Seek, SeekFrom, Write};
use std::path::PathBuf;
use std::sync::Mutex;

#[derive(Default)]
pub struct VideoFiles {
    next: Mutex<u32>,
    open: Mutex<HashMap<u32, (File, PathBuf)>>,
}

#[derive(serde::Serialize)]
pub struct VideoFile {
    id: u32,
    path: String,
}

fn videos_dir() -> PathBuf {
    let home = std::env::var_os("USERPROFILE")
        .or_else(|| std::env::var_os("HOME"))
        .map(PathBuf::from)
        .unwrap_or_else(std::env::temp_dir);
    let base = if cfg!(target_os = "macos") { home.join("Movies") } else { home.join("Videos") };
    base.join("FSAE Sim")
}

/// A name, not a path: anything but letters, digits and `-_ .` is dropped.
fn safe_stem(name: &str) -> String {
    let s: String = name
        .chars()
        .map(|c| if c.is_ascii_alphanumeric() || matches!(c, '-' | '_' | ' ' | '.') { c } else { '_' })
        .collect();
    let s = s.trim().trim_matches('.').trim_end_matches(".mp4").to_string();
    if s.is_empty() { "replay".into() } else { s.chars().take(120).collect() }
}

/// Create the file; resolves to its id and full path.
#[tauri::command]
pub fn video_begin(name: String, state: tauri::State<'_, VideoFiles>) -> Result<VideoFile, String> {
    let dir = videos_dir();
    std::fs::create_dir_all(&dir).map_err(|e| format!("create {}: {e}", dir.display()))?;
    let stem = safe_stem(&name);
    let mut path = dir.join(format!("{stem}.mp4"));
    let mut n = 2;
    while path.exists() {
        path = dir.join(format!("{stem} ({n}).mp4"));
        n += 1;
    }
    let file = File::create(&path).map_err(|e| format!("create {}: {e}", path.display()))?;
    let id = {
        let mut next = state.next.lock().unwrap();
        *next += 1;
        *next
    };
    state.open.lock().unwrap().insert(id, (file, path.clone()));
    Ok(VideoFile { id, path: path.display().to_string() })
}

/// One chunk, raw bytes in the body, `x-video-id` and `x-offset` headers.
#[tauri::command]
pub fn video_write(request: tauri::ipc::Request<'_>, state: tauri::State<'_, VideoFiles>) -> Result<(), String> {
    let header = |k: &str| -> Result<u64, String> {
        request
            .headers()
            .get(k)
            .and_then(|v| v.to_str().ok())
            .and_then(|v| v.parse::<u64>().ok())
            .ok_or_else(|| format!("missing header {k}"))
    };
    let id = header("x-video-id")? as u32;
    let offset = header("x-offset")?;
    let tauri::ipc::InvokeBody::Raw(bytes) = request.body() else {
        return Err("video_write wants a raw body".into());
    };
    let mut open = state.open.lock().unwrap();
    let (file, _) = open.get_mut(&id).ok_or("no such video")?;
    file.seek(SeekFrom::Start(offset)).map_err(|e| e.to_string())?;
    file.write_all(bytes).map_err(|e| e.to_string())
}

/// Close it. `keep: false` (a cancelled export) deletes the partial file.
#[tauri::command]
pub fn video_end(id: u32, keep: bool, state: tauri::State<'_, VideoFiles>) -> Result<String, String> {
    let (file, path) = state.open.lock().unwrap().remove(&id).ok_or("no such video")?;
    file.sync_all().map_err(|e| e.to_string())?;
    drop(file);
    if !keep {
        let _ = std::fs::remove_file(&path);
    }
    Ok(path.display().to_string())
}

/// Show the finished file in Explorer / Finder.
#[tauri::command]
pub fn video_reveal(path: String) -> Result<(), String> {
    let p = PathBuf::from(&path);
    // Only ever something this module wrote.
    if !p.starts_with(videos_dir()) {
        return Err("not an exported video".into());
    }
    #[cfg(target_os = "windows")]
    let r = std::process::Command::new("explorer").arg(format!("/select,{}", p.display())).spawn();
    #[cfg(target_os = "macos")]
    let r = std::process::Command::new("open").arg("-R").arg(&p).spawn();
    #[cfg(not(any(target_os = "windows", target_os = "macos")))]
    let r = std::process::Command::new("xdg-open").arg(p.parent().unwrap_or(&p)).spawn();
    r.map(|_| ()).map_err(|e| e.to_string())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn names_cannot_escape_the_folder() {
        assert_eq!(safe_stem("../../etc/passwd"), "_.._etc_passwd");
        assert_eq!(safe_stem("Josh 38.020 lap 1.mp4"), "Josh 38.020 lap 1");
        assert_eq!(safe_stem("   "), "replay");
    }
}
