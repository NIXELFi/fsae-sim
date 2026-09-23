// Stages the game's static files into ../dist, then runs the Tauri build.
//
// Doing the copy here rather than in a `beforeBuildCommand` is deliberate:
// beforeBuildCommand only runs under the Tauri CLI, so a plain
// `cargo build --release` would happily embed a stale dist and produce an exe
// running last week's physics. Running it from build.rs means the one command
// everybody already knows is always correct, and the app needs no Node, no
// pnpm and no Tauri CLI to build.

use std::{env, fs, path::Path};

const FRONTEND: [&str; 3] = ["index.html", "src", "data"];

fn main() {
    println!("cargo:rerun-if-env-changed=FSAE_SIM_BUNDLE_CAD");
    let manifest = env::var("CARGO_MANIFEST_DIR").expect("CARGO_MANIFEST_DIR");
    let src_tauri = Path::new(&manifest);
    let project = src_tauri.parent().expect("project root above src-tauri");
    let dist = project.join("dist");

    // Start clean so a deleted source file cannot linger in the bundle.
    let _ = fs::remove_dir_all(&dist);
    fs::create_dir_all(&dist).expect("create dist");

    for item in FRONTEND {
        let from = project.join(item);
        assert!(from.exists(), "frontend file missing: {}", from.display());
        stage(&from, &dist.join(item));
    }

    tauri_build::build()
}

/// Recursive copy that also tells Cargo to rebuild when any source file
/// changes. Per-file `rerun-if-changed` rather than one line for the whole
/// directory, because directory watching only notices files being added or
/// removed -- editing bicycle.js in place would not trigger a rebuild.
fn stage(from: &Path, to: &Path) {
    println!("cargo:rerun-if-changed={}", from.display());

    let meta = fs::metadata(from)
        .unwrap_or_else(|e| panic!("cannot read {}: {e}", from.display()));

    if meta.is_dir() {
        fs::create_dir_all(to).unwrap_or_else(|e| panic!("mkdir {}: {e}", to.display()));
        let entries = fs::read_dir(from)
            .unwrap_or_else(|e| panic!("read_dir {}: {e}", from.display()));
        for entry in entries {
            let entry = entry.expect("dir entry");
            stage(&entry.path(), &to.join(entry.file_name()));
        }
    } else {
        if is_team_cad(from) && std::env::var("FSAE_SIM_BUNDLE_CAD").as_deref() != Ok("1") {
            // The team's own CAD (car, steering wheel, dash) is kept out of
            // the exe unless asked for: the repo is public and the feed
            // bucket is public, so a build on a machine that happens to have
            // them in data/ would publish them. A stale copy from an earlier
            // bundled build is removed so it cannot ride along either.
            let _ = fs::remove_file(to);
            return;
        }
        if let Some(parent) = to.parent() {
            fs::create_dir_all(parent).expect("mkdir parent");
        }
        fs::copy(from, to).unwrap_or_else(|e| panic!("copy {}: {e}", from.display()));
    }
}

/// data/car.glb, data/steering-wheel.glb, data/dash.glb (see .gitignore).
fn is_team_cad(p: &Path) -> bool {
    let name = p.file_name().and_then(|n| n.to_str()).unwrap_or("");
    let in_data = p.parent().and_then(|d| d.file_name()).and_then(|n| n.to_str()) == Some("data");
    in_data && matches!(name, "car.glb" | "steering-wheel.glb" | "dash.glb")
}
