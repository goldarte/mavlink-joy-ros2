//! Linker workaround for rclrs 0.7.
//!
//! rclrs 0.7 declares its vendored message bindings with `#[link(...)]`, so the
//! linker is asked for the ROS 2 type support libraries of *every* package it
//! vendors — including `test_msgs`, which is not part of a standard ROS 2
//! desktop installation. A plain `colcon build` therefore fails with
//! `unable to find library -ltest_msgs__rosidl_typesupport_c` although this
//! node never touches those bindings.
//!
//! Missing libraries from that vendored set are replaced by empty shared
//! objects that are added to the linker search path. The linker only has to
//! find the file; `--as-needed` drops it again because no symbol is taken from
//! it. If the real library is installed (`apt install ros-jazzy-test-msgs`),
//! no stub is generated and the real one is used.

use std::env;
use std::path::{Path, PathBuf};
use std::process::Command;

/// Message packages that rclrs 0.7 vendors and asks the linker for.
const VENDORED_PACKAGES: [&str; 7] = [
    "action_msgs",
    "builtin_interfaces",
    "example_interfaces",
    "rcl_interfaces",
    "rosgraph_msgs",
    "test_msgs",
    "unique_identifier_msgs",
];

/// Every vendored package provides these two C libraries.
const SUFFIXES: [&str; 2] = ["__rosidl_generator_c", "__rosidl_typesupport_c"];

fn main() {
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-env-changed=AMENT_PREFIX_PATH");

    let stub_dir =
        PathBuf::from(env::var("OUT_DIR").expect("OUT_DIR is set by cargo")).join("link-stubs");
    let link_paths = ament_library_paths();

    let mut stubs = Vec::new();
    for package in VENDORED_PACKAGES {
        for suffix in SUFFIXES {
            let library = format!("{package}{suffix}");
            if link_paths
                .iter()
                .any(|directory| directory.join(format!("lib{library}.so")).exists())
            {
                continue;
            }
            match write_empty_library(&stub_dir, &library) {
                Ok(path) => stubs.push(path),
                Err(error) => {
                    // Not fatal: maybe the library is only reachable through a
                    // link path we do not know about.
                    println!("cargo:warning=cannot create a stub for {library}: {error}");
                }
            }
        }
    }

    if !stubs.is_empty() {
        println!(
            "cargo:warning=rclrs 0.7 wants {} message libraries that are not installed; \
             linking empty stubs instead (install ros-jazzy-test-msgs to remove this warning)",
            stubs.len()
        );
        println!("cargo:rustc-link-search=native={}", stub_dir.display());
    }
}

/// `lib` directories of every prefix in `AMENT_PREFIX_PATH`.
fn ament_library_paths() -> Vec<PathBuf> {
    env::var("AMENT_PREFIX_PATH")
        .unwrap_or_default()
        .split(':')
        .filter(|prefix| !prefix.is_empty())
        .map(|prefix| Path::new(prefix).join("lib"))
        .collect()
}

/// Compiles an empty shared object that satisfies the linker.
fn write_empty_library(directory: &Path, library: &str) -> Result<PathBuf, String> {
    std::fs::create_dir_all(directory).map_err(|error| error.to_string())?;
    let path = directory.join(format!("lib{library}.so"));
    let compiler = env::var("CC").unwrap_or_else(|_| "cc".to_owned());

    let output = Command::new(&compiler)
        .arg("-shared")
        .arg("-o")
        .arg(&path)
        .args(["-x", "c", "/dev/null"])
        .output()
        .map_err(|error| format!("cannot run {compiler}: {error}"))?;

    if !output.status.success() {
        return Err(String::from_utf8_lossy(&output.stderr).trim().to_owned());
    }
    Ok(path)
}
