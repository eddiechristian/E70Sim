//! Put `memory.x` and `device.x` where the linker can find them. `link.x`
//! from cortex-m-rt includes both by name.

use std::env;
use std::fs;
use std::path::PathBuf;

fn main() {
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    for script in ["memory.x", "device.x"] {
        fs::copy(script, out.join(script)).unwrap();
        println!("cargo:rerun-if-changed={script}");
    }
    println!("cargo:rustc-link-search={}", out.display());
}
