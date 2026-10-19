# Rust module tools

These tools build, inspect and test ArduPilot's experimental native Rust
modules. See the [runtime guide](../../libraries/AP_Rust/README.md) for firmware
configuration and installation, and the [design](../../libraries/AP_Rust/DESIGN.md)
for the container, ABI and resource-ownership contracts.

## Prerequisites

Install Python with `pyelftools` and Rust **1.98.1 or newer**. The minimum compiler
version is shared by the builder and CI through `rust-toolchain-version`.
`RUSTC` selects a compiler other than the default `rustc`.

```sh
rustup update stable
rustup target add --toolchain stable thumbv7em-none-eabihf
python3 -m pip install pyelftools pytest
```

Supported targets are `thumbv7em-none-eabihf` for STM32 and
`x86_64-unknown-linux-gnu` for x86-64 SITL/Linux. Host builds need native
GCC/binutils; firmware builds and some tests also need the ArduPilot ARM
toolchain. Renode integration tests require `RENODE` to name an executable;
SD-card tests also require mtools. Compiler-dependent tests fail if their
compiler is missing or too old. Renode tests skip when their emulator or
firmware fixture is unavailable.

## Build and inspect

Run from the repository root:

```sh
Tools/rust_modules/build.py \
    libraries/AP_Rust/applets/aerobatics/main.rs \
    build/rust_modules/airshow.apmod \
    --target thumbv7em-none-eabihf --stack-size 16384
Tools/rust_modules/apmod_info.py build/rust_modules/airshow.apmod
Tools/rust_modules/apmod_info.py --fixups build/rust_modules/airshow.apmod
```

`build.py` generates bindings below `build/rust_modules/libraries/AP_Rust/`,
compiles the SDK and source with fat LTO, and packages the linked ELF. It also
writes the intermediate `.elf` and `lib<name>_sdk.rlib` beside the output.
`--stack-size` declares module stack use, not a measured bound. The firmware's
`RUST_STACK_SIZE` must additionally cover the host-call margin: 4 KiB on Cortex-M
or 16 KiB on x86-64; backend overhead is additional.

`apmod_info.py` validates the file and reports its target, ABI, code/data sizes,
alignments, stack requirement, relocations and imported API names. It does not
check whether a particular firmware provides those APIs or has enough RAM.

`pack.py` is the lower-level ELF converter. Normal SDK builds should use
`build.py`, which supplies the generated ABI manifest and target profile.
Without a manifest, the converter emits a test-only relocation probe, not a
vehicle-loadable module. Unsupported sections, unresolved symbols and unsupported
instruction relocations are rejected rather than delegated to a firmware linker.

## Tests

Python tests live in `tests/`; generator tests live with the Rust generator.
These toolchain/loader tests run without a vehicle firmware build:

```sh
python3 -m pytest -q Tools/rust_modules/tests/test_build.py \
    Tools/rust_modules/tests/test_config.py \
    Tools/rust_modules/tests/test_modules.py \
    Tools/rust_modules/tests/test_sdk.py \
    Tools/rust_modules/tests/test_imports.py \
    Tools/rust_modules/tests/test_split_alignment.py
python3 -m pytest -q libraries/AP_Rust/generator
flake8 Tools/rust_modules
```

Remaining tests exercise SITL vehicles or ChibiOS examples. Build Plane with
Rust enabled before running `tests/test_runtime.py` or the binding/applet tests.
Lua comparison tests also require Lua. `AP_RUST_SITL_BINARY` selects a separately
built Plane executable. Do not replace it during a test: SITL executes it again
on reboot.

For the H743 SD runtime harness:

```sh
./waf configure --board MatekH743 \
    --extra-hwdef libraries/AP_Rust/tests/enabled.hwdef
./waf --targets examples/RustRuntime
RENODE=/path/to/renode python3 -m pytest -q Tools/rust_modules/tests/test_manager.py
```

The mixed ROMFS/SD harness uses a separate build directory:

```sh
Tools/rust_modules/build_romfs_runtime.py
RENODE=/path/to/renode python3 -m pytest -q Tools/rust_modules/tests/test_romfs_manager.py
```

`OWNED_THREAD_BOARD` selects a compatible board fixture; `AP_RUST_BUILD_ROOT`
and `AP_RUST_ROMFS_BUILD_ROOT` select existing example build roots. Tests for
`OwnedThread`, `ExecutableMemory`, `RustHeap`, `RustModule`, `RustMath` and
`StackGuard` need the corresponding examples built first. H7 stack-guard tests
require a Renode build with the Cortex-M stacking-fault fix; they do not establish
real-hardware fault recovery or flight safety.

Vehicle autotests build and install modules through scoped helpers that refuse
to overwrite existing files. For example:

```sh
Tools/autotest/autotest.py build.Plane test.Plane.AerobaticsScripting_rust
```

Test sources/data remain beside the tools or under `libraries/AP_Rust/tests/`.
Frozen fixtures in `libraries/AP_Rust/tests/compat/` must not be regenerated
during routine SDK changes.
