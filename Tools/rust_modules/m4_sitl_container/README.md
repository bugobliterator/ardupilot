# linux/amd64 SITL container for Rust modules

The SITL runtime for native Rust modules loads `x86_64-unknown-linux-gnu`
modules, so it runs on x86-64 Linux only. On other hosts (an arm64 Mac, for
example) SITL and the autotests for Rust modules run in this `linux/amd64`
container: Ubuntu 22.04 with the ArduPilot build packages, the autotest Python
packages and rustc 1.99.1 with the `x86_64-unknown-linux-gnu` target.

## Use

Check that Docker is running first:

```bash
docker info > /dev/null && echo ok
```

Then run a command from the repository root:

```bash
Tools/rust_modules/m4_sitl_container/run.sh '<command>'
```

`run.sh` builds the image `rust-m4-sitl:<tag>` when it is missing (the tag is
the first 12 hex digits of the Dockerfile's SHA-256, so an edited Dockerfile
builds a new image), removes a container `rust-m4-sitl` left by an earlier
run, copies the worktree into the volume `rust-m4-sitl-work` (mounted at
`/work`, without `.git`; the volume's `build/` persists, so the host's
`build/` is never shared) and runs `<command>` with bash in `/work`, in a
container named `rust-m4-sitl` started with `--init`, so stopping `run.sh`
stops the container. The exit status is the command's; 2 means Docker is not
running.

`/work` is a one-commit git repository without submodule gitlinks, so waf
cannot update submodules there: every configure in the container passes
`--no-submodule-update`, directly or through autotest's `--waf-configure-args`:

```bash
Tools/rust_modules/m4_sitl_container/run.sh \
  './Tools/autotest/autotest.py --no-clean --waf-configure-args="--no-submodule-update" build.Plane test.Plane.AerobaticsScripting_rust'
```

Results are functional only: under amd64 emulation SITL is not timing
representative.

After a branch change, drop the volume so that the next run starts from a
clean copy and build:

```bash
docker volume rm rust-m4-sitl-work
```

## Watching the AirShow from the host

`run.sh --udp-out '<command>'` passes `RUST_M4_SITL_OUT=udp:host.docker.internal:14550`
to the command: an outbound MAVLink stream to the host's UDP port 14550 (no port is
published). `watch_airshow.sh` uses it: it builds SITL Plane (`m4`, the default, with
`--enable-rust-m4`, or `in-process`), installs `airshow.apmod` and its schedule, starts
SITL at real time with `--serial1 udpclient:<host>:14550`, loads the
`AerobaticsScripting_rust_mission` mission after `Rust: loaded 1 modules` and selects AUTO.
It keeps running until it is stopped:

```bash
Tools/rust_modules/m4_sitl_container/run.sh --udp-out 'Tools/rust_modules/m4_sitl_container/watch_airshow.sh m4'
```

On the host, a GCS listens on the port, then arms (`arm throttle` in MAVProxy):

```bash
mavproxy.py --master=udp:0.0.0.0:14550 --out udp:127.0.0.1:14551 --load-module map3d
```

Stop it with Ctrl-C on `run.sh` (or `docker rm -f rust-m4-sitl`).
