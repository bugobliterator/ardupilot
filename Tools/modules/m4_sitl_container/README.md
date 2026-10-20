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
Tools/modules/m4_sitl_container/run.sh '<command>'
```

`run.sh` builds the image `module-m4-sitl:1` on first use, copies the worktree
into the volume `module-m4-sitl-work` (mounted at `/work`, without `.git`; the
volume's `build/` persists, so the host's `build/` is never shared) and runs
`<command>` with bash in `/work`, in a container named `module-m4-sitl`. The exit
status is the command's; 2 means Docker is not running.

`/work` is a one-commit git repository without submodule gitlinks, so waf
cannot update submodules there: every configure in the container passes
`--no-submodule-update`, directly or through autotest's `--waf-configure-args`:

```bash
Tools/modules/m4_sitl_container/run.sh \
  './Tools/autotest/autotest.py --no-clean --waf-configure-args="--no-submodule-update" build.Plane test.Plane.AerobaticsScripting_rust'
```

Results are functional only: under amd64 emulation SITL is not timing
representative.

After a branch change, drop the volume so that the next run starts from a
clean copy and build:

```bash
docker volume rm module-m4-sitl-work
```
