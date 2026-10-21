#!/bin/bash
# Runs in the container, started with run.sh --udp-out: the AirShow mission flow of the
# AerobaticsScripting_rust_mission tests at real time, left running for a GCS on the host.
# SITL sends MAVLink to MODULE_M4_SITL_OUT (UDP); the mission is loaded and AUTO selected:
# arm from the GCS. usage: watch_airshow.sh [m4|in-process]   (default m4)
set -euo pipefail
MODE="${1:-m4}"
case "$MODE" in
    m4) CONFIGURE=(--disable-lua-scripting --enable-m4-modules) ;;
    in-process) CONFIGURE=() ;;
    *) echo "watch_airshow.sh: mode m4 or in-process" >&2; exit 2 ;;
esac
OUT="${MODULE_M4_SITL_OUT:-}"
[ -n "$OUT" ] || { echo "watch_airshow.sh: start the container with run.sh --udp-out" >&2; exit 2; }
./waf configure --board sitl --debug --no-submodule-update "${CONFIGURE[@]}" > /tmp/watch-configure.log
./waf plane > /tmp/watch-build.log
if [ "$MODE" = in-process ]; then
    rm -f build/sitl/bin/module_m4
fi
W=/work/watch
rm -rf "$W"
mkdir -p "$W/modules" "$W/scripts"
python3 Tools/modules/rust/build.py libraries/AP_Module/applets/aerobatics/main.rs "$W/airshow.apmod" \
    --target x86_64-unknown-linux-gnu --stack-size 16384 --heap-size 28672 > /tmp/watch-module.log
cp "$W/airshow.apmod" "$W/modules/airshow.apmod"
cp libraries/AP_Module/applets/aerobatics/AirShow.txt "$W/scripts/trick72.txt"
cat > "$W/watch.parm" << PARM
MOD_ENABLE 1
MOD_HEAP_MAX 32768
MOD_CODE_MAXSIZE 131072
MOD_STACK_MAX 32768
RTL_AUTOLAND 2
PARM
# SITL's udpclient takes an IPv4 address, not a name
TARGET="${OUT#udp:}"
ADDRESS="$(getent ahostsv4 "${TARGET%:*}" | awk 'NR == 1 { print $1 }')"
cd "$W"
/work/build/sitl/bin/arduplane --model plane-3d --speedup 1 -w \
    --home -35.362938,149.165085,585,354 \
    --defaults /work/Tools/autotest/models/plane.parm,/work/Tools/autotest/models/plane-3d.parm,"$W/watch.parm" \
    --serial1 "udpclient:$ADDRESS:${TARGET##*:}" > "$W/sitl.log" 2>&1 &
SITL=$!
# SERIAL0 (TCP 5760, inside the container only): load the mission and select AUTO
python3 - << 'PY'
import time

from pymavlink import mavutil
from pymavlink import mavwp

link = mavutil.mavlink_connection('tcp:127.0.0.1:5760', source_system=254, retries=60)
link.wait_heartbeat()
deadline = time.time() + 180
while True:
    m = link.recv_match(type='STATUSTEXT', blocking=True, timeout=5)
    if m is not None:
        print('SITL:', m.text, flush=True)
        if m.text.startswith('Module: loaded'):
            break
    if time.time() > deadline:
        raise SystemExit('watch_airshow: no "Module: loaded" text')
loader = mavwp.MAVWPLoader()
loader.load('/work/Tools/autotest/ArduPlane_Tests/AerobaticsScripting_rust_mission/rust-aerobatics.txt')
link.waypoint_count_send(loader.count())
sent = set()
while len(sent) < loader.count():
    m = link.recv_match(type=['MISSION_REQUEST', 'MISSION_REQUEST_INT'], blocking=True, timeout=10)
    if m is None:
        raise SystemExit('watch_airshow: mission upload stalled')
    link.mav.send(loader.wp(m.seq))
    sent.add(m.seq)
print('mission upload:', link.recv_match(type='MISSION_ACK', blocking=True, timeout=10), flush=True)
deadline = time.time() + 30
while link.flightmode != 'AUTO':
    if time.time() > deadline:
        raise SystemExit('watch_airshow: AUTO not accepted')
    link.set_mode('AUTO')
    link.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
print('watch_airshow: mission loaded, AUTO selected; arm from the GCS', flush=True)
link.close()
PY
wait "$SITL"
