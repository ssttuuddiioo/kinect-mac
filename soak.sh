#!/bin/bash
# Overnight soak test for the Kinect v2 depth viewer.
#
# Keeps the Mac awake (system sleep would cut USB and read as a sensor fault),
# restarts the viewer if it dies, and records every start/exit with its code.
# The viewer writes its own per-session log; this catches what it cannot -
# hard crashes, kills, and how long each run survived.
#
#   ./soak.sh          start the soak
#   Ctrl-C             stop it
#   ./soak.sh --report summarise the logs afterwards

set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
PY=/opt/homebrew/bin/python3.14
LOGS="$HERE/logs"
SUP="$LOGS/supervisor.log"
mkdir -p "$LOGS"

stamp() { date '+%Y-%m-%d %H:%M:%S'; }

if [ "${1:-}" = "--report" ]; then
    echo "=== supervisor ==="
    [ -f "$SUP" ] && tail -40 "$SUP" || echo "no supervisor log yet"
    echo
    echo "=== per-session totals ==="
    for f in "$LOGS"/health-*.log; do
        [ -e "$f" ] || { echo "no session logs yet"; break; }
        printf '%-34s  %s\n' "$(basename "$f")" \
            "$(grep -cE ' (MISS|DROPOUT) ' "$f" 2>/dev/null | tr -d '\n') miss/dropout events, last: $(tail -1 "$f" | cut -c1-60)"
    done
    echo
    echo "=== stalls / sensor failures ==="
    grep -hE 'STALL|SENSOR_FAIL|EXIT' "$LOGS"/health-*.log 2>/dev/null | tail -20 || echo "none"
    echo
    echo "=== USB errors ==="
    grep -hoE 'LIBUSB_ERROR_[A-Z_]+' "$LOGS"/stderr-*.log "$LOGS"/app.log 2>/dev/null \
        | sort | uniq -c || echo "none"
    echo
    echo "=== hard crashes (segfault stack dumps) ==="
    if [ -s "$LOGS/crash.log" ]; then
        grep -cE 'Fatal Python error|Current thread' "$LOGS/crash.log" | sed 's/^/crash dumps: /'
        echo "  see $LOGS/crash.log"
    else
        echo "none"
    fi
    echo
    echo "=== memory trend (peak RSS over the run) ==="
    grep -hoE 'peakRSS=[0-9]+MB' "$LOGS"/health-*.log 2>/dev/null | awk -F'[=M]' '
        NR==1{first=$2} {last=$2; n++}
        END{if(n) printf "  first %sMB -> last %sMB over %d samples\n", first, last, n;
            else print "  no samples yet"}'
    exit 0
fi

run=0
echo "$(stamp)  SUPERVISOR    soak started, pid $$" >> "$SUP"
child=""
stop() {
    echo "$(stamp)  SUPERVISOR    stopped by user after $run run(s)" >> "$SUP"
    [ -n "$child" ] && kill "$child" 2>/dev/null
    echo; echo "stopped - see $SUP"
    exit 0
}
trap stop INT TERM

echo "Soak running. Logs in $LOGS"
echo "Ctrl-C to stop.  Summarise later with: ./soak.sh --report"

while true; do
    run=$((run + 1))
    started=$(date +%s)
    echo "$(stamp)  SUPERVISOR    run #$run starting" >> "$SUP"

    # -i no idle sleep, -s no system sleep. Display may still sleep, which is fine.
    # Background + wait, so Ctrl-C runs the trap immediately instead of
    # being deferred until the viewer exits on its own.
    caffeinate -is "$PY" "$HERE/kinect_app.py" >> "$LOGS/stderr-$(date +%Y%m%d).log" 2>&1 &
    child=$!
    wait "$child"
    code=$?
    child=""

    lasted=$(( $(date +%s) - started ))
    echo "$(stamp)  SUPERVISOR    run #$run exited code=$code after ${lasted}s" >> "$SUP"

    if [ $code -eq 0 ]; then
        echo "$(stamp)  SUPERVISOR    clean exit, soak finished" >> "$SUP"
        echo "Viewer closed cleanly - soak finished."
        exit 0
    fi

    echo "$(stamp)  SUPERVISOR    restarting in 5s" >> "$SUP"
    sleep 5
done
