#!/bin/bash
# Keep the FPV client paired with the viewer launched by this command.
set -euo pipefail
cd "$(dirname "$0")/.."
export DYLD_FALLBACK_LIBRARY_PATH="/opt/homebrew/lib:/usr/local/lib:/usr/lib"
export DYLD_FRAMEWORK_PATH="$(brew --prefix qt)/lib"
export ROKI_FPV_ROBOT=127.0.0.1
export ROKI_FPV_CONFIG_DIR="$PWD/sim/fpv_profile"
port=8094
python=.deps/macos/venv/bin/python
if ! "$python" -m operator_gui.mac_instance; then
    echo "Сессия Roki уже открыта."
    exit 0
fi
if /usr/sbin/lsof -nP -iUDP:"$port" >/dev/null 2>&1; then
    echo "Порт $port занят. Проверьте уже запущенную сессию." >&2
    exit 1
fi
sim_args=()
if [[ -n "${ROKI_SIM_STATE_DIR:-}" ]]; then
    sim_args+=(--state-dir "$ROKI_SIM_STATE_DIR")
fi
log=$(mktemp -t roki-mujoco)
.deps/macos/venv/bin/mjpython scripts/sim_roki2.py --port "$port" ${sim_args[@]+"${sim_args[@]}"} >"$log" 2>&1 &
sim_pid=$!
cleanup() { kill "$sim_pid" 2>/dev/null || true; rm -f "$log"; }
trap cleanup EXIT
while kill -0 "$sim_pid" 2>/dev/null; do
    actual=$(sed -n 's/^Roki2 simulator: FPV Roki -> 127\.0\.0\.1:\([0-9][0-9]*\)$/\1/p' "$log" | head -1)
    if [[ -n "$actual" ]]; then
        export ROKI_FPV_PORT="$actual"
        echo "MuJoCo и Roki FPV — 127.0.0.1:$actual"
        .deps/macos/venv/bin/python roki_operator.py --robot "$ROKI_FPV_ROBOT" --port "$ROKI_FPV_PORT" --config-dir "$ROKI_FPV_CONFIG_DIR"
        exit
    fi
    sleep 0.2
done
cat "$log" >&2
exit 1
