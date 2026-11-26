#!/bin/bash
set -euo pipefail
cd "$(dirname "$0")"
export ROKI_SIM_STATE_DIR="$PWD/training-profile"
exec /bin/bash ../../roki-ng-operator-gui/scripts/run_macos_simulator.sh
