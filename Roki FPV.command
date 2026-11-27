#!/bin/bash
set -eu
export PATH="/opt/homebrew/bin:/usr/local/bin:$PATH"
exec /bin/bash "$(dirname "$0")/roki-ng-operator-gui/scripts/run_macos.sh" "$@"
