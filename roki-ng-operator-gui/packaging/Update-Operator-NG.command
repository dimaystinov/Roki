#!/bin/bash
set -euo pipefail
app="$(dirname "$0")/FPV Roki.app/Contents/MacOS/FPV Roki"
[[ -x "$app" ]] || app='/Applications/FPV Roki.app/Contents/MacOS/FPV Roki'
exec "$app" --upstream-updater
