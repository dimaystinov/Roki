#!/bin/bash
# Выполняется только на машине сборки; пользователю Homebrew не требуется.
set -euo pipefail
cd "$(dirname "$0")/.."
export DYLD_FRAMEWORK_PATH="$(brew --prefix qt)/lib"
export DYLD_FALLBACK_LIBRARY_PATH="$(brew --prefix)/lib"
.deps/macos/venv/bin/python -m PyInstaller --noconfirm packaging/fpv_roki.spec
