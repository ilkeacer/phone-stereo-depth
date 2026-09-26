#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
exec work/tools/gradle-7.6.4/bin/gradle -p android assembleDebug "$@"
