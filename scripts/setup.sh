#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
# JDK 11, unzip, curl, adb and python3-venv must be available.
mkdir -p work/tools
if [[ ! -x work/tools/sdk/cmdline-tools/latest/bin/sdkmanager ]]; then
 curl -fL --retry 2 https://dl.google.com/android/repository/commandlinetools-linux-9477386_latest.zip -o work/tools/cmdtools.zip
 unzip -q work/tools/cmdtools.zip -d work/tools/sdk
 mkdir -p work/tools/sdk/cmdline-tools/latest
 mv work/tools/sdk/cmdline-tools/{bin,lib,NOTICE.txt,source.properties} work/tools/sdk/cmdline-tools/latest/
fi
if [[ ! -x work/tools/gradle-7.6.4/bin/gradle ]]; then
 curl -fL --retry 2 https://services.gradle.org/distributions/gradle-7.6.4-bin.zip -o work/tools/gradle.zip
 unzip -q work/tools/gradle.zip -d work/tools
fi
# Routine Android SDK installation includes acceptance of the displayed SDK licenses.
set +o pipefail
yes | work/tools/sdk/cmdline-tools/latest/bin/sdkmanager --sdk_root="$PWD/work/tools/sdk" --licenses
set -o pipefail
work/tools/sdk/cmdline-tools/latest/bin/sdkmanager --sdk_root="$PWD/work/tools/sdk" 'platforms;android-33' 'build-tools;33.0.2'
printf 'sdk.dir=%s/work/tools/sdk\n' "$PWD" > android/local.properties
python3 -m venv .venv
.venv/bin/python -m pip install -r host/requirements.txt
