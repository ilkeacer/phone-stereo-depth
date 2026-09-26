#!/usr/bin/env bash
# Show the normal ROS panel with the phone pinned to Wi-Fi ADB transport.
set -euo pipefail
cd "$(dirname "$0")/.."
if [[ -n "${PHONE_ADB_SERIAL:-}" ]]; then
  serial="$PHONE_ADB_SERIAL"
  [[ "$serial" == *:* ]] || {
    echo 'PHONE_ADB_SERIAL kablosuz IP:port bağlantısı olmalı.' >&2; exit 1;
  }
  [[ "$(adb -s "$serial" get-state | tr -d '\r')" == device ]] || {
    echo 'Seçilen kablosuz ADB bağlantısı hazır değil.' >&2; exit 1;
  }
else
  serial="$(scripts/prepare_phone_wifi.sh)"
fi
export PHONE_ADB_SERIAL="$serial"
echo "Telefon görüntüsü için seçilen bağlantı: Wi-Fi ADB ($serial)"
echo 'Panel açılacak; kameralar ancak başlat düğmesine basınca açılır.'
exec scripts/start_ros_dashboard.sh
