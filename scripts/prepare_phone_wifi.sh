#!/usr/bin/env bash
# Print the selected Wi-Fi ADB serial. Never starts cameras or mapping.
set -euo pipefail
cd "$(dirname "$0")/.."
mapfile -t wireless < <(adb devices -l | awk 'NR>1 && $2=="device" && $1 ~ /^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+:[0-9]+$/ {print $1}')
if ((${#wireless[@]}>1)); then
  echo 'Birden fazla kablosuz telefon var; PHONE_ADB_SERIAL değerini açıkça seçin.' >&2
  exit 1
fi
if ((${#wireless[@]}==1)); then
  serial="${wireless[0]}"
  mapfile -t usb < <(adb devices -l | awk 'NR>1 && $2=="device" && $1 !~ /:/ {print $1}')
  if ((${#usb[@]}==1)); then
    usb_identity="$(adb -s "${usb[0]}" shell getprop ro.serialno | tr -d '\r')"
    wifi_identity="$(adb -s "$serial" shell getprop ro.serialno | tr -d '\r')"
    if [[ -n "$usb_identity" && "$wifi_identity" != "$usb_identity" ]]; then
      echo 'Kablosuz ve USB bağlantıları aynı telefona ait değil.' >&2
      exit 1
    fi
  fi
else
  mapfile -t usb < <(adb devices -l | awk 'NR>1 && $2=="device" && $1 !~ /:/ {print $1}')
  if ((${#usb[@]}!=1)); then
    echo 'İlk kablosuz eşleştirme için bir yetkili USB telefon gerekli.' >&2
    exit 1
  fi
  usb_id="${usb[0]}"
  phone_ip="$(adb -s "$usb_id" shell ip -4 -o addr show wlan0 | awk '{split($4,a,"/");print a[1];exit}' | tr -d '\r')"
  if [[ ! "$phone_ip" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
    echo 'Telefonda kullanılabilir Wi-Fi IPv4 adresi bulunamadı.' >&2
    exit 1
  fi
  usb_identity="$(adb -s "$usb_id" shell getprop ro.serialno | tr -d '\r')"
  adb -s "$usb_id" tcpip 5555 >&2
  serial="$phone_ip:5555"
  adb connect "$serial" >&2
  wifi_identity="$(adb -s "$serial" shell getprop ro.serialno | tr -d '\r')"
  if [[ -n "$usb_identity" && "$wifi_identity" != "$usb_identity" ]]; then
    echo 'Kablosuz uçtaki telefon USB ile bağlı telefonla aynı değil.' >&2
    exit 1
  fi
fi
if [[ "$(adb -s "$serial" get-state | tr -d '\r')" != device ]]; then
  echo 'Telefonun kablosuz ADB bağlantısı hazır değil.' >&2
  exit 1
fi
printf '%s\n' "$serial"
