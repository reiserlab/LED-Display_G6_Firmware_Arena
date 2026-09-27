#!/bin/zsh
# Bootloader-route flash of an arbitrary hex (or -b = reboot only). Port must be free.
set -e
HEX=$1; PORT=${2:-/dev/cu.usbmodem121699401}
ls "$PORT" >/dev/null || { echo "no CDC port $PORT"; exit 2; }
/usr/bin/python3 - "$PORT" <<'PY'
import serial, sys, time
s = serial.Serial(sys.argv[1], 134); time.sleep(0.5); s.close(); print("134-baud touch sent")
PY
n=0; until ioreg -p IOUSB -l | grep -q '"idProduct" = 1144'; do sleep 0.5; n=$((n+1)); (( n < 40 )) || { echo "HalfKay never appeared"; exit 3; }; done
echo "HalfKay up after $((n/2)) s"
if [[ "$HEX" == "-b" ]]; then teensy_loader_cli --mcu=TEENSY41 -b; else teensy_loader_cli --mcu=TEENSY41 -w -v "$HEX"; fi
n=0; until ls /dev/cu.usbmodem* >/dev/null 2>&1; do sleep 0.5; n=$((n+1)); (( n < 40 )) || { echo "CDC never came back"; exit 3; }; done
sleep 1; echo "CDC back: $(ls /dev/cu.usbmodem*) at $(TZ=America/New_York date '+%H:%M:%S ET')"
