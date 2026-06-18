Import("env")
import glob

matches = glob.glob("/dev/serial/by-id/usb-Teensyduino_USB_Serial_*-if00")
if matches:
    env.Replace(MONITOR_PORT=matches[0])
