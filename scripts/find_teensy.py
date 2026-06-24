Import("env")
import glob

matches = glob.glob("/dev/serial/by-id/usb-Reiser_Lab_G6_Arena_*-if00")
if matches:
    env.Replace(MONITOR_PORT=matches[0])
