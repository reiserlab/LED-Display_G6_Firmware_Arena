#!/usr/bin/env python3
"""Drive the pit-race-repro sketch: send a mode char, read heartbeats, report when they stop."""
import serial, sys, time, glob
def port():
    p = glob.glob('/dev/cu.usbmodem*')
    if not p: raise SystemExit('no CDC port')
    return p[0]
def run(mode, seconds, quiet_limit=3.0):
    s = serial.Serial(port(), 115200, timeout=0.2)
    time.sleep(0.3); s.reset_input_buffer()
    s.write(mode.encode()); s.flush()
    t0 = time.time(); last_line = t0; n = 0; last = ''
    while time.time() - t0 < seconds:
        line = s.readline()
        if line:
            n += 1; last = line.decode(errors='replace').rstrip(); last_line = time.time()
            if n <= 3 or n % 40 == 0: print(f"  {time.time()-t0:6.1f}s  {last}")
        elif time.time() - last_line > quiet_limit:
            print(f"  HEARTBEAT STOPPED after {last_line - t0:.1f} s of '{mode}' mode; last line: {last}")
            # is the USB CDC still alive? try a status request
            try:
                s.write(b'?'); s.flush(); time.sleep(0.5); r = s.read(200)
                print(f"  '?' after stall -> {r!r}")
            except Exception as e:
                print(f"  '?' after stall -> exception {e}")
            s.close(); return 'stalled', last_line - t0, last
    print(f"  ran {seconds}s in '{mode}' mode without a stall; last line: {last}")
    s.close(); return 'ok', seconds, last
if __name__ == '__main__':
    mode = sys.argv[1]; secs = float(sys.argv[2])
    print(f"== mode {mode} for {secs}s on {port()}  ({time.strftime('%H:%M:%S')})")
    print(run(mode, secs))
