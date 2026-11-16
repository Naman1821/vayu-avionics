#!/usr/bin/env python3
"""
ground_station.py - decode Vayu's radio downlink.

    python3 tools/ground_station.py out/cpu_hang/downlink.bin       # replay a capture
    python3 tools/ground_station.py --port /dev/ttyUSB0 --ping      # live (pip install pyserial)

Events (boots, phase changes, faults, power states) are printed as they
arrive; status frames are summarised every --every frames.
"""
import argparse
import struct
import sys

from vayu_formats import (ALT_SOURCES, PHASES, POWER_STATES, RESET_CAUSES, crc16_ccitt, fault_names,
                          parse_frames)

TC_PING, TC_ERASE_LOG, ERASE_MAGIC = 0x10, 0x11, 0xE5A5E5A5


def frame(typ, payload=b""):
    body = bytes([len(payload), typ]) + payload
    return b"\xaa\x55" + body + struct.pack("<H", crc16_ccitt(body))


def describe(ev):
    n, a = ev["name"], ev["arg"]
    if n == "BOOT":
        return f"BOOT  reset cause {RESET_CAUSES[a & 0xFF]}" + ("  (warm restart: resumed flight)" if a & 0x100 else "")
    if n == "PHASE":
        return f"PHASE -> {PHASES[a]}"
    if n == "FAULTS":
        return "FAULTS " + (", ".join(fault_names(a)) or "all clear")
    if n == "POWER_STATE":
        return f"POWER -> {POWER_STATES[a]}"
    if n == "GPS_CONFIG":
        return "GPS airborne mode " + ("confirmed" if a else "NOT confirmed")
    if n == "ERASE_REFUSED":
        return f"ERASE refused (in {PHASES[a]})"
    if n == "POWER_FAIL":
        return f"POWER FAIL warning, battery {a / 1000:.2f} V"
    return f"{n} {a}"


def status_line(s):
    return (f"{PHASES[s['phase']]:9s} alt {s['alt_m']:8.1f} m ({ALT_SOURCES[s['alt_src']]})  "
            f"climb {s['climb_ms']:+5.1f} m/s  GPS fix {s['gps_fix']} sats {s['gps_num_sv']:2d}  "
            f"box {s['board_temp_c']:5.1f} C heater {s['heater_duty']:3d} %  "
            f"bat {s['vbat']:.2f} V {s['ibat']:.2f} A {s['used_mah']} mAh {POWER_STATES[s['power']]}")


def show(frames, every):
    n = 0
    for f in frames.all:
        if f["kind"] == "event":
            print(f"[{f['t_ms'] / 1000:9.2f} s] {describe(f)}")
        else:
            n += 1
            if n % every == 0:
                print(f"[{f['t_ms'] / 1000:9.2f} s] {status_line(f)}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("capture", nargs="?", help="downlink capture file")
    ap.add_argument("--port", help="serial port of the ground radio")
    ap.add_argument("--baud", type=int, default=57600)
    ap.add_argument("--every", type=int, default=240, help="print every Nth status frame")
    ap.add_argument("--ping", action="store_true", help="send PING first (live only)")
    ap.add_argument("--erase", action="store_true", help="ask to erase the log (refused unless on the ground)")
    a = ap.parse_args()

    if a.capture:
        fr = parse_frames(open(a.capture, "rb").read())
        show(fr, a.every)
        print(f"{len(fr.status)} status frames, {len(fr.events)} events, {fr.crc_errors} bad frames")
        return 0
    if not a.port:
        ap.error("give a capture file or --port")

    import serial  # live mode only

    port = serial.Serial(a.port, a.baud, timeout=0.2)
    if a.ping:
        port.write(frame(TC_PING))
    if a.erase:
        port.write(frame(TC_ERASE_LOG, struct.pack("<I", ERASE_MAGIC)))
    buf = b""
    while True:
        buf += port.read(256)
        cut = buf.rfind(b"\xaa\x55")
        if cut > 0:
            show(parse_frames(buf[:cut]), max(1, a.every // 60))
            buf = buf[cut:]
        sys.stdout.flush()


if __name__ == "__main__":
    raise SystemExit(main())
