"""
vayu_formats.py - decoders for Vayu's binary formats (shared by every tool).

  * black-box log records (64 bytes, see firmware/app/logger.h)
  * radio frames: 0xAA 0x55 | len | type | payload | CRC-16 (firmware/app/telemetry.h)
  * the simulator's CSV outputs (truth, boots, events)
"""
import csv
import struct
from dataclasses import dataclass, field

PHASES = ["BOOT", "PREFLIGHT", "ASCENT", "CRUISE", "DESCENT", "LANDED"]
POWER_STATES = ["NORMAL", "LOW", "CRITICAL"]
ALT_SOURCES = ["NONE", "BARO", "GPS"]
RESET_CAUSES = ["POWER_ON", "BROWN_OUT", "WATCHDOG", "SOFTWARE", "PIN", "UNKNOWN"]
FAULTS = ["BARO", "BARO_RANGE", "IMU", "IMU_STUCK", "GPS_LINK", "GPS_NOFIX", "GPS_CONFIG",
          "POWER_LOW", "POWER_CRIT", "PWRMON", "FLASH", "LOG_FULL", "CAN", "PAYLOAD",
          "TEMP_LOW", "CLOCK"]
FAULT = {name: 1 << i for i, name in enumerate(FAULTS)}
EVENTS = {1: "BOOT", 2: "PHASE", 3: "FAULTS", 4: "POWER_STATE", 5: "PONG", 6: "ERASE_STARTED",
          7: "ERASE_REFUSED", 8: "POWER_FAIL", 9: "CAN_BUS_OFF", 10: "GPS_CONFIG"}
REC_PERIODIC, REC_BOOT, REC_POWER_FAIL = 1, 2, 3

LOG_MAGIC = 0x5641
LOG_FMT = "<HHIIBBH" "iiiiii" "hHh" "hhh" "hhh" "h" "BBBB"
LOG_SIZE = 64
assert struct.calcsize(LOG_FMT) == LOG_SIZE

STATUS_FMT = "<IBBHihiiiBBhBHhHBB"
EVENT_FMT = "<IBH"
TM_STATUS, TM_EVENT = 0x01, 0x02


def _crc_table():
    table = []
    for i in range(256):
        crc = i << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
        table.append(crc)
    return table


_CRC_TABLE = _crc_table()


def crc16_ccitt(data: bytes) -> int:
    """CRC-16/CCITT-FALSE, same as firmware/app/crc.c (table-driven for speed)."""
    crc = 0xFFFF
    for b in data:
        crc = ((crc << 8) & 0xFFFF) ^ _CRC_TABLE[(crc >> 8) ^ b]
    return crc


def fault_names(bits: int) -> list:
    return [n for n, b in FAULT.items() if bits & b]


# ---------------------------------------------------------------- black box
@dataclass
class LogStats:
    records: int = 0
    crc_errors: int = 0      # torn or corrupted records
    seq_gaps: int = 0        # records missing (sequence jumps)
    boots: int = 0


def decode_log(path: str):
    """Return (records, stats). Each record is a dict; bad records are counted and skipped."""
    data = open(path, "rb").read()
    out, st = [], LogStats()
    last_seq = None
    for off in range(0, len(data) - LOG_SIZE + 1, LOG_SIZE):
        raw = data[off:off + LOG_SIZE]
        if raw == b"\xff" * LOG_SIZE:
            break
        v = struct.unpack(LOG_FMT, raw)
        if v[0] != LOG_MAGIC or v[1] != crc16_ccitt(raw[4:]):
            st.crc_errors += 1
            continue
        (magic, crc, seq, t_ms, rtype, phase, faults, alt, gps_alt, lat, lon, pres, ground,
         temp, vbat, ibat, ax, ay, az, gx, gy, gz, climb, heater, nsv, fix, info) = v
        if rtype == REC_BOOT:
            st.boots += 1
        if last_seq is not None and seq != last_seq + 1:
            st.seq_gaps += seq - last_seq - 1
        last_seq = seq
        out.append(dict(
            seq=seq, t_ms=t_ms, type=rtype, phase=phase, faults=faults,
            alt_m=alt / 100, gps_alt_m=gps_alt / 100, lat=lat / 1e7, lon=lon / 1e7,
            pressure_pa=pres, ground_alt_m=ground / 100, board_temp_c=temp / 100,
            vbat=vbat / 1000, ibat=ibat / 1000, acc_g=(ax / 1000, ay / 1000, az / 1000),
            gyro_dps=(gx / 10, gy / 10, gz / 10), climb_ms=climb / 100, heater_duty=heater,
            gps_num_sv=nsv, gps_fix=fix, alt_src=info & 3, reset_cause=(info >> 2) & 7,
            heater_on=bool(info & 0x20), power=(info >> 6) & 3))
        st.records += 1
    return out, st


# ---------------------------------------------------------------- radio
@dataclass
class Frames:
    status: list = field(default_factory=list)
    events: list = field(default_factory=list)
    all: list = field(default_factory=list)  # both kinds, in the order received
    crc_errors: int = 0


def parse_frames(data: bytes) -> Frames:
    f, i = Frames(), 0
    while i + 6 <= len(data):
        if data[i] != 0xAA or data[i + 1] != 0x55:
            i += 1
            continue
        n, typ = data[i + 2], data[i + 3]
        if i + 6 + n > len(data):
            break
        body = data[i + 2:i + 4 + n]
        crc = data[i + 4 + n] | (data[i + 5 + n] << 8)
        if crc != crc16_ccitt(body):
            f.crc_errors += 1
            i += 1
            continue
        payload = data[i + 4:i + 4 + n]
        if typ == TM_STATUS and n == struct.calcsize(STATUS_FMT):
            (t_ms, phase, power, faults, alt, climb, gps_alt, lat, lon, nsv, fix, temp, heater,
             vbat, ibat, mah, reset, src) = struct.unpack(STATUS_FMT, payload)
            f.status.append(dict(kind="status", t_ms=t_ms, phase=phase, power=power, faults=faults,
                                 alt_m=alt / 100, climb_ms=climb / 100, gps_alt_m=gps_alt / 100,
                                 lat=lat / 1e7, lon=lon / 1e7, gps_num_sv=nsv, gps_fix=fix,
                                 board_temp_c=temp / 100, heater_duty=heater, vbat=vbat / 1000,
                                 ibat=ibat / 1000, used_mah=mah, reset_cause=reset, alt_src=src))
            f.all.append(f.status[-1])
        elif typ == TM_EVENT and n == struct.calcsize(EVENT_FMT):
            t_ms, code, arg = struct.unpack(EVENT_FMT, payload)
            f.events.append(dict(kind="event", t_ms=t_ms, code=code, name=EVENTS.get(code, str(code)), arg=arg))
            f.all.append(f.events[-1])
        i += 6 + n
    return f


# ---------------------------------------------------------------- simulator outputs
def read_csv(path: str) -> list:
    with open(path, newline="") as fp:
        rows = list(csv.DictReader(fp))
    for r in rows:
        for k, v in r.items():
            try:
                r[k] = float(v)
            except (TypeError, ValueError):
                pass
    return rows


def attach_time(items: list, boots: list, is_boot) -> None:
    """Give log records / radio frames an absolute time 't' (s since power-on).

    Firmware time restarts at every boot. Each boot leaves a marker (a REC_BOOT
    record, an EVT_BOOT frame); the simulator's boots.csv says when it happened."""
    k = -1
    for it in items:
        if is_boot(it):
            k += 1
        base = boots[max(k, 0)]["sim_ms"] / 1000.0 if boots else 0.0
        it["t"] = base + it["t_ms"] / 1000.0
        it["boot"] = max(k, 0)
