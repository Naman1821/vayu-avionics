#!/usr/bin/env python3
"""
test_scenarios.py - fly every scenario in the digital twin and check, from the
black-box log and the radio downlink alone, that the firmware did the right thing.

    python3 tests/test_scenarios.py            # all scenarios
    python3 tests/test_scenarios.py brownout   # one

Writes out/results.json (used by the plots) and docs/RESULTS.md.
"""
import bisect
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "tools"))
from vayu_formats import (ALT_SOURCES, FAULT, PHASES, REC_BOOT, REC_POWER_FAIL,  # noqa: E402
                          attach_time, decode_log, parse_frames, read_csv)

OUT = ROOT / "out"
SIM = ROOT / "build" / "vayu_sim"
BARO, GPS = ALT_SOURCES.index("BARO"), ALT_SOURCES.index("GPS")
ASCENT, CRUISE, DESCENT, LANDED = (PHASES.index(p) for p in ("ASCENT", "CRUISE", "DESCENT", "LANDED"))


class Flight:
    """Everything one simulated flight left behind, on a common time axis."""

    def __init__(self, name):
        d = OUT / name
        self.name = name
        self.truth = read_csv(d / "truth.csv")
        self.boots = read_csv(d / "boots.csv")
        self.sim_events = read_csv(d / "events.csv")
        self.recs, self.log = decode_log(str(d / "flash.bin"))
        attach_time(self.recs, self.boots, lambda r: r["type"] == REC_BOOT)
        self.periodic = [r for r in self.recs if r["type"] == 1]
        self.tm = parse_frames((d / "downlink.bin").read_bytes())
        attach_time(self.tm.all, self.boots, lambda x: x.get("code") == 1)
        self._tt = [r["t_s"] for r in self.truth]

    def truth_at(self, t, key="alt_m"):
        i = min(max(bisect.bisect_left(self._tt, t), 1), len(self._tt) - 1)
        a, b = self.truth[i - 1], self.truth[i]
        f = (t - a["t_s"]) / (b["t_s"] - a["t_s"])
        return a[key] + f * (b[key] - a[key])

    def recs_between(self, t0, t1):
        return [r for r in self.periodic if t0 <= r["t"] < t1]

    def first(self, pred, after=0.0):
        for r in self.periodic:
            if r["t"] >= after and pred(r):
                return r["t"]
        return None

    def alt_error(self, t0=300, t1=None, phases=(ASCENT, CRUISE, DESCENT)):
        t1 = t1 or 1e9
        errs = [abs(r["alt_m"] - self.truth_at(r["t"])) for r in self.periodic
                if t0 <= r["t"] < t1 and r["phase"] in phases]
        return max(errs) if errs else float("nan")

    def phase_sequence(self):
        seq = []
        for r in self.recs:
            name = PHASES[r["phase"]]
            if name != "BOOT" and (not seq or seq[-1] != name):
                seq.append(name)
        return seq

    def events(self, name):
        return [e for e in self.tm.events if e["name"] == name]

    def sim_said(self, text):
        return [e for e in self.sim_events if text in str(e["event"])]


# --------------------------------------------------------------------------------
def common_checks(f, resets=0, torn_allowed=0):
    expected = ["PREFLIGHT", "ASCENT", "CRUISE", "DESCENT", "LANDED"]
    return [
        ("black box intact (CRC, sequence)",
         f.log.crc_errors <= torn_allowed and f.log.seq_gaps == 0,
         f"{f.log.records} records, {f.log.crc_errors} torn, {f.log.seq_gaps} missing"),
        ("every radio frame valid", f.tm.crc_errors == 0 and len(f.tm.status) > 1000,
         f"{len(f.tm.status)} status frames, {f.tm.crc_errors} bad"),
        ("flight phases in order, ends LANDED", f.phase_sequence() == expected,
         " > ".join(f.phase_sequence())),
        ("resets", len(f.boots) - 1 == resets, f"{len(f.boots) - 1} (expected {resets})"),
    ]


def check_nominal(f):
    launch = f.first(lambda r: r["phase"] == ASCENT)
    above12 = [r for r in f.periodic if r["alt_m"] > 12100]
    fix_frac = sum(r["gps_fix"] == 3 for r in above12) / max(len(above12), 1)
    cruise = [r for r in f.periodic if r["phase"] == CRUISE]
    temps = [r["board_temp_c"] for r in cruise]
    refused = f.events("ERASE_REFUSED")
    return common_checks(f) + [
        ("launch detected within 10 s", launch is not None and 0 <= launch - 300 <= 10,
         f"{launch - 300:.1f} s after lift-off"),
        ("altitude error in flight <= 10 m", f.alt_error() <= 10, f"max {f.alt_error():.1f} m"),
        ("GPS 3D fix kept above 12 km", fix_frac >= 0.99, f"{100 * fix_frac:.1f} % of samples"),
        ("box held 0..5 C at 50,000 ft (+/-2 C)", min(temps) >= -2 and max(temps) <= 7,
         f"{min(temps):.1f} .. {max(temps):.1f} C in -56.5 C air"),
        ("no faults while cruising", all(r["faults"] == 0 for r in cruise), "fault bits all clear"),
        ("ground PING answered", len(f.events("PONG")) == 1, "PONG received"),
        ("in-flight ERASE refused, log kept", len(refused) == 1 and len(f.events("ERASE_STARTED")) == 0,
         f"refused in {PHASES[refused[0]['arg']]}" if refused else "not refused"),
    ]


def check_baro_failure(f):
    det = f.first(lambda r: r["faults"] & FAULT["BARO"], after=1000)
    during = f.recs_between(1001, 1059)
    back = f.first(lambda r: r["alt_src"] == BARO and not r["faults"] & FAULT["BARO"], after=1060)
    phases = {r["phase"] for r in f.recs_between(990, 1070)}
    return common_checks(f) + [
        ("barometer loss detected <= 0.5 s", det is not None and det - 1000 <= 0.5, f"{det - 1000:.2f} s"),
        ("altitude switched to GPS", during and all(r["alt_src"] == GPS for r in during), "GPS for 58 s"),
        ("altitude error during outage <= 30 m", f.alt_error(1000, 1061) <= 30,
         f"max {f.alt_error(1000, 1061):.1f} m"),
        ("barometer back within 3 s", back is not None and back - 1060 <= 3, f"{back - 1060:.2f} s"),
        ("no false phase change", phases == {ASCENT}, "stayed ASCENT"),
    ]


def check_i2c_stuck(f):
    healthy = f.first(lambda r: not r["faults"] & (FAULT["BARO"] | FAULT["PWRMON"]) and r["alt_src"] == BARO,
                      after=1200.2)
    return common_checks(f) + [
        ("stuck bus freed by 9-clock recovery", len(f.sim_said("bus recovered")) == 1, "1 recovery"),
        ("barometer + power monitor back within 3 s", healthy is not None and healthy - 1200 <= 3,
         f"{healthy - 1200:.2f} s"),
        ("altitude error <= 30 m", f.alt_error() <= 30, f"max {f.alt_error():.1f} m"),
    ]


def check_gps_outage(f):
    det = f.first(lambda r: r["faults"] & FAULT["GPS_NOFIX"], after=2700)
    clear = f.first(lambda r: not r["faults"] & FAULT["GPS_NOFIX"], after=2760)
    during = f.recs_between(2700, 2760)
    return common_checks(f) + [
        ("loss of fix reported <= 6 s", det is not None and det - 2700 <= 6, f"{det - 2700:.1f} s"),
        ("fix back <= 10 s after signal returns", clear is not None and clear - 2760 <= 10,
         f"{clear - 2760:.1f} s"),
        ("altitude stayed on barometer", all(r["alt_src"] == BARO for r in during), "BARO throughout"),
        ("altitude error <= 10 m", f.alt_error() <= 10, f"max {f.alt_error():.1f} m"),
    ]


def check_brownout(f):
    reset_t = f.boots[1]["sim_ms"] / 1000 if len(f.boots) > 1 else None
    pf = [r for r in f.recs if r["type"] == REC_POWER_FAIL]
    after = [r for r in f.recs if r["boot"] == 1]
    gps_ok = [e for e in f.events("GPS_CONFIG") if e["arg"] == 1 and reset_t and e["t"] > reset_t]
    fix_back = f.first(lambda r: r["gps_fix"] == 3, after=reset_t or 0)
    last_before = max((r["t"] for r in f.recs if r["boot"] == 0), default=0)
    gap = after[0]["t"] - last_before if after else 99
    return common_checks(f, resets=1, torn_allowed=1) + [
        ("reset cause logged as BROWN_OUT", len(f.boots) == 2 and f.boots[1]["cause"] == "brown-out",
         str(f.boots[1]["cause"]) if len(f.boots) > 1 else "-"),
        ("power-fail record saved before the reset", len(pf) == 1 and pf[0]["t"] < reset_t,
         f"{reset_t - pf[0]['t']:.3f} s before" if pf else "missing"),
        ("resumed in ASCENT (warm restart)", bool(after) and after[0]["phase"] == ASCENT,
         PHASES[after[0]["phase"]] if after else "-"),
        ("GPS airborne mode re-applied <= 5 s", bool(gps_ok) and gps_ok[0]["t"] - reset_t <= 5,
         f"{gps_ok[0]['t'] - reset_t:.2f} s" if gps_ok else "never"),
        ("GPS fix back at 13.5 km <= 10 s", fix_back is not None and fix_back - reset_t <= 10,
         f"{fix_back - reset_t:.1f} s"),
        ("logging gap <= 1 s", gap <= 1.0, f"{gap:.2f} s"),
    ]


def check_cpu_hang(f):
    reset_t = f.boots[1]["sim_ms"] / 1000 if len(f.boots) > 1 else None
    after = [r for r in f.recs if r["boot"] == 1]
    last_before = max((r["t"] for r in f.recs if r["boot"] == 0), default=0)
    gap = after[0]["t"] - last_before if after else 99
    return common_checks(f, resets=1) + [
        ("watchdog reset", len(f.boots) == 2 and f.boots[1]["cause"] == "watchdog",
         str(f.boots[1]["cause"]) if len(f.boots) > 1 else "-"),
        ("recovered <= 0.6 s after the hang", reset_t is not None and reset_t - 3000 <= 0.6,
         f"{reset_t - 3000:.3f} s"),
        ("resumed in CRUISE (warm restart)", bool(after) and after[0]["phase"] == CRUISE,
         PHASES[after[0]["phase"]] if after else "-"),
        ("logging gap <= 1 s", gap <= 1.0, f"{gap:.2f} s"),
    ]


def check_can_bus_off(f):
    can = f.first(lambda r: r["faults"] & FAULT["CAN"], after=2500)
    pay = f.first(lambda r: r["faults"] & FAULT["PAYLOAD"], after=2500)
    clear = f.first(lambda r: not r["faults"] & (FAULT["CAN"] | FAULT["PAYLOAD"]), after=2510)
    return common_checks(f) + [
        ("bus-off event sent to ground", len(f.events("CAN_BUS_OFF")) >= 1,
         f"{len(f.events('CAN_BUS_OFF'))} event(s)"),
        ("CAN fault raised <= 0.2 s", can is not None and can - 2500 <= 0.2, f"{can - 2500:.2f} s"),
        ("payload loss raised <= 4 s", pay is not None and pay - 2500 <= 4, f"{pay - 2500:.1f} s"),
        ("bus and payload back <= 5 s after repair", clear is not None and clear - 2510 <= 5,
         f"{clear - 2510:.1f} s"),
    ]


def check_imu_stuck(f):
    det = f.first(lambda r: r["faults"] & FAULT["IMU_STUCK"], after=1000)
    return common_checks(f) + [
        ("frozen IMU detected <= 1.5 s", det is not None and det - 1000 <= 1.5, f"{det - 1000:.2f} s"),
    ]


def check_gps_rejects(f):
    above = [r for r in f.periodic if r["alt_m"] > 12100]
    fix_frac = sum(r["gps_fix"] == 3 for r in above) / max(len(above), 1)
    return common_checks(f) + [
        ("config failure reported (GPS_CONFIG)", any(r["faults"] & FAULT["GPS_CONFIG"] for r in f.periodic),
         "fault bit set"),
        ("consequence: no fix above 12 km", fix_frac <= 0.01, f"{100 * fix_frac:.1f} % fixes above 12 km"),
        ("barometer keeps altitude right", f.alt_error() <= 10, f"max {f.alt_error():.1f} m"),
    ]


def check_no_heater(f):
    tmin = min(r["box_c"] for r in f.truth)
    return common_checks(f) + [
        ("box falls below the -40 C rating", tmin < -40, f"min {tmin:.1f} C"),
        ("firmware reports TEMP_LOW", any(r["faults"] & FAULT["TEMP_LOW"] for r in f.periodic), "fault bit set"),
    ]


def check_low_battery(f):
    low = f.first(lambda r: r["power"] >= 1)
    crit = f.first(lambda r: r["power"] == 2)
    duty_before = [r["heater_duty"] for r in f.recs_between((low or 0) - 1200, low or 0)]
    duty_after = [r["heater_duty"] for r in f.recs_between((low or 0) + 300, (crit or 1e9))]
    in_crit = [r for r in f.periodic if r["power"] == 2]
    vmin = min(r["vbat"] for r in f.truth)
    avg = lambda xs: sum(xs) / max(len(xs), 1)  # noqa: E731
    return common_checks(f) + [
        ("LOW, then CRITICAL, in flight", low is not None and crit is not None and low < crit,
         f"LOW at {low:.0f} s, CRITICAL at {crit:.0f} s" if low and crit else "-"),
        ("heater throttled in LOW", avg(duty_after) < 0.5 * avg(duty_before),
         f"duty {avg(duty_before):.0f} % -> {avg(duty_after):.0f} %"),
        ("heater off the moment CRITICAL starts", bool(in_crit) and not any(r["heater_on"] for r in in_crit),
         f"{sum(r['heater_on'] for r in in_crit)} of {len(in_crit)} CRITICAL samples with heater on"),
        ("no brown-out, logged to landing", vmin > 5.6 and not f.sim_said("exhausted"), f"min {vmin:.2f} V"),
    ]


CHECKS = {
    "nominal": check_nominal, "baro_failure": check_baro_failure, "i2c_bus_stuck": check_i2c_stuck,
    "gps_outage": check_gps_outage, "brownout": check_brownout, "cpu_hang": check_cpu_hang,
    "can_bus_off": check_can_bus_off, "imu_stuck": check_imu_stuck,
    "gps_rejects_airborne": check_gps_rejects, "no_heater": check_no_heater, "low_battery": check_low_battery,
}


def run(name):
    subprocess.run([str(SIM), name], cwd=ROOT, check=True, stdout=subprocess.DEVNULL)
    f = Flight(name)
    results = []
    for desc, ok, detail in CHECKS[name](f):
        results.append({"check": desc, "ok": bool(ok), "detail": detail})
    return f, results


def main():
    names = sys.argv[1:] or list(CHECKS)
    about = {}
    for line in subprocess.run([str(SIM), "--list"], capture_output=True, text=True).stdout.splitlines():
        n, _, text = line.partition(" ")
        about[n] = text.strip()

    report, failed = {}, 0
    for name in names:
        f, results = run(name)
        bad = [r for r in results if not r["ok"]]
        failed += len(bad)
        print(f"{'PASS' if not bad else 'FAIL'}  {name:22s} {sum(r['ok'] for r in results)}/{len(results)} checks")
        for r in results:
            print(f"      {'ok ' if r['ok'] else 'BAD'} {r['check']:45s} {r['detail']}")
        report[name] = {"about": about.get(name, ""), "results": results,
                        "energy_wh": f.truth[-1]["energy_wh"], "heater_wh": f.truth[-1]["heater_wh"],
                        "duration_s": f.truth[-1]["t_s"]}

    OUT.mkdir(exist_ok=True)
    (OUT / "results.json").write_text(json.dumps(report, indent=1))
    if len(names) == len(CHECKS):
        write_markdown(report)
    total = sum(len(v["results"]) for v in report.values())
    print(f"\n{len(names)} scenarios, {total} checks, {failed} failed")
    return 1 if failed else 0


def write_markdown(report):
    lines = ["# Simulated flight results", "",
             "Generated by `make scenarios` (tests/test_scenarios.py). Every check is made only from",
             "what the firmware wrote to its black box and sent over the radio, compared with the",
             "simulator's ground truth. All flights: 18,000 ft launch site at -20 C, climb to",
             "50,000 ft, 60 min loiter at -56.5 C, descend, land (about 2 hours).", ""]
    for name, v in report.items():
        ok = all(r["ok"] for r in v["results"])
        lines += [f"## {'PASS' if ok else 'FAIL'}: `{name}`", "", v["about"], "",
                  "| Check | Result | Measured |", "|---|---|---|"]
        lines += [f"| {r['check']} | {'pass' if r['ok'] else '**FAIL**'} | {r['detail']} |" for r in v["results"]]
        lines.append("")
    (ROOT / "docs" / "RESULTS.md").write_text("\n".join(lines))


if __name__ == "__main__":
    sys.exit(main())
