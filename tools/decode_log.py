#!/usr/bin/env python3
"""
decode_log.py - read a black-box dump (W25Q128 contents) and summarise it.

    python3 tools/decode_log.py out/nominal/flash.bin
    python3 tools/decode_log.py out/brownout/flash.bin --csv brownout_log.csv

On real hardware the dump comes off the flash chip (or over the debug port);
in the digital twin the simulator writes it to out/<scenario>/flash.bin.
"""
import argparse
import csv
from collections import Counter

from vayu_formats import (PHASES, POWER_STATES, REC_BOOT, REC_POWER_FAIL, RESET_CAUSES,
                          decode_log, fault_names)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dump", help="flash image")
    ap.add_argument("--csv", help="write every record to this CSV file")
    a = ap.parse_args()

    recs, st = decode_log(a.dump)
    if not recs:
        print("no valid records")
        return 1

    print(f"{st.records} records, {st.boots} boot(s), {st.crc_errors} torn/corrupt, {st.seq_gaps} missing")
    boot = 0
    for r in recs:
        if r["type"] == REC_BOOT:
            boot += 1
            print(f"  boot {boot}: reset cause {RESET_CAUSES[r['reset_cause']]}, resumed in {PHASES[r['phase']]}")
        elif r["type"] == REC_POWER_FAIL:
            print(f"  power-fail warning logged at t={r['t_ms'] / 1000:.3f} s (boot {boot}), battery {r['vbat']:.2f} V")

    phases = Counter(PHASES[r["phase"]] for r in recs)
    print("  samples per phase: " + ", ".join(f"{p} {n}" for p, n in phases.items()))
    seen = 0
    for r in recs:
        seen |= r["faults"]
    print(f"  fault bits ever set: {', '.join(fault_names(seen)) or 'none'}")
    print(f"  max altitude {max(r['alt_m'] for r in recs):.0f} m, "
          f"board temperature {min(r['board_temp_c'] for r in recs):.1f} .. {max(r['board_temp_c'] for r in recs):.1f} C, "
          f"lowest battery {min(r['vbat'] for r in recs if r['vbat'] > 0):.2f} V "
          f"({POWER_STATES[max(r['power'] for r in recs)]} at worst)")

    if a.csv:
        flat = []
        for r in recs:
            row = {k: v for k, v in r.items() if k not in ("acc_g", "gyro_dps")}
            row.update(acc_x=r["acc_g"][0], acc_y=r["acc_g"][1], acc_z=r["acc_g"][2],
                       gyro_x=r["gyro_dps"][0], gyro_y=r["gyro_dps"][1], gyro_z=r["gyro_dps"][2],
                       phase=PHASES[r["phase"]], faults="|".join(fault_names(r["faults"])))
            flat.append(row)
        with open(a.csv, "w", newline="") as fp:
            w = csv.DictWriter(fp, fieldnames=list(flat[0]))
            w.writeheader()
            w.writerows(flat)
        print(f"wrote {a.csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
