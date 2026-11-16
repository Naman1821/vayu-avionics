#!/usr/bin/env python3
"""
plot_flight.py - figures for the README, made only from what the firmware
logged (black box) plus the simulator's ground truth.

    python3 tools/plot_flight.py      # runs any missing scenarios, writes docs/img/*.png
"""
import math
import subprocess
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "tests"))
from test_scenarios import Flight  # noqa: E402
from vayu_formats import PHASES  # noqa: E402

IMG = ROOT / "docs" / "img"

# validated categorical order (blue, orange, aqua) + neutral chrome
BLUE, ORANGE, AQUA = "#2a78d6", "#eb6834", "#1baf7a"
SURFACE, INK, INK2, MUTED, GRID, AXIS = "#fcfcfb", "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7"
BAND = "#f1f0ec"

plt.rcParams.update({
    "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
    "axes.edgecolor": AXIS, "axes.labelcolor": INK2, "axes.titlecolor": INK,
    "axes.titlesize": 11, "axes.titleweight": "bold", "axes.titlelocation": "left",
    "axes.labelsize": 9, "xtick.color": MUTED, "ytick.color": MUTED,
    "xtick.labelsize": 8, "ytick.labelsize": 8, "axes.grid": True, "grid.color": GRID,
    "grid.linewidth": 0.6, "grid.linestyle": "-", "axes.spines.top": False,
    "axes.spines.right": False, "lines.linewidth": 2, "legend.frameon": False,
    "legend.fontsize": 8, "font.size": 9,
})


def flight(name):
    if not (ROOT / "out" / name / "flash.bin").exists():
        subprocess.run([str(ROOT / "build" / "vayu_sim"), name], cwd=ROOT, check=True, stdout=subprocess.DEVNULL)
    return Flight(name)


def thin(xs, ys, step):
    return xs[::step], ys[::step]


def label_end(ax, x, y, text, color):
    ax.annotate(text, (x, y), xytext=(4, 0), textcoords="offset points", va="center",
                fontsize=8, color=INK2)
    ax.plot([x], [y], "o", ms=4, color=color)


def phase_bands(ax, f, ymax_label=True):
    spans, cur, start = [], None, 0.0
    for r in f.periodic:
        if r["phase"] != cur:
            if cur is not None:
                spans.append((cur, start, r["t"]))
            cur, start = r["phase"], r["t"]
    spans.append((cur, start, f.periodic[-1]["t"]))
    for i, (p, a, b) in enumerate(spans):
        if i % 2 == 1:
            ax.axvspan(a / 60, b / 60, color=BAND, lw=0, zorder=0)
        if ymax_label:
            ax.text((a + b) / 120, 1.01, PHASES[p].capitalize(), transform=ax.get_xaxis_transform(),
                    ha="center", va="bottom", fontsize=7.5, color=MUTED)


# ------------------------------------------------------------------ figures
def mission_overview(f):
    t = [r["t"] / 60 for r in f.periodic]
    fig, axes = plt.subplots(3, 1, figsize=(9, 7.2), sharex=True, gridspec_kw={"hspace": 0.45})

    ax = axes[0]
    phase_bands(ax, f)
    x, y = thin(t, [r["alt_m"] / 1000 for r in f.periodic], 10)
    ax.plot(x, y, color=BLUE)
    ax.set_ylabel("km above sea level")
    ax.set_title("Altitude the firmware logged", pad=16)
    ax.text(0.99, 0.08, f"max error vs truth: {f.alt_error():.1f} m", transform=ax.transAxes,
            ha="right", fontsize=8, color=INK2)

    ax = axes[1]
    phase_bands(ax, f, False)
    tt = [r["t_s"] / 60 for r in f.truth]
    ax.plot(tt, [r["amb_c"] for r in f.truth], color=ORANGE, label="outside air (truth)")
    x, y = thin(t, [r["board_temp_c"] for r in f.periodic], 10)
    ax.plot(x, y, color=BLUE, label="avionics box (logged)")
    ax.axhline(-40, color=MUTED, lw=1)
    ax.text(t[-1] * 0.995, -37, "parts rated to -40 C", ha="right", fontsize=7.5, color=MUTED)
    label_end(ax, tt[-1], f.truth[-1]["amb_c"], "outside air", ORANGE)
    label_end(ax, x[-1], y[-1], "box", BLUE)
    ax.set_ylabel("deg C")
    ax.set_title("Heater holds the box at 0 to 5 C in -56.5 C air")
    ax.legend(loc="center", ncol=2)

    ax = axes[2]
    phase_bands(ax, f, False)
    x, y = thin(t, [r["heater_duty"] for r in f.periodic], 10)
    ax.plot(x, y, color=BLUE)
    ax.set_ylim(0, 105)
    ax.set_ylabel("% of last 60 s")
    ax.set_xlabel("minutes since power-on")
    ax.set_title("Heater duty cycle")
    fig.savefig(IMG / "mission_overview.png", dpi=150, bbox_inches="tight")
    plt.close(fig)


def gps_airborne(nominal, rejected):
    fig, ax = plt.subplots(figsize=(8, 2.6))
    rows = ((rejected, ORANGE, "receiver left in\n'portable' mode"), (nominal, BLUE, "airborne mode\nconfirmed (default)"))
    for y, (f, color, name) in enumerate(rows):
        climb = [r for r in f.periodic if PHASES[r["phase"]] == "ASCENT"]
        segs, start, prev = [], None, None
        for r in climb:
            alt, fix = r["alt_m"] / 1000, r["gps_fix"] == 3
            if fix and start is None:
                start = alt
            if not fix and start is not None:
                segs.append((start, prev - start))
                start = None
            prev = alt
        if start is not None:
            segs.append((start, prev - start))
        ax.broken_barh(segs, (y - 0.3, 0.6), color=color)
        covered = sum(w for _, w in segs)
        top = max(r["alt_m"] for r in climb) / 1000
        if covered < top - climb[0]["alt_m"] / 1000 - 0.5:
            ax.text(13.6, y, "no fix above 12 km", va="center", ha="center", fontsize=8, color=INK2)
    ax.set_yticks([0, 1], [r[2] for r in rows])
    ax.tick_params(axis="y", length=0, labelsize=8, labelcolor=INK2)
    ax.grid(axis="y", visible=False)
    ax.axvline(12, color=MUTED, lw=1)
    ax.text(12.05, 1.55, "12 km: altitude limit of the default model", fontsize=7.5, color=MUTED, va="center")
    ax.set_ylim(-0.6, 1.8)
    ax.set_xlim(5.4, 15.4)
    ax.set_xlabel("altitude during the climb, km  (bar = GPS 3D fix)")
    ax.set_title("Why the firmware sets, then reads back, the GPS dynamic model")
    fig.savefig(IMG / "gps_airborne.png", dpi=150, bbox_inches="tight")
    plt.close(fig)


def heater_whatif(nominal, no_heater, low_batt):
    fig, ax = plt.subplots(figsize=(8, 3.6))
    for f, color, name in ((nominal, BLUE, "heater, full battery"), (low_batt, AQUA, "heater, battery at 40 %"),
                           (no_heater, ORANGE, "no heater")):
        x = [r["t_s"] / 60 for r in f.truth][::10]
        y = [r["box_c"] for r in f.truth][::10]
        ax.plot(x, y, color=color, label=name)
        label_end(ax, x[-1], y[-1], name, color)
    ax.axhline(-40, color=MUTED, lw=1)
    ax.text(1, -38.8, "parts rated to -40 C", fontsize=7.5, color=MUTED)
    ax.set_xlabel("minutes since power-on")
    ax.set_ylabel("box temperature, deg C")
    ax.set_xlim(0, 150)
    ax.set_title("Avionics box temperature, 2-hour mission to 50,000 ft")
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.2), ncol=3)
    fig.savefig(IMG / "heater_whatif.png", dpi=150, bbox_inches="tight")
    plt.close(fig)


def low_battery(f):
    t = [r["t"] / 60 for r in f.periodic]
    fig, axes = plt.subplots(2, 1, figsize=(8, 4.8), sharex=True, gridspec_kw={"hspace": 0.45})
    ax = axes[0]
    x, y = thin(t, [r["vbat"] for r in f.periodic], 10)
    ax.plot(x, y, color=BLUE)
    for v, name in ((6.6, "LOW 6.6 V"), (6.2, "CRITICAL 6.2 V")):
        ax.axhline(v, color=MUTED, lw=1)
        ax.text(1, v + 0.03, name, fontsize=7.5, color=MUTED)
    low = f.first(lambda r: r["power"] >= 1)
    crit = f.first(lambda r: r["power"] == 2)
    ax.annotate("enters LOW: heater only\nkeeps the box above -20 C", (low / 60, 6.6), xytext=(46, 6.36),
                fontsize=7.5, color=INK2, arrowprops=dict(arrowstyle="-", color=MUTED, lw=0.8))
    ax.annotate("enters CRITICAL:\nheater off", (crit / 60, 6.22), xytext=(104, 6.88),
                fontsize=7.5, color=INK2, arrowprops=dict(arrowstyle="-", color=MUTED, lw=0.8))
    ax.set_ylabel("battery, V (logged)")
    ax.set_title("Low battery: the firmware sheds load and keeps logging to landing")
    ax = axes[1]
    x, y = thin(t, [r["heater_duty"] for r in f.periodic], 10)
    ax.plot(x, y, color=BLUE)
    ax.set_ylabel("heater duty, %")
    ax.set_xlabel("minutes since power-on")
    ax.set_ylim(0, 105)
    fig.savefig(IMG / "low_battery.png", dpi=150, bbox_inches="tight")
    plt.close(fig)


# --- the two "why it matters" calculations, from first principles -------------
def cdiv(a, b):
    q = abs(a) // abs(b)
    return q if (a >= 0) == (b > 0) else -q


def ms5611(c, d1, d2, second_order=True):
    dt = d2 - (c[5] << 8)
    temp = 2000 + cdiv(dt * c[6], 1 << 23)
    off = (c[2] << 16) + cdiv(c[4] * dt, 1 << 7)
    sens = (c[1] << 15) + cdiv(c[3] * dt, 1 << 8)
    if second_order and temp < 2000:
        t2 = cdiv(dt * dt, 1 << 31)
        off2 = cdiv(5 * (temp - 2000) ** 2, 2)
        sens2 = cdiv(5 * (temp - 2000) ** 2, 4)
        if temp < -1500:
            off2 += 7 * (temp + 1500) ** 2
            sens2 += cdiv(11 * (temp + 1500) ** 2, 2)
        temp, off, sens = temp - t2, off - off2, sens - sens2
    return cdiv(cdiv(d1 * sens, 1 << 21) - off, 1 << 15), temp


def isa_alt(p):
    if p >= 22632.06:
        return 288.15 / 0.0065 * (1 - (p / 101325) ** (287.053 * 0.0065 / 9.80665))
    return 11000 + 287.053 * 216.65 / 9.80665 * math.log(22632.06 / p)


def isa_p(h):
    if h < 11000:
        return 101325 * (1 - 0.0065 * h / 288.15) ** 5.25588
    return 22632.06 * math.exp(-(h - 11000) * 9.80665 / (287.053 * 216.65))


def design_checks():
    fig, axes = plt.subplots(1, 2, figsize=(9.5, 3.4), gridspec_kw={"wspace": 0.35})

    ax = axes[0]
    hs = [i * 100 for i in range(0, 201)]
    err = [44330 * (1 - (isa_p(h) / 101325) ** 0.190295) - h for h in hs]
    ax.plot([h / 1000 for h in hs], err, color=BLUE)
    e50 = 44330 * (1 - (isa_p(15240) / 101325) ** 0.190295) - 15240
    ax.plot([15.24], [e50], "o", ms=5, color=BLUE)
    ax.annotate(f"{e50:.0f} m at 50,000 ft", (15.24, e50), xytext=(-95, -4), textcoords="offset points",
                fontsize=8, color=INK2)
    ax.set_xlabel("true altitude, km")
    ax.set_ylabel("error, m")
    ax.set_title("One-line altitude formula vs. two-layer ISA")

    ax = axes[1]
    c = [0, 40127, 36924, 23317, 23282, 33464, 28312, 0]
    p_true = isa_p(5486.4)
    temps = list(range(-40, 21))
    errs = []
    for tc in temps:
        want_t, want_p = round(tc * 100), round(p_true)
        lo, hi = 0, 0xFFFFFF
        while lo < hi:
            mid = (lo + hi) // 2
            lo, hi = (mid + 1, hi) if ms5611(c, 0, mid)[1] < want_t else (lo, mid)
        d2 = lo
        lo, hi = 0, 0xFFFFFF
        while lo < hi:
            mid = (lo + hi) // 2
            lo, hi = (mid + 1, hi) if ms5611(c, mid, d2)[0] < want_p else (lo, mid)
        p1, _ = ms5611(c, lo, d2, second_order=False)
        errs.append(isa_alt(p1) - isa_alt(ms5611(c, lo, d2)[0]))
    ax.plot(temps, errs, color=BLUE)
    e20 = errs[temps.index(-20)]
    ax.plot([-20], [e20], "o", ms=5, color=BLUE)
    ax.annotate(f"{e20:.0f} m at the -20 C launch site", (-20, e20), xytext=(8, -4), textcoords="offset points",
                fontsize=8, color=INK2)
    ax.set_xlabel("sensor temperature, deg C")
    ax.set_ylabel("altitude error, m")
    ax.set_title("MS5611 without 2nd-order compensation")
    fig.savefig(IMG / "design_checks.png", dpi=150, bbox_inches="tight")
    plt.close(fig)


def main():
    IMG.mkdir(parents=True, exist_ok=True)
    nominal = flight("nominal")
    mission_overview(nominal)
    gps_airborne(nominal, flight("gps_rejects_airborne"))
    heater_whatif(nominal, flight("no_heater"), flight("low_battery"))
    low_battery(flight("low_battery"))
    design_checks()
    print("wrote", ", ".join(sorted(p.name for p in IMG.glob("*.png"))))


if __name__ == "__main__":
    main()
