#!/usr/bin/env python3
"""
retract_audit.py - classify every RETRACT attempt in HAUCS Pi logs.

Purpose: a "Retract timeout" alone is ambiguous. It can mean the servo drove
into the mechanical stop for the full RETRACT_SEC (stall, damaging), or that
the servo never moved at all because the signal pin was open (harmless).
The Hall reading comes over I2C from the ADS1115, independent of the servo
signal line on GPIO, so the Hall trace separates the two cases.

Usage:
    python3 retract_audit.py cc_*.log
    python3 retract_audit.py --csv out.csv cc_*.log

Verdicts:
    OK              reached settle, retract completed normally
    NO-MOTION       Hall never moved -> servo was not driving (signal open?)
    STALLED         Hall fell then went flat -> drove into the stop and held
    IMPAIRED        Hall moved but never settled and never went flat
    SENSOR-SUSPECT  Hall readings outside the plausible 0..13000 window
"""

import sys
import re
import csv
import glob
from datetime import datetime

# Tuning. MOVE_TH is in raw Hall counts; the winch spans roughly 2500..12285,
# so a few hundred counts is well above sensor noise but far below real travel.
MOVE_TH = 200      # total travel below this = "never moved"
FLAT_TH = 30       # per-sample change below this counts as no progress
FLAT_MIN = 3.0     # seconds of no progress at the end to call it a stall
HALL_LO = 0        # plausible reading window; outside = suspect sensor
HALL_HI = 13000

# Hall reading at or below which retract_adaptive() takes the settle branch
# and calls neutral(). Flat readings below this line mean the winch finished
# and the servo stopped - NOT that it is grinding against a stop. Must match
# HALL_TARGET + RETRACT_SETTLE in the config the log was produced under.
SETTLE_HALL = 2500 + 50

TS = re.compile(r"^(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2})[,.]?(\d*)")
RE_START = re.compile(r"RETRACT for ([\d.]+)s")
RE_HALL = re.compile(r"currnt hall_raw val:\s*(-?\d+)")
RE_DONE = re.compile(r"Fully retractd")
RE_TIMEOUT = re.compile(r"Retract timeout")
RE_ABORT = re.compile(r"SAFETY ABORT")
RE_SERVO = re.compile(r"servo\.value\s+(-?[\d.]+)")


def stamp(line):
    m = TS.match(line)
    if not m:
        return None
    base = datetime.strptime(m.group(1), "%Y-%m-%d %H:%M:%S")
    frac = m.group(2)
    if frac:
        base = base.replace(microsecond=int(frac.ljust(6, "0")[:6]))
    return base


def classify(ev):
    halls = ev["halls"]
    if not halls:
        return "NO-DATA", "no Hall readings logged"

    lo, hi = min(halls), max(halls)
    if lo < HALL_LO or hi > HALL_HI:
        return "SENSOR-SUSPECT", "Hall out of range (%d..%d)" % (lo, hi)

    travel = halls[0] - min(halls)

    if ev["done"]:
        return "OK", "settled, travel %d counts" % travel

    if travel < MOVE_TH:
        return "NO-MOTION", "Hall flat at ~%d for the whole attempt" % halls[0]

    # Below the settle line the code neutrals the servo, so a flat trace here
    # means finished-and-stopped. The timeout warning in the log is spurious:
    # st["RETRACTED"] was already 1, so retract_adaptive() never returned True.
    if halls[-1] < SETTLE_HALL:
        return "SETTLED-NO-FLAG", ("reached %d, below settle line %d; servo "
                                   "neutraled, timeout warning is spurious"
                                   % (halls[-1], SETTLE_HALL))

    # Walk back from the end counting how long progress had stopped.
    flat_n = 0
    for i in range(len(halls) - 1, 0, -1):
        if abs(halls[i - 1] - halls[i]) < FLAT_TH:
            flat_n += 1
        else:
            break
    flat_s = flat_n * ev["dt"] if ev["dt"] else 0.0

    if flat_s >= FLAT_MIN:
        return "STALLED", "fell to %d then flat %.1fs (%d samples)" % (
            min(halls), flat_s, flat_n)

    return "IMPAIRED", "travel %d counts, ended at %d, no settle" % (
        travel, halls[-1])


def parse(path):
    events, cur = [], None
    for line in open(path, errors="ignore"):
        t = stamp(line)

        m = RE_START.search(line)
        if m:
            if cur:
                events.append(cur)
            cur = {"file": path, "t0": t, "dur": float(m.group(1)),
                   "halls": [], "times": [], "done": False,
                   "timeout": False, "abort": False, "servo": None,
                   "dt": 0.0}
            continue

        if cur is None:
            continue

        m = RE_SERVO.search(line)
        if m and cur["servo"] is None:
            cur["servo"] = float(m.group(1))

        m = RE_HALL.search(line)
        if m:
            cur["halls"].append(int(m.group(1)))
            cur["times"].append(t)
            continue

        if RE_DONE.search(line):
            cur["done"] = True
            cur["t1"] = t
        elif RE_TIMEOUT.search(line):
            cur["timeout"] = True
            cur["t1"] = t
        elif RE_ABORT.search(line):
            cur["abort"] = True

    if cur:
        events.append(cur)

    for ev in events:
        ts = [x for x in ev["times"] if x]
        if len(ts) > 1:
            span = (ts[-1] - ts[0]).total_seconds()
            ev["dt"] = span / (len(ts) - 1) if span > 0 else 0.0
            ev["elapsed"] = span
        else:
            ev["elapsed"] = 0.0
    return events


def main():
    args = [a for a in sys.argv[1:]]
    out_csv = None
    if "--csv" in args:
        i = args.index("--csv")
        out_csv = args[i + 1]
        del args[i:i + 2]

    if not args:
        print(__doc__)
        return 1

    # Windows shells do not expand wildcards, so do it here. Sorted so the
    # output runs oldest to newest when filenames carry a date stamp.
    paths = []
    for a in args:
        hits = sorted(glob.glob(a))
        if hits:
            paths.extend(hits)
        else:
            print("no match: %s" % a)
    if not paths:
        return 1

    rows = []
    for path in paths:
        for ev in parse(path):
            verdict, note = classify(ev)
            rows.append({
                "file": path.split("/")[-1],
                "time": ev["t0"].strftime("%m-%d %H:%M:%S") if ev["t0"] else "?",
                "elapsed_s": round(ev["elapsed"], 1),
                "samples": len(ev["halls"]),
                "hall_start": ev["halls"][0] if ev["halls"] else "",
                "hall_min": min(ev["halls"]) if ev["halls"] else "",
                "hall_end": ev["halls"][-1] if ev["halls"] else "",
                "servo_cmd": ev["servo"] if ev["servo"] is not None else "",
                "verdict": verdict,
                "note": note,
            })

    if not rows:
        print("No RETRACT attempts found. Check the log path.")
        return 1

    w = [max(len(str(r[k])) for r in rows + [{k: k}]) for k in rows[0]]
    keys = list(rows[0].keys())
    print("  ".join(k.ljust(w[i]) for i, k in enumerate(keys)))
    print("  ".join("-" * w[i] for i in range(len(keys))))
    for r in rows:
        print("  ".join(str(r[k]).ljust(w[i]) for i, k in enumerate(keys)))

    tally = {}
    for r in rows:
        tally[r["verdict"]] = tally.get(r["verdict"], 0) + 1
    print("\n%d retract attempts: %s" % (
        len(rows), ", ".join("%s %d" % (k, v) for k, v in sorted(tally.items()))))

    stalls = [r for r in rows if r["verdict"] == "STALLED"]
    if stalls:
        print("\n%d stalled attempts - these are the damaging ones." % len(stalls))
        print("Earliest: %s  Latest: %s" % (stalls[0]["time"], stalls[-1]["time"]))

    if out_csv:
        with open(out_csv, "w", newline="") as f:
            wr = csv.DictWriter(f, fieldnames=keys)
            wr.writeheader()
            wr.writerows(rows)
        print("\nwrote %s" % out_csv)
    return 0


if __name__ == "__main__":
    sys.exit(main())
