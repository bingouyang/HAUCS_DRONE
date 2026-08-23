#!/usr/bin/env python3
"""
winch_audit.py - full deploy/retract cycle audit for HAUCS Pi logs.

Supersedes retract_audit.py. Adds the release (descent) half of the cycle and
speed trending, so servo torque loss shows up as a declining counts/sec trend
before it becomes an outright failure.

Usage:
    python winch_audit.py logs/cc_*.log
    python winch_audit.py --csv audit.csv logs/cc_*.log
    python winch_audit.py --trend logs/cc_*.log      # speed trend only

RETRACT verdicts (Hall is logged every loop, so these are well determined):
    OK               reached settle
    SETTLED-NO-FLAG  ended below the settle line but no "Fully retractd";
                     RETRACTED was latched from a prior cycle. Servo was
                     neutraled - harmless, but the timeout warning is spurious.
    NO-MOTION        Hall never moved. Servo not driving: open signal line, or
                     a fault pulling the supply down. No stall, no damage.
    STALLED          Hall fell, then went flat ABOVE the settle line. Drove
                     against something and held. This is the damaging case.
    IMPAIRED         moved, never settled, never went flat
    SENSOR-SUSPECT   readings outside the plausible window

RELEASE is only bracketed by two log lines with no Hall samples between them,
so descent speed is derived from the pre-release Hall reading and the first
reading of the following retract. Treat it as an estimate, not a measurement.
"""

import sys
import re
import csv
import glob
from datetime import datetime

# --- tuning -----------------------------------------------------------------
MOVE_TH = 200      # total travel below this = "never moved"
FLAT_TH = 30       # per-sample change below this counts as no progress
FLAT_MIN = 3.0     # seconds of no progress at the end to call it a stall
HALL_LO = 0        # plausible reading window
HALL_HI = 13000

# Hall reading at or below which retract_adaptive() takes the settle branch and
# calls neutral(). Flat readings below this mean finished-and-stopped, NOT a
# stall. Must match HALL_TARGET + RETRACT_SETTLE for the build that made the log.
SETTLE_HALL = 2500 + 50

TS = re.compile(r"^(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2})[,.]?(\d*)")

RE_RET_START = re.compile(r"RETRACT for ([\d.]+)s")
RE_HALL = re.compile(r"currnt hall_raw val:\s*(-?\d+)")
RE_RET_DONE = re.compile(r"Fully retractd")
RE_RET_TIMEOUT = re.compile(r"Retract timeout")
RE_SERVO = re.compile(r"servo\.value\s+(-?[\d.]+)")

RE_REL_START = re.compile(r"Release, winch servo open:\s*(-?[\d.]+)")
RE_REL_DONE = re.compile(r"Release complete")
RE_REL_ABORT = re.compile(r"Aborting release|ABORT release")
RE_PRE_OK = re.compile(r"Hall pre-check OK:\s*(-?\d+)")
RE_PRE_OUT = re.compile(r"Hall reads (-?\d+) before release")
RE_RECOVERED = re.compile(r"Recovery retract OK: hall now (-?\d+)")
RE_STALL = re.compile(r"Retract stalled at hall (-?\d+)")


def stamp(line):
    m = TS.match(line)
    if not m:
        return None
    base = datetime.strptime(m.group(1), "%Y-%m-%d %H:%M:%S")
    if m.group(2):
        base = base.replace(microsecond=int(m.group(2).ljust(6, "0")[:6]))
    return base


def classify_retract(ev):
    halls = ev["halls"]
    if not halls:
        return "NO-DATA", "no Hall readings logged"
    lo, hi = min(halls), max(halls)
    if lo < HALL_LO or hi > HALL_HI:
        return "SENSOR-SUSPECT", "Hall out of range (%d..%d)" % (lo, hi)

    travel = halls[0] - min(halls)
    if ev["done"]:
        return "OK", "settled, travel %d counts" % travel
    if ev["stall_logged"]:
        return "STALLED", "stall cutout fired at hall %d" % ev["stall_logged"]
    if travel < MOVE_TH:
        return "NO-MOTION", "Hall flat at ~%d, servo not driving" % halls[0]
    if halls[-1] < SETTLE_HALL:
        return "SETTLED-NO-FLAG", ("ended at %d, below settle line %d; servo "
                                   "neutraled, timeout warning spurious"
                                   % (halls[-1], SETTLE_HALL))

    flat_n = 0
    for i in range(len(halls) - 1, 0, -1):
        if abs(halls[i - 1] - halls[i]) < FLAT_TH:
            flat_n += 1
        else:
            break
    flat_s = flat_n * ev["dt"] if ev["dt"] else 0.0
    if flat_s >= FLAT_MIN:
        return "STALLED", "fell to %d then flat %.1fs" % (min(halls), flat_s)
    return "IMPAIRED", "travel %d, ended %d, no settle" % (travel, halls[-1])


def parse(path):
    """Return (retracts, releases) in log order."""
    rets, rels = [], []
    cur = None            # open retract
    rel = None            # open release
    pre_hall = None       # last pre-release Hall reading

    for line in open(path, errors="ignore"):
        t = stamp(line)

        m = RE_PRE_OK.search(line) or RE_PRE_OUT.search(line) or RE_RECOVERED.search(line)
        if m:
            pre_hall = int(m.group(1))

        m = RE_REL_START.search(line)
        if m:
            rel = {"file": path, "t0": t, "cmd": float(m.group(1)),
                   "hall_before": pre_hall, "done": False, "abort": False,
                   "t1": None, "hall_after": None}
            rels.append(rel)
            continue

        if rel is not None and rel["t1"] is None:
            if RE_REL_DONE.search(line):
                rel["done"] = True
                rel["t1"] = t
            elif RE_REL_ABORT.search(line):
                rel["abort"] = True
                rel["t1"] = t

        m = RE_RET_START.search(line)
        if m:
            cur = {"file": path, "t0": t, "dur": float(m.group(1)),
                   "halls": [], "times": [], "done": False, "timeout": False,
                   "servo": None, "dt": 0.0, "closed": False,
                   "stall_logged": None}
            rets.append(cur)
            continue

        if cur is None:
            continue

        m = RE_SERVO.search(line)
        if m and cur["servo"] is None:
            cur["servo"] = float(m.group(1))

        m = RE_HALL.search(line)
        if m and not cur["closed"]:
            h = int(m.group(1))
            cur["halls"].append(h)
            cur["times"].append(t)
            # first reading after a release closes that release's travel
            if rel is not None and rel["hall_after"] is None and len(cur["halls"]) == 1:
                rel["hall_after"] = h
            continue

        m = RE_STALL.search(line)
        if m:
            cur["stall_logged"] = int(m.group(1))
            cur["closed"] = True
        elif RE_RET_DONE.search(line):
            cur["done"] = True
            cur["closed"] = True     # stop collecting; avoids swallowing the release
        elif RE_RET_TIMEOUT.search(line):
            cur["timeout"] = True
            cur["closed"] = True

    for ev in rets:
        ts = [x for x in ev["times"] if x]
        if len(ts) > 1:
            span = (ts[-1] - ts[0]).total_seconds()
            ev["dt"] = span / (len(ts) - 1) if span > 0 else 0.0
            ev["elapsed"] = span
        else:
            ev["elapsed"] = 0.0
    return rets, rels


def table(rows, cols):
    if not rows:
        print("  (none)")
        return
    w = []
    for k, _ in cols:
        w.append(max([len(str(r.get(k, ""))) for r in rows] + [len(k)]))
    print("  ".join(k.ljust(w[i]) for i, (k, _) in enumerate(cols)))
    print("  ".join("-" * w[i] for i in range(len(cols))))
    for r in rows:
        print("  ".join(str(r.get(k, "")).ljust(w[i]) for i, (k, _) in enumerate(cols)))


def main():
    args = list(sys.argv[1:])
    out_csv = None
    trend_only = False
    if "--csv" in args:
        i = args.index("--csv")
        out_csv = args[i + 1]
        del args[i:i + 2]
    if "--trend" in args:
        args.remove("--trend")
        trend_only = True
    if not args:
        print(__doc__)
        return 1

    paths = []
    for a in args:
        hits = sorted(glob.glob(a))
        if hits:
            paths.extend(hits)
        else:
            print("no match: %s" % a)
    if not paths:
        return 1

    rrows, drows = [], []
    for path in paths:
        rets, rels = parse(path)
        for ev in rets:
            v, note = classify_retract(ev)
            tr = (ev["halls"][0] - min(ev["halls"])) if ev["halls"] else 0
            spd = round(tr / ev["elapsed"]) if ev["elapsed"] > 1 and tr > MOVE_TH else ""
            rrows.append({
                "file": path.split("/")[-1].split("\\")[-1],
                "time": ev["t0"].strftime("%m-%d %H:%M:%S") if ev["t0"] else "?",
                "cmd": ev["servo"] if ev["servo"] is not None else "",
                "travel": tr,
                "sec": round(ev["elapsed"], 1),
                "cts_s": spd,
                "h_start": ev["halls"][0] if ev["halls"] else "",
                "h_end": ev["halls"][-1] if ev["halls"] else "",
                "verdict": v,
                "note": note,
            })
        for ev in rels:
            sec = ""
            if ev["t0"] and ev["t1"]:
                sec = round((ev["t1"] - ev["t0"]).total_seconds(), 1)
            tr = ""
            spd = ""
            if ev["hall_before"] is not None and ev["hall_after"] is not None:
                tr = ev["hall_after"] - ev["hall_before"]
                if isinstance(sec, float) and sec > 1 and tr > MOVE_TH:
                    spd = round(tr / sec)
            drows.append({
                "file": path.split("/")[-1].split("\\")[-1],
                "time": ev["t0"].strftime("%m-%d %H:%M:%S") if ev["t0"] else "?",
                "cmd": ev["cmd"],
                "travel": tr,
                "sec": sec,
                "cts_s": spd,
                "h_before": ev["hall_before"] if ev["hall_before"] is not None else "",
                "h_after": ev["hall_after"] if ev["hall_after"] is not None else "",
                "verdict": ("ABORT" if ev["abort"]
                            else "NO-PAYOUT" if (isinstance(tr, int) and tr < MOVE_TH)
                            else "OK" if ev["done"] else "INCOMPLETE"),
            })

    if not trend_only:
        print("=== RETRACT (ascent) ===")
        table(rrows, [("file", 0), ("time", 0), ("cmd", 0), ("travel", 0),
                      ("sec", 0), ("cts_s", 0), ("h_start", 0), ("h_end", 0),
                      ("verdict", 0), ("note", 0)])
        print("\n=== RELEASE (descent) ===")
        print("travel and cts_s are estimates: Hall is not sampled during release,")
        print("so travel spans the pre-release reading to the first retract reading.")
        table(drows, [("file", 0), ("time", 0), ("cmd", 0), ("travel", 0),
                      ("sec", 0), ("cts_s", 0), ("h_before", 0), ("h_after", 0),
                      ("verdict", 0)])

    tally = {}
    for r in rrows:
        tally[r["verdict"]] = tally.get(r["verdict"], 0) + 1
    print("\n%d retracts: %s" % (
        len(rrows), ", ".join("%s %d" % kv for kv in sorted(tally.items()))))
    stalls = [r for r in rrows if r["verdict"] == "STALLED"]
    print("%d releases, %d stalled retracts" % (len(drows), len(stalls)))
    if stalls:
        print("STALLED are the damaging ones - earliest %s, latest %s"
              % (stalls[0]["time"], stalls[-1]["time"]))

    # --- speed trend, grouped by command level ---
    print("\n=== SPEED TREND (retract, grouped by command) ===")
    print("Torque loss shows as a falling trend WITHIN a command group.")
    print("Within-session decline that recovers next session is battery sag or")
    print("heat, not damage - compare first-of-session values across days.")
    groups = {}
    for r in rrows:
        # Only healthy retracts belong in a torque trend; a stalled or impaired
        # run reports a low speed for reasons that have nothing to do with torque.
        if r["verdict"] == "OK" and r["cts_s"] != "" and r["cmd"] != "":
            groups.setdefault(round(float(r["cmd"]), 3), []).append(r)
    for cmd in sorted(groups):
        g = groups[cmd]
        sp = [float(x["cts_s"]) for x in g]
        print("\n  cmd %-8s n=%d  first %.0f  last %.0f  min %.0f  max %.0f"
              % (cmd, len(g), sp[0], sp[-1], min(sp), max(sp)))
        for x in g:
            bar = "#" * max(1, int(float(x["cts_s"]) / max(sp) * 40))
            print("    %s  %6s  %s" % (x["time"], x["cts_s"], bar))

    if out_csv:
        with open(out_csv, "w", newline="") as f:
            wr = csv.writer(f)
            wr.writerow(["phase"] + list(rrows[0].keys()) if rrows else ["phase"])
            for r in rrows:
                wr.writerow(["retract"] + list(r.values()))
            for r in drows:
                wr.writerow(["release"] + list(r.values()))
        print("\nwrote %s" % out_csv)
    return 0


if __name__ == "__main__":
    sys.exit(main())
