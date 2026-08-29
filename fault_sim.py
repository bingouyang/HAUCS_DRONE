#!/usr/bin/env python3
"""
fault_sim.py - drive the HAUCS status code and its detail messages without
needing a real winch fault, and model the servo wear each cast causes.

Two channels, as discussed:

  HAUCS (NAMED_VALUE_FLOAT)  -> a number. Bind it in Mission Planner as a HUD
                                user item or a Quick tab cell. It PERSISTS,
                                because MP holds the last value it received.
                                This is the only channel that stays on screen.

  STATUSTEXT                 -> the detail, in the Messages tab. Sent at INFO
                                so it never displaces a real flight warning on
                                MP's high priority line.

Code table. Under 8 is normal progress, 8 and above is a fault - one rule for
the operator to remember.

     0  idle
     1  latch pulse, release commanded
     2  payload descending
     3  sampling at depth
     4  retract running
     5  cast complete, transmitting
     8  LATCH DID NOT OPEN         (pulses exhausted, Hall never confirmed)
     9  RETRACT TIMEOUT            (line not home)
    10  SENSOR FAULT               (Hall read failing)
    11  NO SAMPLES                 (cast produced nothing)

WEAR MODEL
----------
Gear-flank wear rises steeply with load, so it is modelled as

    wear += (torque / rated_torque) ** WEAR_EXP  *  seconds

with WEAR_EXP = 3, the usual order for surface fatigue. Forces, from the
measured mechanism:

    latch pull      0.5 lb = 2.22 N   -> 3.15 oz-in =  0.71% of the 444 oz-in rating
    retract         15 g   = 0.147 N  -> 0.21 oz-in =  0.05%
    STALL           servo pushes to its rating         100%

Relative wear per event:

    normal latch pull, 0.25 s     8.9e-08
    retract, 16 s                 1.7e-09
    ONE 3-SECOND STALL            3.0e+00     <- 34 million normal pulls

That ratio is the whole failure story. Casts do nothing; stalls do everything.
FAIL_UNITS is calibrated so roughly 45 stall events consume a servo, which is
about what three months of frequent drop failures produced.

Wear accumulates across runs in fault_sim_wear.json so a long soak can be
built up over several sessions. --reset clears it.

Usage:
    python3 fault_sim.py --cast            # one clean cast, codes 1-5-0
    python3 fault_sim.py --fault 8         # jump straight to a fault code
    python3 fault_sim.py --all             # every code in turn, 4s apart
    python3 fault_sim.py --cast --fault 8  # a cast that fails at the latch
    python3 fault_sim.py --loop            # clean cast, then a failing one,
                                           # repeating, for leaving on screen
    python3 fault_sim.py --random -n 20    # 20 casts, faults drawn at random
    python3 fault_sim.py --random -n 200 --fast --no-send    # wear soak, no MAVLink
    python3 fault_sim.py --reset           # zero the accumulated wear

Run it on the Pi with the main script stopped - they cannot share
/dev/serial0. Or point it at a UDP link with --port for a desk test.
"""

import argparse
import json
import random
import sys
import time

try:
    from pymavlink import mavutil
except ImportError:
    print("pymavlink not installed:  pip3 install pymavlink")
    sys.exit(1)

FC_CONN_STR = "/dev/serial0"
FC_BAUD = 115200
PREFIX = "HAUCS: "
MAX_LEN = 49
FIELD = b"HAUCS"          # NAMED_VALUE_FLOAT names are capped at 10 chars
FIELD_WEAR = b"HAUCS_W"   # servo wear, percent of modelled life

# ---- wear model -----------------------------------------------------------
LB_N          = 0.453592 * 9.81   # pound-force to newtons
LATCH_FORCE_N = 0.5 * LB_N        # measured: 0.5 lb to open the latch
RETRACT_N     = 0.015 * 9.81      # ~15 g effective during controlled retract
LEVER_M       = 0.010             # effective radius the force acts at
RATED_OZIN    = 444.0             # Triple4 at 7.4 V
WEAR_EXP      = 3.0               # surface fatigue is steeply superlinear
FAIL_UNITS    = 180.0             # ~45 stalls of 4 s consumes a servo
WEAR_FILE     = "fault_sim_wear.json"


def torque_frac(force_n):
    """Fraction of the servo's rated torque this force demands."""
    return (force_n * LEVER_M * 141.6) / RATED_OZIN


FRAC_LATCH   = torque_frac(LATCH_FORCE_N)
FRAC_RETRACT = torque_frac(RETRACT_N)
FRAC_STALL   = 1.0                # a stalled servo pushes to its rating


def wear(frac, seconds):
    return (frac ** WEAR_EXP) * seconds

# code -> (label, detail sent as STATUSTEXT)
CODES = {
    0:  ("idle",            "idle"),
    1:  ("latch pulse",     "release: latch pulse 1 of 2"),
    2:  ("descending",      "payload descending"),
    3:  ("sampling",        "sampling at depth"),
    4:  ("retracting",      "retract running"),
    5:  ("cast complete",   "cast complete, sending data"),
    8:  ("LATCH FAIL",      "LATCH DID NOT OPEN after 2 pulses"),
    9:  ("RETRACT TIMEOUT", "RETRACT TIMEOUT, line not home"),
    10: ("SENSOR FAULT",    "hall read failing, position unknown"),
    11: ("NO SAMPLES",      "cast complete but 0 samples"),
}

CLEAN_CAST = [(1, 1.0), (2, 3.0), (3, 4.0), (4, 4.0), (5, 2.0), (0, 2.0)]


def send(m, code, detail=None, quiet=False):
    """Emit the code on the persistent channel and the detail on the tab."""
    text = detail if detail is not None else CODES.get(code, ("", ""))[1]
    try:
        m.mav.named_value_float_send(
            int(time.time() * 1000) & 0xFFFFFFFF, FIELD, float(code))
    except Exception as e:
        print("  named_value_float failed: %s" % e)
    if text:
        try:
            m.mav.statustext_send(
                mavutil.mavlink.MAV_SEVERITY_INFO,
                (PREFIX + text)[:MAX_LEN].encode("ascii", "replace"))
        except Exception as e:
            print("  statustext failed: %s" % e)
    if not quiet:
        label = CODES.get(code, ("?", ""))[0]
        print("  HAUCS=%-3d %-16s %s" % (code, label, text))


def hold(m, code, seconds, detail=None):
    """Hold a code on screen. Named floats are one-shot and MP keeps the last
    value, so a dropped packet would leave the display stale - resend at 1 Hz
    so it is self-healing. The detail goes out once, not repeatedly."""
    send(m, code, detail)
    t_end = time.time() + seconds
    while time.time() < t_end:
        time.sleep(min(1.0, max(0.0, t_end - time.time())))
        if time.time() < t_end:
            send(m, code, detail="", quiet=True)


def load_wear():
    try:
        with open(WEAR_FILE) as f:
            d = json.load(f)
        return float(d.get("units", 0.0)), int(d.get("casts", 0)), int(d.get("stalls", 0))
    except Exception:
        return 0.0, 0, 0


def save_wear(units, casts, stalls):
    try:
        with open(WEAR_FILE, "w") as f:
            json.dump({"units": units, "casts": casts, "stalls": stalls,
                       "pct": 100.0 * units / FAIL_UNITS}, f)
    except Exception as e:
        print("  could not save wear: %s" % e)


# Fault draw. Weights are rough field frequencies: most casts are clean, the
# latch sticking is the common failure, sensor and sample faults are rarer.
FAULT_WEIGHTS = [
    (None, 70),   # clean cast
    (8,    18),   # latch did not open  -> the damaging one
    (9,     6),   # retract timeout
    (10,    3),   # sensor fault
    (11,    3),   # no samples
]


def draw_fault():
    total = sum(w for _, w in FAULT_WEIGHTS)
    r = random.uniform(0, total)
    acc = 0
    for code, w in FAULT_WEIGHTS:
        acc += w
        if r <= acc:
            return code
    return None


def simulate_cast(m, cfg, fault, send_mav=True):
    """Run one cast, return (wear_units, stall_seconds, description).

    Only the latch path can stall: the servo drives at RELEASE_PWR against a
    mechanism that will not move, at its rated torque, until the code gives
    up. Everything else in a cast is far below the wear threshold.
    """
    w = 0.0
    stall_s = 0.0

    def step(code, secs, detail=None):
        if send_mav:
            hold(m, code, secs if not cfg["fast"] else 0.05, detail)
        elif not cfg["quiet"]:
            print("  HAUCS=%-3d %s" % (code, CODES.get(code, ("?", ""))[1]))

    # --- latch ---
    step(1, 1.0)
    if fault == 8:
        # pulses exhausted. Under the OLD scheme the servo held torque for the
        # whole timeout; under the pulsed scheme it is TRIES x PULSE_SEC.
        stall_s = cfg["stall_sec"]
        w += wear(FRAC_STALL, stall_s)
        step(8, 4.0)
        return w, stall_s, "latch did not open (%.2fs stalled)" % stall_s
    # latch opened: brief pull at the measured 0.5 lb
    w += wear(FRAC_LATCH, cfg["latch_pulse"])

    # --- descend, sample ---
    step(2, 3.0)
    step(3, 4.0)

    # --- retract ---
    step(4, 4.0)
    if fault == 9:
        w += wear(FRAC_STALL, cfg["retract_stall"])
        stall_s += cfg["retract_stall"]
        step(9, 4.0)
        return w, stall_s, "retract timeout (%.1fs stalled)" % cfg["retract_stall"]
    w += wear(FRAC_RETRACT, 16.0)

    if fault in (10, 11):
        step(fault, 4.0)
        return w, stall_s, CODES[fault][0]

    step(5, 2.0)
    step(0, 1.0)
    return w, stall_s, "clean"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=FC_CONN_STR)
    ap.add_argument("--baud", type=int, default=FC_BAUD)
    ap.add_argument("--sysid", type=int, default=1)
    ap.add_argument("--compid", type=int, default=191)
    ap.add_argument("--cast", action="store_true",
                    help="run one clean cast, codes 1 through 5 then idle")
    ap.add_argument("--fault", type=int, metavar="CODE",
                    help="raise this fault code (8, 9, 10, 11)")
    ap.add_argument("--all", action="store_true",
                    help="walk every code in the table, 4s each")
    ap.add_argument("--loop", action="store_true",
                    help="clean cast then a failing cast, repeating")
    ap.add_argument("--hold", type=float, default=8.0,
                    help="seconds to hold a fault code (default 8)")
    ap.add_argument("--random", action="store_true",
                    help="draw a fault at random each cast (or none)")
    ap.add_argument("-n", "--casts", type=int, default=1,
                    help="how many casts to run with --random")
    ap.add_argument("--stall-sec", type=float, default=0.5,
                    help="seconds the servo stalls when the latch fails. "
                         "0.5 = the pulsed scheme (2 x 0.25s). Use 3.0 to "
                         "model the OLD scheme that ran out RELEASE_SEC.")
    ap.add_argument("--retract-stall", type=float, default=2.0,
                    help="seconds stalled on a retract timeout")
    ap.add_argument("--fast", action="store_true",
                    help="skip the real-time holds; wear is unaffected")
    ap.add_argument("--no-send", action="store_true",
                    help="model only, no MAVLink link needed")
    ap.add_argument("--reset", action="store_true",
                    help="zero the accumulated wear and exit")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    if args.reset:
        save_wear(0.0, 0, 0)
        print("wear reset to 0")
        return 0

    if not (args.cast or args.fault is not None or args.all or args.loop
            or args.random):
        ap.print_help()
        print("\nPick at least one of --cast, --fault, --all, --loop, --random")
        return 1

    m = None
    if args.no_send:
        print("model only, no MAVLink link")
    else:
        print("opening %s as sys=%d comp=%d"
              % (args.port, args.sysid, args.compid))
    try:
        if args.no_send:
            m = None
        elif "://" in args.port or ":" in args.port:
            m = mavutil.mavlink_connection(args.port,
                                           source_system=args.sysid,
                                           source_component=args.compid)
        else:
            m = mavutil.mavlink_connection(args.port, baud=args.baud,
                                           source_system=args.sysid,
                                           source_component=args.compid)
    except Exception as e:
        print("FAILED to open the link: %s" % e)
        print("  - is the main script still running? it holds /dev/serial0")
        return 1

    print()
    print("In Mission Planner: right-click the HUD -> user items, or the Quick")
    print("tab, and bind the field named HAUCS. The number stays on screen.")
    print("Details appear in the Messages tab.")
    print()

    cfg = {"stall_sec": args.stall_sec, "retract_stall": args.retract_stall,
           "latch_pulse": 0.25, "fast": args.fast, "quiet": args.quiet}

    try:
        if args.random:
            units, casts, stalls = load_wear()
            print("starting wear: %.2f units (%.1f%% of modelled life), "
                  "%d casts, %d stalls" % (units, 100*units/FAIL_UNITS,
                                           casts, stalls))
            print("stall model: %.2fs per latch failure\n" % args.stall_sec)
            failed = False
            for i in range(1, args.casts + 1):
                fault = draw_fault()
                w, st, why = simulate_cast(m, cfg, fault,
                                           send_mav=not args.no_send)
                units += w
                casts += 1
                if st > 0:
                    stalls += 1
                pct = 100.0 * units / FAIL_UNITS
                print("cast %-4d %-34s wear +%.4f  total %.2f (%.1f%%)"
                      % (i, why, w, units, pct))
                if m is not None:
                    send(m, 0 if fault is None else fault,
                         detail="", quiet=True)
                    try:
                        m.mav.named_value_float_send(
                            int(time.time() * 1000) & 0xFFFFFFFF,
                            FIELD_WEAR, float(min(pct, 999.0)))
                    except Exception:
                        pass
                if pct >= 100.0 and not failed:
                    failed = True
                    print("\n*** SERVO FAILED at cast %d, %d stall events ***"
                          % (casts, stalls))
                    if m is not None:
                        send(m, 8, detail="SERVO WORN OUT, %d stalls" % stalls)
                    break
            save_wear(units, casts, stalls)
            print("\n%d casts, %d stall events, wear %.2f of %.0f (%.1f%%)"
                  % (casts, stalls, units, FAIL_UNITS,
                     100.0 * units / FAIL_UNITS))
            if not failed and units > 0:
                remaining = (FAIL_UNITS - units)
                per_cast = units / max(casts, 1)
                if per_cast > 0:
                    print("at this rate, ~%d more casts to failure"
                          % int(remaining / per_cast))
            return 0

        if args.all:
            print("walking every code, %.0fs each:" % args.hold)
            for code in sorted(CODES):
                hold(m, code, args.hold)
            send(m, 0)
            return 0

        rounds = 0
        while True:
            rounds += 1
            if args.cast or args.loop:
                print("clean cast:")
                for code, secs in CLEAN_CAST:
                    hold(m, code, secs)

            if args.fault is not None or args.loop:
                code = args.fault if args.fault is not None else 8
                if code not in CODES:
                    print("unknown code %d; known: %s"
                          % (code, sorted(CODES)))
                    return 1
                print("fault:")
                # a fault interrupts a cast, so lead in with the state it
                # would have been in when it went wrong
                if args.loop:
                    hold(m, 1, 1.5)
                hold(m, code, args.hold)
                print("  (code stays until the next cast clears it)")

            if not args.loop:
                break
            print("--- round %d done, repeating ---\n" % rounds)
            time.sleep(3.0)

    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        if m is not None:
            send(m, 0, detail="sim stopped")
            print("\nsent HAUCS=0 on exit")
    return 0


if __name__ == "__main__":
    sys.exit(main())
