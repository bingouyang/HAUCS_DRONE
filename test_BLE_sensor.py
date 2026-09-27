#!/usr/bin/env python3
"""
test_BLE_sensor.py  --  092726

Talk to the BLE sensor directly, without running the flight script. Built
because nothing in the package could read or clear the sensor's own state:
every path to it went through main_rc8_uart_*.py.

The sensor keeps its sample buffer on its own power. Restarting the Pi or the
main script does NOT clear it -- startup only sends reads plus `cal ps`,
`set max_sample` and `set sample_hz`. `sample reset` is the only thing that
empties it, and auto_sensing() on the sensor re-fires only when the count is
back to 0. So a buffer left full after a failed fetch means the NEXT cast
collects nothing, however many times you restart.

Usage:
    python3 test_BLE_sensor.py                      # status only, changes nothing
    python3 test_BLE_sensor.py --dump cast.csv      # pull the buffer to CSV
    python3 test_BLE_sensor.py --dump cast.csv --reset   # pull, then clear
    python3 test_BLE_sensor.py --reset --force      # clear WITHOUT saving
    python3 test_BLE_sensor.py --stop               # clear the sampling flag

--reset refuses to run on a non-empty buffer unless the same command also
dumped it, or --force is given. The buffer is often the only copy of a cast
whose fetch failed in flight.
"""

import argparse
import csv
import os
import sys
import time

from bt_helper import BluetoothReader, QMutex

FETCH_TIMEOUT = 5.0          # BLE inactivity timeout during a dump
CONNECT_TRIES = 3


def connect(ble):
    for attempt in range(1, CONNECT_TRIES + 1):
        print("connecting to sensor, attempt %d of %d ..." % (attempt, CONNECT_TRIES))
        try:
            if ble.connect():
                print("  connected to %s" % ble.sdata.get("name", "?"))
                return True
        except Exception as e:
            print("  connect raised %s: %s" % (e.__class__.__name__, e))
        time.sleep(1.0)
    return False


def read_int(v):
    """bt_helper returns command replies as ['key', 'value'] lists, or "" on
    failure. Never raise on a malformed reply."""
    try:
        return int(float(v[1]))
    except Exception:
        return None


def show_status(ble):
    print("\n--- sensor status ---")
    size = ble.get_sample_size()
    flag = ble.get_sampl_flag()
    n = read_int(size)
    print("  samples buffered : %s%s"
          % (n if n is not None else "unreadable (%r)" % (size,),
             "   <== auto_sensing will NOT re-arm until this is 0" if n else ""))
    print("  sampling flag    : %s" % (flag[1] if flag and len(flag) > 1 else "?"))
    try:
        ble.init_sensor_status()
        print("  sample rate      : %s Hz" % ble.sdata.get("sample_hz"))
        print("  battery          : %s V (%s)"
              % (ble.sdata.get("battv"), ble.sdata.get("batt_status")))
        print("  init DO          : %s" % ble.sdata.get("init_do"))
        print("  init pressure    : %s hPa" % ble.sdata.get("init_pressure"))
    except Exception as e:
        print("  status reads failed: %s: %s" % (e.__class__.__name__, e))
    print("  timeouts recorded: %d" % ble.transmission_timeouts)
    return n


def dump(ble, path, expected):
    """Pull the whole buffer. `sample print` always dumps from the start, so
    this is safe to repeat -- and a partial pull can be retried."""
    print("\n--- dumping buffer to %s ---" % os.path.abspath(path))
    t0 = time.time()
    ok = bool(ble.get_sample_data(FETCH_TIMEOUT))
    do = list(ble.sdata.get("do_vals") or [])
    tp = list(ble.sdata.get("temp_vals") or [])
    pr = list(ble.sdata.get("pressure_vals") or [])
    n = min(len(do), len(tp), len(pr))
    if n != max(len(do), len(tp), len(pr)):
        print("  ragged lists do=%d temp=%d press=%d, truncating to %d"
              % (len(do), len(tp), len(pr), n))
    do, tp, pr = do[:n], tp[:n], pr[:n]

    # A corrupted record is appended by bt_helper as 0,0,0 rather than skipped,
    # so it looks like a real sample. Absolute pressure is never 0, which makes
    # them easy to find -- flag rather than drop, so the row count still lines
    # up with the sensor's own index.
    bad = [i for i in range(n) if pr[i] == 0 and tp[i] == 0 and do[i] == 0]

    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["index", "DO", "temp_C", "pressure_hPa", "suspect"])
        for i in range(n):
            w.writerow([i, do[i], tp[i], pr[i], 1 if i in bad else 0])

    print("  transfer %s in %.1fs" % ("complete" if ok else "CUT OFF", time.time() - t0))
    print("  rows written     : %d%s"
          % (n, "" if expected in (None, n) else "  (sensor reported %s)" % expected))
    if bad:
        print("  suspect rows     : %d all-zero record(s) at %s%s"
              % (len(bad), bad[:10], " ..." if len(bad) > 10 else ""))
        print("                     these are corrupted BLE records, not real data")
    if pr:
        print("  pressure range   : %.2f to %.2f hPa"
              % (min(p for p in pr if p) if any(pr) else 0.0, max(pr)))
        print("                     a flat range near the surface value means the")
        print("                     payload never went down")
    complete = ok and (expected is None or n >= expected)
    if not complete:
        print("  NOT a complete pull -- run again before resetting")
    return complete, n


def main():
    p = argparse.ArgumentParser(description="Read or clear the BLE sensor's own state")
    p.add_argument("--dump", metavar="CSV", help="pull the sample buffer to this file")
    p.add_argument("--reset", action="store_true",
                   help="clear the buffer so the next cast can auto-arm")
    p.add_argument("--stop", action="store_true",
                   help="clear the sampling flag (sensor stops sampling)")
    p.add_argument("--force", action="store_true",
                   help="allow --reset to discard an unsaved buffer")
    args = p.parse_args()

    ble = BluetoothReader(QMutex())
    if not connect(ble):
        print("\nERROR: could not connect to the sensor.")
        print("  If the main script is running it holds the link -- stop it first.")
        return 1

    rc = 0
    try:
        n = show_status(ble)

        dumped = False
        if args.dump:
            dumped, _ = dump(ble, args.dump, n)

        if args.stop:
            print("\n--- clearing the sampling flag ---")
            ble.set_sampl_flag(0)
            time.sleep(0.2)
            flag = ble.get_sampl_flag()
            print("  flag now: %s" % (flag[1] if flag and len(flag) > 1 else "?"))

        if args.reset:
            print("\n--- reset ---")
            if n and not dumped and not args.force:
                print("  REFUSED: %d samples on the sensor and nothing saved." % n)
                print("  This buffer may be the only copy of a cast whose fetch")
                print("  failed in flight. Re-run with --dump FILE, or --force to")
                print("  discard it deliberately.")
                rc = 2
            elif n and args.dump and not dumped and not args.force:
                print("  REFUSED: the dump did not complete, so the CSV is short.")
                print("  Run the --dump again, or add --force to discard the rest.")
                rc = 2
            else:
                ble.set_sample_reset()
                time.sleep(0.3)
                after = read_int(ble.get_sample_size())
                print("  samples buffered now: %s" % after)
                if after:
                    print("  WARNING: still non-empty, the reset did not take")
                    rc = 3
                else:
                    print("  sensor is armed for the next cast")

        if not (args.dump or args.reset or args.stop):
            print("\n(status only -- nothing was changed. --dump / --reset / --stop "
                  "to act.)")
    finally:
        try:
            conn = getattr(ble, "uart_connection", None)
            if conn and getattr(conn, "connected", False):
                conn.disconnect()
                print("\ndisconnected")
        except Exception:
            pass
    return rc


if __name__ == "__main__":
    sys.exit(main())
