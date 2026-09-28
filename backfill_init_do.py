"""
backfill_init_do.py
-------------------
Repair historical winch records whose init_do was uploaded as a raw ADC count.

Background
----------
init_do is meant to be on the same scale as the do[] samples, so that
do / init_do is saturation as a fraction. The truck path in firebase_worker.py
has hardcoded init_do = 1 since the beginning. The winch path in
mavproxy_haucs/__init__.py wrote the sensor's raw ADC count (~4121) while still
sending do[] as already normalised ratios of 0.2 to 0.6, so any consumer
dividing by it scales the whole cast down by a factor of thousands. Cast
20260927_18:32:14 rendered as a flat 0.0008 mg/L column because of this.

The producer was fixed 092826 (writes init_do: 1, keeps the count as
init_do_raw). This script applies the same shape to records written before
that fix:

    init_do      -> 1
    init_do_raw  -> the original value (added, not overwritten)

Selection guard
---------------
Two-sided on purpose. A genuine air calibration in raw counts paired with do[]
also in raw counts is self-consistent and must NOT be rewritten; only the
mismatch is repaired:

    init_do > 100  AND  max(do[] where > 0) < 10

This mirrors converter.py do_cal_mismatched() and static/js/do_cal.js. If you
change the rule, change it in all three.

Safety
------
Dry run by default. Nothing is written unless you pass --apply.
A record that already has init_do_raw is skipped, so re-running is harmless.

Usage
-----
    python backfill_init_do.py                      # dry run, all ponds
    python backfill_init_do.py --pond ton29         # dry run, one pond
    python backfill_init_do.py --days 30            # dry run, last 30 days
    python backfill_init_do.py --apply --backup before.json   # write, keep originals
    python backfill_init_do.py --restore before.json          # preview an undo
    python backfill_init_do.py --restore before.json --apply  # undo it

--backup writes a NEW file; it is an output, not something you supply. It holds
the complete original records the script is about to change, so --restore can
put them back exactly as they were.

Requires fb_key.json in the same directory as this script (or --key).
"""

import os
import sys
import json
import argparse
from datetime import datetime, timedelta, timezone

import firebase_admin
from firebase_admin import credentials, db


DB_URL = 'https://haucs-monitoring-default-rtdb.firebaseio.com/'
FARM_ROOT = 'LH_Farm'

# Must match converter.py and static/js/do_cal.js
DO_CAL_RAW_COUNT_MIN = 100
DO_CAL_RATIO_MAX = 10

# Keys under FARM_ROOT that are not ponds
SKIP_KEYS = {'overview', 'drone', 'gps', 'email', 'comments', 'equipment',
             'bathymetry', 'recent', 'watchdog'}


# -- Firebase ----------------------------------------------------------

def login(key_path=None):
    if key_path is None:
        key_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                'fb_key.json')
    with open(key_path, 'r') as f:
        key = json.load(f)
    cred = credentials.Certificate(key)
    firebase_admin.initialize_app(cred, {'databaseURL': DB_URL})


def list_pond_keys():
    farm = db.reference(FARM_ROOT).get(shallow=True)
    if not farm:
        return []
    return sorted(k for k in farm.keys()
                  if k.startswith('pond_') and k not in SKIP_KEYS)


# -- Guard -------------------------------------------------------------

def needs_backfill(entry):
    """True when init_do is a raw count sitting against ratio-scale samples."""
    if not isinstance(entry, dict):
        return False
    if 'init_do_raw' in entry:
        return False               # already backfilled
    try:
        init_do = float(entry.get('init_do'))
    except (TypeError, ValueError):
        return False
    samples = []
    for v in (entry.get('do') or []):
        try:
            v = float(v)
        except (TypeError, ValueError):
            continue
        if v > 0:
            samples.append(v)
    if not samples:
        return False
    return init_do > DO_CAL_RAW_COUNT_MIN and max(samples) < DO_CAL_RATIO_MAX


# -- Main --------------------------------------------------------------

def scan(pond_keys, start_key):
    """Returns a list of (pond_key, timestamp, entry) needing repair."""
    hits = []
    for pond_key in pond_keys:
        ref = db.reference(f'{FARM_ROOT}/{pond_key}').order_by_key()
        if start_key:
            ref = ref.start_at(start_key)
        records = ref.get()
        if not records:
            continue

        n_win = n_hit = 0
        for ts, entry in records.items():
            if not isinstance(entry, dict):
                continue
            if entry.get('type') != 'winch':
                continue
            n_win += 1
            if needs_backfill(entry):
                n_hit += 1
                hits.append((pond_key, ts, entry))

        if n_win:
            print(f"  {pond_key}: {n_win} winch record(s), {n_hit} need repair")
    return hits


def restore(path, apply_changes):
    """Put back the records exactly as --backup captured them."""
    with open(path, 'r') as f:
        payload = json.load(f)

    if not payload:
        print(f"{path} is empty. Nothing to restore.")
        return

    print(f"Restoring {len(payload)} record(s) from {path}\n")

    planned = []
    for row in payload:
        pond_key, ts, entry = row['pond'], row['ts'], row['entry']
        original = entry.get('init_do')
        live = db.reference(f'{FARM_ROOT}/{pond_key}/{ts}').get()
        if live is None:
            print(f"  SKIP {pond_key}/{ts}: record no longer exists")
            continue
        now = live.get('init_do')
        if now == original and 'init_do_raw' not in live:
            print(f"  SKIP {pond_key}/{ts}: already at its original value")
            continue
        print(f"  {pond_key}/{ts}: init_do {now} -> {original}, drop init_do_raw")
        planned.append((pond_key, ts, original))

    if not planned:
        print("\nNothing to change.")
        return

    if not apply_changes:
        print(f"\nDRY RUN. {len(planned)} record(s) would be restored. "
              "Re-run with --apply to write.")
        return

    done = 0
    for pond_key, ts, original in planned:
        # None deletes the key in the Realtime Database, so the record ends up
        # exactly as it was rather than keeping a leftover init_do_raw.
        db.reference(f'{FARM_ROOT}/{pond_key}/{ts}').update({
            'init_do': original,
            'init_do_raw': None,
        })
        done += 1
    print(f"\nRestored {done} record(s).")


def main():
    ap = argparse.ArgumentParser(
        description="Backfill init_do on historical winch records")
    ap.add_argument('--apply', action='store_true',
                    help="Actually write. Without this the script only reports.")
    ap.add_argument('--pond', default=None,
                    help="Limit to one pond id, e.g. ton29 or BP2 "
                         "(the pond_ prefix is optional)")
    ap.add_argument('--days', type=float, default=None,
                    help="Only look at records from the last N days "
                         "(default: the whole history)")
    ap.add_argument('--backup', default=None,
                    help="Write the pre-change records to this JSON file")
    ap.add_argument('--restore', default=None,
                    help="Undo a previous run using the JSON written by "
                         "--backup. Preview only unless --apply is also given.")
    ap.add_argument('--key', default=None,
                    help="Path to fb_key.json (default: next to this script)")
    args = ap.parse_args()

    login(args.key)

    if args.restore:
        restore(args.restore, args.apply)
        return

    if args.pond:
        pid = args.pond.strip().strip("'\"")
        if pid.startswith('pond_'):
            pid = pid[5:]
        pond_keys = [f'pond_{pid}']
    else:
        pond_keys = list_pond_keys()

    start_key = None
    if args.days:
        cutoff = datetime.now(timezone.utc) - timedelta(days=args.days)
        start_key = cutoff.strftime('%Y%m%d_%H:%M:%S')

    window = f"last {args.days} day(s)" if args.days else "full history"
    print(f"Scanning {len(pond_keys)} pond(s), {window}\n")

    hits = scan(pond_keys, start_key)

    if not hits:
        print("\nNothing to repair. Every winch record already has init_do on "
              "the same scale as its do[] samples.")
        return

    print(f"\n{len(hits)} record(s) need repair:\n")
    for pond_key, ts, entry in hits:
        samples = [float(v) for v in (entry.get('do') or []) if float(v) > 0]
        print(f"  {pond_key}/{ts}  init_do={entry['init_do']}  "
              f"do[] max={max(samples):.3f}  n={len(samples)}")

    if args.backup:
        payload = [{'pond': p, 'ts': t, 'entry': e} for p, t, e in hits]
        with open(args.backup, 'w') as f:
            json.dump(payload, f, indent=2)
        print(f"\nBackup of {len(payload)} record(s) written to {args.backup}")

    if not args.apply:
        print("\nDRY RUN. Nothing was written.")
        print("Repair with:   --apply --backup before.json")
        print("Undo it with:  --restore before.json --apply")
        return

    # Write only the two fields, so nothing else on the record can be lost.
    written = 0
    failed = 0
    for pond_key, ts, entry in hits:
        try:
            db.reference(f'{FARM_ROOT}/{pond_key}/{ts}').update({
                'init_do': 1,
                'init_do_raw': entry['init_do'],
            })
            written += 1
        except Exception as e:
            failed += 1
            print(f"  ERROR {pond_key}/{ts}: {e}")

    print(f"\nDone. Repaired {written} record(s)"
          + (f", {failed} failed" if failed else "") + ".")
    if written:
        print("Re-run without --apply to confirm the scan now comes back clean.")


if __name__ == '__main__':
    try:
        main()
    except FileNotFoundError as e:
        print(f"Could not open the Firebase key: {e}")
        sys.exit(1)
