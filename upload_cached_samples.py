"""
upload_cached_samples.py
------------------------
Upload the Pi's cached sample CSVs straight to Firebase, bypassing the MAVLink
encoder entirely. Use this to recover casts whose encoded values were clipped
or quantized in transit.

The Pi has no pond map of its own -- pond ID resolution lives on the GCS inside
MAVProxy (db_helper.get_pond_id). This script reuses that same infrastructure:
nearest sampling point from sampling_points.csv, 100 m threshold, identical
maths. No GeoJSON and no shapely, so it runs anywhere the MAVProxy files are.

Cache CSV columns (written by cache_sample_csv in main_rc8_uart_parm.py):
    time, DO, temp, pressure, init_DO, init_pressure, batt_v, lat, lon

Record written matches the GCS winch record in mavproxy_haucs/__init__.py.

Usage:
    python upload_cached_samples.py --dry-run
    python upload_cached_samples.py --file 'cache/sample_20260810*.csv'
    python upload_cached_samples.py --file cache/sample_20260810*.csv
    python upload_cached_samples.py --sampling-csv ../MAVProxy/sampling_points.csv

Setting the pond explicitly (needed for files with no reliable lat/lon, such
as casts recovered from a log). Three ways, in priority order:

    per file   --file 'recovered/a.csv=BP2' --file 'recovered/b.csv=BP3'
    per glob   --file 'recovered/*.csv=BP2'
    for all    --pond BP2 --file 'recovered/*.csv'

Anything left unspecified falls back to the nearest sampling point. When every
file has a pond, sampling_points.csv is not read at all.

Needs alongside it (or pass paths):
    fb_key.json
    sampling_points.csv      (from the GCS / MAVProxy directory)

Dependencies: firebase-admin, pandas, numpy, pytz
"""

import os
import re
import csv
import glob
import json
import argparse
from datetime import datetime

import numpy as np
import pandas as pd
import pytz
import firebase_admin
from firebase_admin import credentials, db

UTC_TZ = pytz.utc
MATCH_THRESHOLD_M = 100      # same threshold as db_helper.get_pond_id
METERS_PER_DEGREE = 111_000

# The sensor reports init_do in raw counts (~4000) while the do samples are
# already normalised ratios (~0.7-0.9). The GCS truck path hardcodes init_do=1
# for exactly this reason. See firebase_worker.update_firebase:
#     upload_data['init_do'] = 1  # hardcoded to handle legacy website
# The winch path in __init__.py does NOT do this, which is why winch DO reads
# as ~0.02 percent saturation on the website. We normalise here to match the
# truck path, and keep the sensor's raw value under init_do_raw for reference.
NORMALISE_INIT_DO = True


# ---- Firebase ------------------------------------------------------

def login(key_path='fb_key.json'):
    with open(key_path) as f:
        key = json.load(f)
    cred = credentials.Certificate(key)
    firebase_admin.initialize_app(cred, {
        'databaseURL': 'https://haucs-monitoring-default-rtdb.firebaseio.com/'
    })


# ---- Pond lookup, mirroring db_helper.get_pond_id -------------------

def load_sampling_points(path='sampling_points.csv'):
    """
    Returns (pond_ids, pond_gps) exactly as db_helper.get_pond_id builds them.
    The CSV column order is pond, lon, lat -- note lon before lat.
    """
    df = pd.read_csv(path)
    pond_ids = df.pop('pond').astype(str).to_numpy()
    pond_gps = df.to_numpy()          # [[lon, lat], ...]
    return pond_ids, pond_gps


def get_pond_id(lat, lng, pond_ids, pond_gps):
    """Nearest sampling point within MATCH_THRESHOLD_M, else None."""
    point = np.array([float(lng), float(lat)])
    tiled = np.tile(point, (pond_gps.shape[0], 1))
    distances = np.linalg.norm(pond_gps - tiled, axis=1)
    min_dist_m = distances.min() * METERS_PER_DEGREE
    if min_dist_m < MATCH_THRESHOLD_M:
        return str(pond_ids[np.argmin(distances)]), min_dist_m
    return None, min_dist_m


# ---- Cache parsing --------------------------------------------------

def parse_cache_file(path):
    with open(path, newline='') as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return None

    def col(name):
        out = []
        for r in rows:
            v = r.get(name, '')
            if v not in ('', None):
                try:
                    out.append(float(v))
                except ValueError:
                    pass
        return out

    do_vals = col('DO')
    if not do_vals:
        return None

    press = col('pressure')
    init_do_raw = col('init_DO')
    init_p = col('init_pressure')
    battv = col('batt_v')
    lat = col('lat')
    lon = col('lon')

    return {
        'do': do_vals,
        'temp': col('temp'),
        'pressure': press,
        'init_do_raw': init_do_raw[0] if init_do_raw else None,
        'init_pressure': init_p[0] if init_p else (press[0] if press else 1013.0),
        'batt_v': battv[0] if battv else None,
        'lat': lat[0] if lat else None,
        'lng': lon[0] if lon else None,
        'n': len(rows),
    }


def timestamp_from_filename(path, local_tz):
    """sample_YYYYMMDD_HHMMSS_mmm.csv -> Firebase UTC key."""
    stem = os.path.basename(path).replace('sample_', '').replace('.csv', '')
    parts = stem.split('_')
    if len(parts) < 2:
        return None
    try:
        naive = datetime.strptime(parts[0] + parts[1], '%Y%m%d%H%M%S')
    except ValueError:
        return None
    return local_tz.localize(naive).astimezone(UTC_TZ).strftime('%Y%m%d_%H:%M:%S')


# ---- File selection -------------------------------------------------

def split_pond_suffix(pattern):
    """
    Split an optional '=POND' suffix off a --file argument.

    Recovered log CSVs and hand-made files often need their pond stated
    explicitly, and a batch can span several ponds, so the pond travels with
    the path rather than being one global flag:

        --file 'recovered/a.csv=BP2' --file 'recovered/b.csv=BP3'
        --file 'recovered/*.csv=BP2'

    '=' is used rather than ':' so Windows drive letters (D: drive paths) are safe.
    Only the last '=' is considered, and only when what follows contains no
    path separator, so a filename that legitimately contains '=' still works.
    """
    if '=' not in pattern:
        return pattern, None
    head, tail = pattern.rsplit('=', 1)
    tail = tail.strip()
    if not head or not tail:
        return pattern, None
    # A pond id is a short bare token: letters, digits, underscore. Anything
    # containing a path separator or a dot is part of the filename, so a file
    # legitimately named 'weird=name.csv' is left alone.
    if not re.match(r'^[A-Za-z0-9_]+$', tail):
        return pattern, None
    return head, tail


def collect_files(args):
    """
    Resolve --file into a sorted, de-duplicated list of (path, pond) pairs,
    where pond is a per-file override or None.

    Handles a plain path, a glob, repeated flags, a directory, and the
    shell-pre-expanded form where extra paths arrive as positionals.
    """
    patterns = list(args.file or [])
    patterns += list(getattr(args, 'extra', None) or [])

    if not patterns:
        return [(f, None) for f in
                sorted(glob.glob(os.path.join(args.cache_dir, 'sample_*.csv')))]

    out = []
    for raw in patterns:
        pat, pond = split_pond_suffix(raw)
        if any(ch in pat for ch in '*?['):
            hits = sorted(glob.glob(pat))
            if not hits:
                print("  warning: no match for %s" % pat)
            out.extend((h, pond) for h in hits)
        elif os.path.isdir(pat):
            out.extend((h, pond) for h in
                       sorted(glob.glob(os.path.join(pat, 'sample_*.csv'))))
        elif os.path.exists(pat):
            out.append((pat, pond))
        else:
            print("  warning: not found %s" % pat)

    # De-duplicate by absolute path. A later explicit pond wins over an
    # earlier one so a glob can set a default and a specific file refine it.
    seen = {}
    for f, pond in out:
        real = os.path.abspath(f)
        if real not in seen or pond is not None:
            seen[real] = (f, pond)
    return sorted(seen.values(), key=lambda x: x[0])


# ---- Main -----------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(
        description="Upload cached Pi sample CSVs to Firebase, bypassing the encoder")
    ap.add_argument('--cache-dir', default='cache',
                    help="Directory of sample_*.csv (default: cache)")
    ap.add_argument('--file', action='append', default=None,
                    metavar='PATH[=POND]',
                    help="Specific CSV or glob, e.g. 'cache/sample_20260810*.csv'. "
                         "Repeatable. Append '=POND' to set the pond for that "
                         "file or glob, e.g. 'recovered/a.csv=BP2'. Overrides "
                         "--pond and the lat/lon lookup for those files.")
    ap.add_argument('--sampling-csv', default='sampling_points.csv',
                    help="MAVProxy sampling_points.csv used for pond lookup")
    ap.add_argument('--pond', default=None,
                    help="Force a pond ID for every file that has no '=POND' "
                         "suffix, instead of resolving from lat/lon")
    ap.add_argument('--drone-id', default='SPLASHY_UNK',
                    help="Value for the record sid field (default: SPLASHY_UNK)")
    ap.add_argument('--type', default='winch',
                    help="Value for the record type field (default: winch)")
    ap.add_argument('--tz', default='America/Chicago',
                    help="Timezone of the Pi filenames (default: America/Chicago)")
    ap.add_argument('--raw-init-do', action='store_true',
                    help="Write the sensor's raw init_do instead of normalising to 1. "
                         "Only use this if the sensor firmware is changed so that "
                         "do samples and init_do share the same units.")
    ap.add_argument('--overwrite', action='store_true',
                    help="Replace an existing record at the same key")
    ap.add_argument('--dry-run', action='store_true',
                    help="Show what would be uploaded without writing")
    ap.add_argument('extra', nargs='*',
                    help="Extra CSV paths, usually from an unquoted shell glob")
    args = ap.parse_args()

    local_tz = pytz.timezone(args.tz)
    files = collect_files(args)
    if not files:
        print("No cache files matched.")
        return

    print("Found %d cache file(s)" % len(files))

    # The sampling point table is only needed for files whose pond is not
    # already known, so a fully specified batch can run without it.
    needs_lookup = any(pond is None for _, pond in files) and not args.pond

    pond_ids = pond_gps = None
    if needs_lookup:
        if not os.path.exists(args.sampling_csv):
            print("\nERROR: %s not found." % args.sampling_csv)
            print("Pond lookup lives on the GCS. Either copy sampling_points.csv")
            print("from the MAVProxy directory, point at it with --sampling-csv,")
            print("or bypass the lookup entirely with --pond BP2.")
            return
        pond_ids, pond_gps = load_sampling_points(args.sampling_csv)
        print("Loaded %d sampling point(s) from %s" % (len(pond_ids), args.sampling_csv))
    print()

    if not args.dry_run:
        login()

    n_ok = n_skip = 0

    for path, pond_override in files:
        name = os.path.basename(path)
        parsed = parse_cache_file(path)
        if parsed is None:
            print("%s: no usable rows, skipping" % name)
            n_skip += 1
            continue

        ts_key = timestamp_from_filename(path, local_tz)
        if ts_key is None:
            print("%s: cannot parse timestamp from filename, skipping" % name)
            n_skip += 1
            continue

        # Resolve pond: per-file override, then --pond, then lat/lon lookup
        if pond_override:
            pond_id, dist, how = pond_override, None, 'from --file'
        elif args.pond:
            pond_id, dist, how = args.pond, None, 'from --pond'
        elif pond_ids is not None and parsed['lat'] is not None and parsed['lng'] is not None:
            pond_id, dist = get_pond_id(parsed['lat'], parsed['lng'], pond_ids, pond_gps)
            how = 'from lat/lon %.0f m' % dist if dist is not None else 'from lat/lon'
        else:
            pond_id, dist, how = None, None, 'unresolved'

        if pond_id is None:
            msg = "no pond within %d m" % MATCH_THRESHOLD_M
            if dist is not None:
                msg += " (nearest %.0f m)" % dist
            print("%s: %s, skipping. Use --pond to force." % (name, msg))
            n_skip += 1
            continue

        # init_do normalisation -- see the note at the top of this file
        raw_init = parsed['init_do_raw']
        init_do = raw_init if args.raw_init_do else 1

        record = {
            'do': parsed['do'],
            'temp': parsed['temp'],
            'pressure': parsed['pressure'],
            'init_do': init_do,
            'init_pressure': parsed['init_pressure'],
            'pid': str(pond_id),
            'sid': args.drone_id,
            'type': args.type,
            'seq': 0,
        }
        if raw_init is not None:
            record['init_do_raw'] = raw_init
        if parsed['lat'] is not None:
            record['lat'] = parsed['lat']
        if parsed['lng'] is not None:
            record['lng'] = parsed['lng']
        if parsed['batt_v'] is not None:
            record['batt_v'] = parsed['batt_v']

        press = parsed['pressure']
        span = (max(press) - min(press)) if press else 0.0
        depth_in = span * 10.197 / 25.4
        do_lo, do_hi = min(parsed['do']), max(parsed['do'])

        print("%s" % name)
        print("    -> LH_Farm/pond_%s/%s   (%s)" % (pond_id, ts_key, how))
        print("       n=%d  depth span %.1f in (%.2f ft)  DO %.2f..%.2f"
              % (parsed['n'], depth_in, depth_in / 12, do_lo, do_hi))
        print("       init_do=%s (raw %s)  init_pressure=%.2f"
              % (init_do, raw_init, parsed['init_pressure']))

        if args.dry_run:
            n_ok += 1
            continue

        ref = db.reference('LH_Farm/pond_%s/%s' % (pond_id, ts_key))
        if ref.get() and not args.overwrite:
            print("       record exists, skipping (use --overwrite)")
            n_skip += 1
            continue

        ref.set(record)
        n_ok += 1

    print()
    if args.dry_run:
        print("DRY RUN. %d would upload, %d skipped." % (n_ok, n_skip))
    else:
        print("Done. %d uploaded, %d skipped." % (n_ok, n_skip))


if __name__ == '__main__':
    main()
