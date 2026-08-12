"""
upload_cached_samples.py
------------------------
Upload cached sample CSVs from the Pi directly to Firebase, bypassing the
MAVLink encoder path entirely. This restores full precision (decimal pressure,
correct init_do) for records that were corrupted by encoder scaling issues.

Cache files are written by cache_sample_csv() in main_rc8_uart_parm.py to:
    cache/sample_YYYYMMDD_HHMMSS_mmm.csv

Columns: time, DO, temp, pressure, init_DO, init_pressure, batt_v, lat, lon

Pond ID is resolved from lat/lon by point-in-polygon against the farm GeoJSON
files, with a small buffer (same approach as cleanup_unknown_ponds.py).

Usage:
    python upload_cached_samples.py --dry-run
    python upload_cached_samples.py
    python upload_cached_samples.py --cache-dir /path/to/cache
    python upload_cached_samples.py --type winch --drone-id SPLASHY_2
    python upload_cached_samples.py --file cache/sample_20260811_024640_123.csv

Requires (same directory unless overridden):
    fb_key.json
    farm_features.json
    basler_features.json

Dependencies:
    pip install firebase-admin shapely pytz
"""

import os
import csv
import glob
import json
import argparse
from datetime import datetime

import pytz
import firebase_admin
from firebase_admin import credentials, db
from shapely.geometry import shape, Point

UTC_TZ = pytz.utc
POLYGON_BUFFER_M = 8
METERS_PER_DEGREE = 111_000


# ---- Firebase login ------------------------------------------------

def login(key_path='fb_key.json'):
    with open(key_path) as f:
        key = json.load(f)
    cred = credentials.Certificate(key)
    firebase_admin.initialize_app(cred, {
        'databaseURL': 'https://haucs-monitoring-default-rtdb.firebaseio.com/'
    })


# ---- Pond resolution from lat/lon ----------------------------------

def load_pond_polygons(*geojson_paths):
    buffer_deg = POLYGON_BUFFER_M / METERS_PER_DEGREE
    polygons = []
    for path in geojson_paths:
        try:
            with open(path) as f:
                geo = json.load(f)
        except FileNotFoundError:
            print("  warning: %s not found, skipping" % path)
            continue
        for feature in geo.get('features', []):
            pond_id = str(feature['properties']['number'])
            poly = shape(feature['geometry']).buffer(buffer_deg)
            polygons.append((pond_id, poly))
    return polygons


def match_pond(lat, lon, polygons):
    point = Point(float(lon), float(lat))
    for pond_id, poly in polygons:
        if poly.contains(point):
            return pond_id
    return None


# ---- Cache file parsing --------------------------------------------

def parse_cache_file(path):
    """
    Read one cache CSV and return a dict with the arrays and scalars,
    or None if the file has no usable rows.
    """
    with open(path, newline='') as f:
        rows = list(csv.DictReader(f))

    if not rows:
        return None

    def col_float(name):
        out = []
        for r in rows:
            v = r.get(name, '')
            if v not in ('', None):
                try:
                    out.append(float(v))
                except ValueError:
                    pass
        return out

    do_vals    = col_float('DO')
    temp_vals  = col_float('temp')
    press_vals = col_float('pressure')
    init_do    = col_float('init_DO')
    init_press = col_float('init_pressure')
    battv      = col_float('batt_v')
    lat_vals   = col_float('lat')
    lon_vals   = col_float('lon')

    if not do_vals:
        return None

    return {
        'do': do_vals,
        'temp': temp_vals,
        'pressure': press_vals,
        'init_do': init_do[0] if init_do else 1,
        'init_pressure': init_press[0] if init_press else (press_vals[0] if press_vals else 1013),
        'sensor_battv': battv[0] if battv else None,
        'lat': lat_vals[0] if lat_vals else None,
        'lng': lon_vals[0] if lon_vals else None,
    }


def timestamp_from_filename(path, local_tz):
    """
    cache/sample_YYYYMMDD_HHMMSS_mmm.csv -> Firebase UTC key.
    The filename timestamp is Pi local time; convert to UTC.
    """
    base = os.path.basename(path)
    stem = base.replace('sample_', '').replace('.csv', '')
    parts = stem.split('_')
    if len(parts) < 2:
        return None
    date_part, time_part = parts[0], parts[1]
    try:
        naive = datetime.strptime(date_part + time_part, '%Y%m%d%H%M%S')
    except ValueError:
        return None
    local = local_tz.localize(naive)
    return local.astimezone(UTC_TZ).strftime('%Y%m%d_%H:%M:%S')


# ---- Main -----------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Upload cached Pi sample CSVs directly to Firebase")
    parser.add_argument('--cache-dir', default='cache',
                        help="Directory holding sample_*.csv (default: cache)")
    parser.add_argument('--file', default=None,
                        help="Upload a single specific CSV instead of the whole dir")
    parser.add_argument('--type', default='winch',
                        help="Value for the record 'type' field (default: winch)")
    parser.add_argument('--drone-id', default=None,
                        help="Optional drone_id to store on each record")
    parser.add_argument('--tz', default='America/Chicago',
                        help="Timezone of the Pi filenames (default: America/Chicago)")
    parser.add_argument('--lh-geojson', default='farm_features.json')
    parser.add_argument('--basler-geojson', default='basler_features.json')
    parser.add_argument('--pond', default=None,
                        help="Force a pond ID instead of resolving from lat/lon")
    parser.add_argument('--overwrite', action='store_true',
                        help="Overwrite if a record already exists at that key")
    parser.add_argument('--dry-run', action='store_true',
                        help="Show what would be uploaded without writing")
    args = parser.parse_args()

    local_tz = pytz.timezone(args.tz)

    if args.file:
        files = [args.file]
    else:
        files = sorted(glob.glob(os.path.join(args.cache_dir, 'sample_*.csv')))

    if not files:
        print("No cache files found.")
        return

    print("Found %d cache file(s)\n" % len(files))

    login()
    polygons = load_pond_polygons(args.lh_geojson, args.basler_geojson)
    print("Loaded %d pond polygon(s)\n" % len(polygons))

    n_ok = n_skip = 0

    for path in files:
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

        # Resolve pond
        if args.pond:
            pond_id = args.pond
        elif parsed['lat'] is not None and parsed['lng'] is not None:
            pond_id = match_pond(parsed['lat'], parsed['lng'], polygons)
        else:
            pond_id = None

        if pond_id is None:
            print("%s: no pond match (lat=%s lng=%s), skipping. Use --pond to force."
                  % (name, parsed['lat'], parsed['lng']))
            n_skip += 1
            continue

        pond_key = 'pond_%s' % pond_id

        # Build the Firebase record in the same shape the web app expects
        record = {
            'do': parsed['do'],
            'temp': parsed['temp'],
            'pressure': parsed['pressure'],
            'init_do': parsed['init_do'],
            'init_pressure': parsed['init_pressure'],
            'pid': pond_id,
            'type': args.type,
        }
        if parsed['lat'] is not None:
            record['lat'] = parsed['lat']
        if parsed['lng'] is not None:
            record['lng'] = parsed['lng']
        if parsed['sensor_battv'] is not None:
            record['sensor_battv'] = parsed['sensor_battv']
        if args.drone_id:
            record['drone_id'] = args.drone_id

        # Depth range as a sanity readout
        if parsed['pressure']:
            span = max(parsed['pressure']) - min(parsed['pressure'])
            depth_in = span * 10.197 / 25.4
        else:
            depth_in = 0

        print("%s" % name)
        print("    -> LH_Farm/%s/%s" % (pond_key, ts_key))
        print("       n=%d  init_do=%s  init_p=%s  depth_span=%.1f in (%.2f ft)"
              % (len(parsed['do']), parsed['init_do'],
                 parsed['init_pressure'], depth_in, depth_in / 12))

        if args.dry_run:
            n_ok += 1
            continue

        ref = db.reference('LH_Farm/%s/%s' % (pond_key, ts_key))
        if ref.get() and not args.overwrite:
            print("       record already exists, skipping (use --overwrite)")
            n_skip += 1
            continue

        ref.set(record)
        n_ok += 1

    print("")
    if args.dry_run:
        print("DRY RUN. %d would upload, %d skipped. Re-run without --dry-run to apply."
              % (n_ok, n_skip))
    else:
        print("Done. %d uploaded, %d skipped." % (n_ok, n_skip))


if __name__ == '__main__':
    main()
