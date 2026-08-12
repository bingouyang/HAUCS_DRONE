"""
log_to_cache_csv.py
-------------------
Recover winch casts from the Pi's log files and write them as cache-format
CSVs that upload_cached_samples.py can consume unchanged.

Why this exists
---------------
When a BLE transfer dies partway through, main_rc8_uart_parm.py never reaches
cache_sample_csv(), so the cast is never written to disk and is lost. But every
sample was already written to the log by bt_helper.send_receive_command before
the crash. This script reconstructs those casts from the log text.

It also filters out the casts where the winch never actually released. Those
show up as a flat pressure trace at the retracted depth (about 7 in) and are
not real profiles, so they are skipped by default. Use --min-descent 0 to keep
everything.

Output matches the Pi cache format exactly:
    time, DO, temp, pressure, init_DO, init_pressure, batt_v, lat, lon
written to  <outdir>/sample_YYYYMMDD_HHMMSS_000.csv
so the normal upload path works:

    python log_to_cache_csv.py cc_260810_2024.log --outdir recovered
    python upload_cached_samples.py --dry-run --file 'recovered/sample_*.csv'
    python upload_cached_samples.py --file 'recovered/sample_*.csv'

Usage:
    python log_to_cache_csv.py <logfile> [<logfile> ...] [options]

Options:
    --outdir DIR        where to write CSVs (default: recovered)
    --min-descent IN    minimum descent in inches to count as a real cast
                        (default 12; set 0 to keep every cast)
    --init-pressure P   override the surface pressure reference
    --list              report what was found and write nothing
"""

import os
import re
import csv
import argparse

# bt_helper logs each sample as:
#   ble incoming msg: ['ts', '<ms>', 'd', '<do>', 't', '<temp>', 'p', '<press>']
TS_RE = re.compile(
    r"msg: \['ts', '(\d+)', 'd', '([-\d.]+)', 't', '([-\d.]+)', 'p', '([-\d.]+)'\]")

# The command dict is echoed on every line, so 'dstart' appears everywhere.
# Match only the actual response to avoid splitting on every sample.
DSTART_RE = re.compile(r"msg: \['dstart'")
DSIZE_RE = re.compile(r"msg: \['dsize', '(\d+)'\]")

INIT_DO_RE = re.compile(r"msg: \['init_do', '([\d.]+)'\]")
INIT_P_RE = re.compile(r"msg: \['init_p', '([\d.]+)'\]")
CAL_P_RE = re.compile(r"msg: \['init p', ' ?([\d.]+)'\]")
BATT_RE = re.compile(r"msg: \['v', '([\d.]+)'")

RELEASE_RE = re.compile(r"EVENT: winch release.*lat=([-\d.]+) lon=([-\d.]+)")
GCS_RE = re.compile(r"EVENT: requesting data to send to gcs, lat=([-\d.]+) lon=([-\d.]+)")

STAMP_RE = re.compile(r"^(\d{4})-(\d\d)-(\d\d) (\d\d):(\d\d):(\d\d)")

HPA_TO_INCH = 10.197 / 25.4


def parse_log(path):
    """
    Walk one log file and return a list of cast dicts. Calibration values and
    coordinates are carried forward as they appear, so each cast gets whatever
    was most recently in effect.
    """
    casts = []
    cur = None

    init_do = None
    init_p_get = None      # from 'get init_p'
    init_p_cal = None      # from 'cal ps', taken after the surface calibration
    batt = None
    rel_lat = rel_lon = None
    gcs_lat = gcs_lon = None
    dsize = None

    def close(status):
        nonlocal cur
        if cur is not None and cur['rows']:
            cur['status'] = status
            casts.append(cur)
        cur = None

    for line in open(path, errors='ignore'):
        m = STAMP_RE.match(line)
        stamp = m.groups() if m else None

        m = INIT_DO_RE.search(line)
        if m:
            init_do = float(m.group(1))
        m = INIT_P_RE.search(line)
        if m:
            init_p_get = float(m.group(1))
        m = CAL_P_RE.search(line)
        if m:
            init_p_cal = float(m.group(1))
        m = BATT_RE.search(line)
        if m:
            batt = float(m.group(1))
        m = DSIZE_RE.search(line)
        if m:
            dsize = int(m.group(1))

        m = RELEASE_RE.search(line)
        if m:
            rel_lat, rel_lon = float(m.group(1)), float(m.group(2))
        m = GCS_RE.search(line)
        if m:
            gcs_lat, gcs_lon = float(m.group(1)), float(m.group(2))

        if DSTART_RE.search(line):
            close('incomplete')
            cur = {
                'stamp': stamp,
                'rows': [],
                'init_do': init_do,
                # The cache uses ble.sdata['init_pressure']. Prefer the value
                # from the surface calibration when one was performed, since
                # that is the reference the depths are relative to.
                'init_pressure': init_p_cal if init_p_cal is not None else init_p_get,
                'batt_v': batt,
                # lat/lon are locked at the release edge in mav_thread, which is
                # where the sensor actually went, so prefer those.
                'lat': rel_lat if rel_lat is not None else gcs_lat,
                'lon': rel_lon if rel_lon is not None else gcs_lon,
                'dsize': dsize,
                'source': os.path.basename(path),
            }
            continue

        m = TS_RE.search(line)
        if m and cur is not None:
            cur['rows'].append((
                int(m.group(1)) / 1000.0,   # sensor ts, ms -> s
                float(m.group(2)),          # DO
                float(m.group(3)),          # temp
                float(m.group(4)),          # pressure
            ))
            continue

        if 'fetch failed' in line:
            close('FETCH FAILED')
        elif 'finish sampling' in line:
            close('ok')

    close('incomplete')
    return casts


def summarize(c, init_p_override=None):
    p = [r[3] for r in c['rows']]
    d = [r[1] for r in c['rows']]
    ref = init_p_override if init_p_override is not None else c['init_pressure']
    descent_in = (max(p) - min(p)) * HPA_TO_INCH
    depth_in = ((max(p) - ref) * HPA_TO_INCH) if ref else float('nan')
    return {
        'n': len(c['rows']),
        'descent_in': descent_in,
        'depth_in': depth_in,
        'do_lo': min(d), 'do_hi': max(d),
        'p_lo': min(p), 'p_hi': max(p),
    }


def filename_for(c):
    y, mo, d, hh, mm, ss = c['stamp']
    return "sample_%s%s%s_%s%s%s_000.csv" % (y, mo, d, hh, mm, ss)


def write_cast(c, outdir, init_p_override=None):
    os.makedirs(outdir, exist_ok=True)
    path = os.path.join(outdir, filename_for(c))
    ref = init_p_override if init_p_override is not None else c['init_pressure']

    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['time', 'DO', 'temp', 'pressure',
                    'init_DO', 'init_pressure', 'batt_v', 'lat', 'lon'])
        for t, do, temp, press in c['rows']:
            w.writerow([t, do, temp, press,
                        c['init_do'] if c['init_do'] is not None else '',
                        ref if ref is not None else '',
                        c['batt_v'] if c['batt_v'] is not None else '',
                        c['lat'] if c['lat'] is not None else '',
                        c['lon'] if c['lon'] is not None else ''])
    return path


def main():
    ap = argparse.ArgumentParser(
        description="Recover winch casts from Pi logs into cache-format CSVs")
    ap.add_argument('logs', nargs='+', help="log file(s) to parse")
    ap.add_argument('--outdir', default='recovered',
                    help="output directory (default: recovered)")
    ap.add_argument('--min-descent', type=float, default=12.0,
                    help="minimum descent in inches to treat as a real cast "
                         "(default 12; use 0 to keep all)")
    ap.add_argument('--init-pressure', type=float, default=None,
                    help="override the surface pressure reference in hPa")
    ap.add_argument('--list', action='store_true',
                    help="report only, write nothing")
    args = ap.parse_args()

    all_casts = []
    for path in args.logs:
        if not os.path.exists(path):
            print("  warning: not found %s" % path)
            continue
        found = parse_log(path)
        all_casts.extend(found)
        print("%s: %d cast(s) with samples" % (os.path.basename(path), len(found)))
    print()

    if not all_casts:
        print("No casts found.")
        return

    print("  %-9s %-13s %5s %8s %8s  %-17s %s" %
          ("time", "fetch", "n", "descent", "depth", "DO range", "keep"))
    kept = []
    for c in all_casts:
        s = summarize(c, args.init_pressure)
        keep = s['descent_in'] >= args.min_descent
        if keep:
            kept.append(c)
        print("  %-9s %-13s %5d %6.1fin %6.1fin  %.2f-%.2f        %s" %
              (":".join(c['stamp'][3:6]), c['status'], s['n'],
               s['descent_in'], s['depth_in'], s['do_lo'], s['do_hi'],
               "yes" if keep else "no (no release)"))

    print()
    print("%d of %d cast(s) pass the %.0f in descent filter."
          % (len(kept), len(all_casts), args.min_descent))

    if not kept:
        print("Nothing to write. Lower --min-descent to keep flat casts.")
        return

    if args.list:
        print("List mode, nothing written.")
        return

    print()
    for c in kept:
        p = write_cast(c, args.outdir, args.init_pressure)
        s = summarize(c, args.init_pressure)
        print("  wrote %s  (%d rows, %.1f in, lat=%s lon=%s, init_do=%s)"
              % (p, s['n'], s['depth_in'], c['lat'], c['lon'], c['init_do']))

    print()
    print("Next:")
    print("  python upload_cached_samples.py --dry-run --file '%s/sample_*.csv'"
          % args.outdir)
    print("  python upload_cached_samples.py --file '%s/sample_*.csv'"
          % args.outdir)


if __name__ == '__main__':
    main()
