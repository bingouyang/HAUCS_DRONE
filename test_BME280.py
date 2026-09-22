#!/usr/bin/env python3
"""
test_BME280.py  --  092226

Standalone BME280 check: identify the chip, dump its calibration, and log
temperature / pressure / humidity.

Read directly over smbus2 rather than through Blinka, to match ina_helper.py
and test_INA226.py -- one dependency, and the raw register view is what you
want when a bus is misbehaving.

Bus map on this Pi, for reference:
    0x44   INA226      rail monitor
    0x48   ADS1115     Hall sensor
    0x76   BME280      this script  (0x77 if SDO is pulled high)
No address collision, so this can run alongside the others -- though not at
the same time as the flight script, which holds /dev/serial0 and polls the
INA226.

Usage:
    python3 test_BME280.py                      # auto-detect, 1 Hz, forever
    python3 test_BME280.py --hz 5 --secs 60
    python3 test_BME280.py --addr 0x77
    python3 test_BME280.py --csv bme_log.csv
    python3 test_BME280.py --calib             # dump coefficients and exit

IMPORTANT -- BMP280 vs BME280. Boards sold as "BME280" are very often
BMP280, which has NO humidity sensor. The chip ID register settles it:
    0x60  BME280   temperature + pressure + humidity
    0x58  BMP280   temperature + pressure only
    0x56/0x57/0x58  BMP280 samples
This script reports which it found and omits humidity on a BMP280 rather
than printing a garbage number.
"""

import argparse
import csv
import os
import struct
import sys
import time

from smbus2 import SMBus

BUS_NUM = 1
ADDR_CANDIDATES = (0x76, 0x77)

# ---- registers -----------------------------------------------------------
REG_CHIP_ID = 0xD0
REG_RESET = 0xE0
REG_CTRL_HUM = 0xF2
REG_STATUS = 0xF3
REG_CTRL_MEAS = 0xF4
REG_CONFIG = 0xF5
REG_DATA = 0xF7          # press(3) temp(3) hum(2) = 8 bytes
REG_CALIB1 = 0x88        # 26 bytes, 0x88..0xA1
REG_CALIB2 = 0xE1        # 7 bytes,  0xE1..0xE7

CHIP_BME280 = 0x60
CHIP_BMP280 = (0x56, 0x57, 0x58)

# oversampling codes: 0=skip 1=x1 2=x2 3=x4 4=x8 5=x16
OSRS = {0: "skip", 1: "x1", 2: "x2", 4: "x4", 8: "x8", 16: "x16"}
OSRS_CODE = {0: 0, 1: 1, 2: 2, 4: 3, 8: 4, 16: 5}

# IIR filter coefficients
FILTER_CODE = {0: 0, 2: 1, 4: 2, 8: 3, 16: 4}


def rd(bus, addr, reg, n=1):
    return bus.read_i2c_block_data(addr, reg, n)


def wr(bus, addr, reg, val):
    bus.write_byte_data(addr, reg, val)


# ---------------------------------------------------------------------------
def detect(bus, want=None):
    """Return (addr, chip_id). Tries 0x76 then 0x77 unless want is given."""
    tried = []
    for addr in ((want,) if want else ADDR_CANDIDATES):
        try:
            cid = rd(bus, addr, REG_CHIP_ID)[0]
            tried.append((addr, cid))
            if cid == CHIP_BME280 or cid in CHIP_BMP280:
                return addr, cid
        except Exception as e:
            tried.append((addr, "no response (%s)" % e.__class__.__name__))
    raise RuntimeError(
        "no BME280/BMP280 found. Tried: %s"
        % ", ".join("0x%02X=%s" % (a, ("0x%02X" % c) if isinstance(c, int) else c)
                    for a, c in tried))


def read_calibration(bus, addr, has_hum):
    """Parse the factory coefficients. These are little-endian, unlike the
    measurement registers, which are big-endian -- a classic source of
    nonsense readings when hand-rolling this driver."""
    c = {}
    b1 = bytes(rd(bus, addr, REG_CALIB1, 26))
    (c["T1"], c["T2"], c["T3"],
     c["P1"], c["P2"], c["P3"], c["P4"], c["P5"],
     c["P6"], c["P7"], c["P8"], c["P9"]) = struct.unpack("<HhhHhhhhhhhh", b1[0:24])
    c["H1"] = b1[25]

    if has_hum:
        b2 = bytes(rd(bus, addr, REG_CALIB2, 7))
        c["H2"] = struct.unpack("<h", b2[0:2])[0]
        c["H3"] = b2[2]
        e4, e5, e6 = b2[3], b2[4], b2[5]
        # H4 and H5 are 12-bit signed, packed across three bytes sharing e5
        h4 = (e4 << 4) | (e5 & 0x0F)
        h5 = (e6 << 4) | (e5 >> 4)
        c["H4"] = h4 - 4096 if h4 > 2047 else h4
        c["H5"] = h5 - 4096 if h5 > 2047 else h5
        c["H6"] = struct.unpack("<b", b2[6:7])[0]
    return c


def configure(bus, addr, has_hum, osrs_t=16, osrs_p=16, osrs_h=16, filt=16):
    """Normal mode, continuous conversion."""
    if has_hum:
        wr(bus, addr, REG_CTRL_HUM, OSRS_CODE[osrs_h])
    # t_sb = 000 (0.5 ms standby), filter, spi3w off
    wr(bus, addr, REG_CONFIG, (0 << 5) | (FILTER_CODE[filt] << 2))
    # osrs_t, osrs_p, mode = 11 (normal)
    wr(bus, addr, REG_CTRL_MEAS,
       (OSRS_CODE[osrs_t] << 5) | (OSRS_CODE[osrs_p] << 2) | 0b11)
    time.sleep(0.1)
    back = rd(bus, addr, REG_CTRL_MEAS)[0]
    return back


def read_raw(bus, addr, has_hum):
    n = 8 if has_hum else 6
    d = rd(bus, addr, REG_DATA, n)
    adc_p = (d[0] << 12) | (d[1] << 4) | (d[2] >> 4)
    adc_t = (d[3] << 12) | (d[4] << 4) | (d[5] >> 4)
    adc_h = ((d[6] << 8) | d[7]) if has_hum else None
    return adc_t, adc_p, adc_h


# ---- Bosch compensation, floating point form ------------------------------
def compensate_t(adc_t, c):
    v1 = (adc_t / 16384.0 - c["T1"] / 1024.0) * c["T2"]
    v2 = ((adc_t / 131072.0 - c["T1"] / 8192.0) ** 2) * c["T3"]
    t_fine = v1 + v2
    return t_fine / 5120.0, t_fine


def compensate_p(adc_p, t_fine, c):
    v1 = t_fine / 2.0 - 64000.0
    v2 = v1 * v1 * c["P6"] / 32768.0
    v2 = v2 + v1 * c["P5"] * 2.0
    v2 = v2 / 4.0 + c["P4"] * 65536.0
    v1 = (c["P3"] * v1 * v1 / 524288.0 + c["P2"] * v1) / 524288.0
    v1 = (1.0 + v1 / 32768.0) * c["P1"]
    if v1 == 0:
        return None
    p = 1048576.0 - adc_p
    p = (p - v2 / 4096.0) * 6250.0 / v1
    v1 = c["P9"] * p * p / 2147483648.0
    v2 = p * c["P8"] / 32768.0
    return p + (v1 + v2 + c["P7"]) / 16.0      # Pa


def compensate_h(adc_h, t_fine, c):
    if adc_h is None:
        return None
    h = t_fine - 76800.0
    h = (adc_h - (c["H4"] * 64.0 + c["H5"] / 16384.0 * h)) * (
        c["H2"] / 65536.0 * (1.0 + c["H6"] / 67108864.0 * h
                             * (1.0 + c["H3"] / 67108864.0 * h)))
    h = h * (1.0 - c["H1"] * h / 524288.0)
    return max(0.0, min(100.0, h))


# ---------------------------------------------------------------------------
def main():
    p = argparse.ArgumentParser(description="BME280 / BMP280 test")
    p.add_argument("--addr", type=lambda x: int(x, 0), default=None,
                   help="I2C address (0x76 or 0x77); omit to auto-detect")
    p.add_argument("--hz", type=float, default=1.0, help="sample rate (default 1)")
    p.add_argument("--secs", type=float, default=0.0,
                   help="run for this long; 0 = until Ctrl+C")
    p.add_argument("--csv", default=None, help="also log to this CSV")
    p.add_argument("--calib", action="store_true",
                   help="dump the calibration coefficients and exit")
    p.add_argument("--filter", type=int, default=16, choices=sorted(FILTER_CODE),
                   help="IIR filter coefficient (default 16, heavy smoothing)")
    p.add_argument("--osrs", type=int, default=16, choices=sorted(OSRS_CODE),
                   help="oversampling for all three channels (default 16)")
    p.add_argument("--slp", type=float, default=1013.25,
                   help="sea-level pressure in hPa, for the altitude estimate")
    args = p.parse_args()

    with SMBus(BUS_NUM) as bus:
        try:
            addr, cid = detect(bus, args.addr)
        except Exception as e:
            print("ERROR: %s" % e)
            print("\nCheck with:  i2cdetect -y 1     (expect 76 or 77)")
            print("If nothing shows at all, the bus itself is down -- SDA stuck")
            print("low reads as a dark LED on a terminal HAT.")
            return 1

        has_hum = (cid == CHIP_BME280)
        name = "BME280" if has_hum else "BMP280"
        print("%s @ 0x%02X   chip_id=0x%02X" % (name, addr, cid))
        if not has_hum:
            print("        NOTE: chip ID says BMP280 -- pressure and temperature")
            print("        only, NO humidity sensor. Boards are routinely sold as")
            print("        BME280 when they are this part.")

        c = read_calibration(bus, addr, has_hum)
        if args.calib:
            print("\ncalibration coefficients:")
            for k in ("T1", "T2", "T3"):
                print("   dig_%-3s %8d" % (k, c[k]))
            for k in ("P1", "P2", "P3", "P4", "P5", "P6", "P7", "P8", "P9"):
                print("   dig_%-3s %8d" % (k, c[k]))
            if has_hum:
                for k in ("H1", "H2", "H3", "H4", "H5", "H6"):
                    print("   dig_%-3s %8d" % (k, c[k]))
            # all-zero or all-0xFF coefficients mean the reads are not landing
            vals = [c[k] for k in c]
            if all(v == 0 for v in vals):
                print("\n   *** every coefficient is 0 -- reads are not working")
            elif all(v in (65535, -1, 255) for v in vals):
                print("\n   *** every coefficient is 0xFF -- SDA likely floating")
            return 0

        back = configure(bus, addr, has_hum,
                         osrs_t=args.osrs, osrs_p=args.osrs,
                         osrs_h=args.osrs, filt=args.filter)
        print("        osrs=x%d filter=%d  ctrl_meas readback=0x%02X"
              % (args.osrs, args.filter, back))
        print("        sea-level ref %.2f hPa for the altitude column" % args.slp)
        print()

        fh = wtr = None
        if args.csv:
            fh = open(args.csv, "w", newline="")
            wtr = csv.writer(fh)
            wtr.writerow(["t_s", "temp_C", "pressure_hPa", "humidity_pct",
                          "altitude_m", "adc_t", "adc_p", "adc_h"])
            print("logging to %s" % os.path.abspath(args.csv))

        hdr = "%8s %10s %14s" % ("t(s)", "temp(C)", "press(hPa)")
        if has_hum:
            hdr += " %9s" % "hum(%)"
        hdr += " %10s" % "alt(m)"
        print(hdr)
        print("-" * len(hdr))

        t0 = time.time()
        n = 0
        period = 1.0 / args.hz if args.hz > 0 else 0.0
        tmin = tmax = pmin = pmax = None
        try:
            while True:
                if args.secs and (time.time() - t0) >= args.secs:
                    break
                adc_t, adc_p, adc_h = read_raw(bus, addr, has_hum)
                temp, t_fine = compensate_t(adc_t, c)
                pa = compensate_p(adc_p, t_fine, c)
                hum = compensate_h(adc_h, t_fine, c)
                hpa = pa / 100.0 if pa else float("nan")
                # barometric formula, same shape as the ADS1115 depth maths but
                # in air rather than water
                alt = 44330.0 * (1.0 - (hpa / args.slp) ** 0.1903) if pa else float("nan")

                tmin = temp if tmin is None else min(tmin, temp)
                tmax = temp if tmax is None else max(tmax, temp)
                pmin = hpa if pmin is None else min(pmin, hpa)
                pmax = hpa if pmax is None else max(pmax, hpa)

                el = time.time() - t0
                line = "%8.2f %10.3f %14.3f" % (el, temp, hpa)
                if has_hum:
                    line += " %9.3f" % hum
                line += " %10.2f" % alt
                print(line)

                if wtr:
                    wtr.writerow(["%.3f" % el, "%.4f" % temp, "%.4f" % hpa,
                                  "" if hum is None else "%.4f" % hum,
                                  "%.3f" % alt, adc_t, adc_p,
                                  "" if adc_h is None else adc_h])
                    n += 1
                    if n % 20 == 0:
                        fh.flush()

                time.sleep(period)
        except KeyboardInterrupt:
            pass
        except Exception as e:
            print("\nread failed: %s" % e)
            print("If this started mid-run, suspect the bus rather than the chip.")
        finally:
            if fh:
                fh.flush()
                fh.close()
            print()
            if tmin is not None:
                print("  temp     %.3f .. %.3f C   (spread %.3f)"
                      % (tmin, tmax, tmax - tmin))
                print("  pressure %.3f .. %.3f hPa (spread %.3f = %.1f cm of air)"
                      % (pmin, pmax, pmax - pmin, (pmax - pmin) * 8.3))
            if args.csv:
                print("  wrote %d rows to %s" % (n, os.path.abspath(args.csv)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
