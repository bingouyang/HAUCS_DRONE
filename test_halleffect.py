#!/usr/bin/env python3
"""
test_hall.py
------------
Standalone Hall effect sensor diagnostic for HAUCS winch.

Reads the ADS1115 ADC (I2C) and prints the raw Hall sensor value
continuously so you can verify wiring, range, and direction.

Usage:
    python3 test_hall.py

Move the winch spool manually and watch the values change.
Expected behaviour:
    - Fully retracted  : value near HALL_TARGET (~2500)
    - Fully extended   : value near HALL_MAX    (~12285)
    - Moving away      : value increases
    - Moving back      : value decreases

If values go the WRONG direction when you extend the spool,
the Hall sensor polarity is reversed -- check wiring.

Press Ctrl+C to exit.
"""

import time
import sys

# ── CONFIG (must match wParms in main_rc8_uart_parm.py) ──────────────────────
ADC_CHANNEL  = 0       # ADS1115 channel the Hall sensor is wired to
HALL_TARGET  = 2500    # expected value when fully retracted
HALL_MAX     = 12285   # expected value when fully extended
RETRACT_TH   = 8000    # threshold used in retract logic
SAMPLE_HZ    = 5       # how many readings per second to print
# ─────────────────────────────────────────────────────────────────────────────

def init_adc():
    try:
        import board
        import busio
        import adafruit_ads1x15.ads1115 as ADS
        from adafruit_ads1x15.analog_in import AnalogIn
    except ImportError as e:
        print("ERROR: missing library -- %s" % e)
        print("Install with: pip install adafruit-circuitpython-ads1x15 --break-system-packages")
        sys.exit(1)

    try:
        i2c = busio.I2C(board.SCL, board.SDA)
        ads = ADS.ADS1115(i2c)
        adc = AnalogIn(ads, ADC_CHANNEL)
        return adc
    except Exception as e:
        print("ERROR: ADS1115 init failed -- %s" % e)
        print()
        print("Possible causes:")
        print("  1. I2C not enabled  -- run: sudo raspi-config -> Interface Options -> I2C -> Enable")
        print("  2. Wrong wiring     -- check SDA/SCL connections")
        print("  3. Wrong I2C address -- run: i2cdetect -y 1  (should show 0x48)")
        print("  4. Missing power    -- check 3.3V/5V and GND to ADS1115")
        sys.exit(1)


def read_raw(adc):
    # ADS1115 .value returns a 16-bit signed int scaled to full range
    return int(adc.value)


def bar(val, lo, hi, width=30):
    """Simple ASCII progress bar."""
    frac = max(0.0, min(1.0, (val - lo) / (hi - lo)))
    filled = int(frac * width)
    return "[" + "#" * filled + "-" * (width - filled) + "]"


def status(val):
    if val <= HALL_TARGET + 500:
        return "RETRACTED   "
    elif val >= RETRACT_TH:
        return "EXTENDED    "
    else:
        return "MID-TRAVEL  "


def main():
    print()
    print("=" * 55)
    print("  HAUCS Hall Effect Sensor Test")
    print("=" * 55)
    print("  ADC channel : %d" % ADC_CHANNEL)
    print("  HALL_TARGET : %d  (fully retracted)" % HALL_TARGET)
    print("  RETRACT_TH  : %d  (considered extended)" % RETRACT_TH)
    print("  HALL_MAX    : %d  (fully extended)" % HALL_MAX)
    print()
    print("  Move the spool and watch values change.")
    print("  Retract -> value should DECREASE toward %d" % HALL_TARGET)
    print("  Extend  -> value should INCREASE toward %d" % HALL_MAX)
    print()
    print("  If direction is REVERSED, Hall sensor polarity is wrong.")
    print()
    print("  Press Ctrl+C to exit.")
    print("=" * 55)
    print()

    print("Initialising ADS1115...")
    adc = init_adc()
    print("ADS1115 OK")
    print()

    # First reading sanity check
    try:
        first = read_raw(adc)
        print("First reading: %d" % first)
        if first < 100:
            print("WARNING: reading very low -- check sensor power and wiring")
        elif first > 15000:
            print("WARNING: reading very high -- sensor may be saturated")
        else:
            print("Reading looks plausible.")
    except Exception as e:
        print("ERROR reading ADC: %s" % e)
        sys.exit(1)

    print()
    print("  %-8s  %-6s  %-32s  %s" % ("TIME(s)", "VALUE", "POSITION", "STATUS"))
    print("  " + "-" * 65)

    interval = 1.0 / SAMPLE_HZ
    t_start  = time.time()
    prev_val = None

    try:
        while True:
            t = time.time() - t_start
            val = read_raw(adc)

            direction = ""
            if prev_val is not None:
                diff = val - prev_val
                if abs(diff) > 50:
                    direction = "^^ extending" if diff > 0 else "vv retracting"

            b = bar(val, HALL_TARGET, HALL_MAX)
            s = status(val)
            print("  %-8.1f  %-6d  %s  %s  %s" % (t, val, b, s, direction))

            prev_val = val
            time.sleep(interval)

    except KeyboardInterrupt:
        print()
        print("=" * 55)
        print("  Test complete.")
        print("  Last reading : %d" % prev_val if prev_val else "  No readings taken.")
        print()
        print("  CHECKLIST:")
        print("  [ ] Value near %d when fully retracted" % HALL_TARGET)
        print("  [ ] Value near %d when fully extended" % HALL_MAX)
        print("  [ ] Value INCREASES when extending")
        print("  [ ] Value DECREASES when retracting")
        print("  [ ] No stuck/frozen readings")
        print("=" * 55)
        print()


if __name__ == "__main__":
    main()
