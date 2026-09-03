#!/usr/bin/env python3
"""
test_INA226.py  --  083026 rev3

Bench winch test: keyboard servo control, INA226 rail logging, and the Hall
effect readout from test_winch.py, with an automatic stop when the Hall value
falls below a threshold.

rev3 adds:
  - ADS1115 Hall reading alongside every V/A sample, logged in the same CSV row
  - RELEASE / RETRACT keys at the real wParms power levels, replacing rev2's
    arbitrary 0.4 / -0.12 presets
  - hall auto-stop: while driving in the RETRACT direction, pulses are cut the
    moment Hall drops to HALL_STOP. This is the interlock the flight code has
    and the bench script did not -- driving a retract into the mechanical stop
    is what wore out the earlier servos, and it is easy to do by hand.

The ADS1115 is read directly over smbus2 rather than through Blinka, so both
chips share one open bus and there is no second I2C library to contend with.

KEYS (changed from rev2 -- r and t now match test_winch.py)
    r  RELEASE (extend)     t  RETRACT
    a  more negative        d  more positive
    z  neutral (1500 us)    x  STOP PULSES (servo off)
    o  re-tare shunt offset (stops pulses first)
    [  slower sampling      ]  faster sampling
    q  quit

Usage:
    sudo pigpiod
    python3 test_INA226.py
    python3 test_INA226.py --hall-stop 7100        # the fifth servo
    python3 test_INA226.py --no-hall               # rail only, no ADS1115
    python3 test_INA226.py --hz 200 --avg 1        # fast, noisier

CALIBRATION WARNING: --hall-stop defaults to 2550, which matches the units
that read ~1035 fully retracted. The fifth servo reads about 7000 retracted,
so on that unit the default will never trigger and the auto-stop is useless.
The startup banner prints the live Hall reading -- check it against the
threshold BEFORE driving anything.

MUST be run from a real terminal (or ssh). Thonny / IDE run panels do not
provide a tty and the keys will not work.
"""

import argparse
import csv
import os
import select
import sys
import termios
import time
import tty

from smbus2 import SMBus
from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory

# ===== SERVO SETTINGS (from test_servor.py) =====
PIN = 17            # your GPIO pin
MIN_PW = 0.0009     # 900 us
MAX_PW = 0.0021     # 2100 us
FRAME = 0.02        # 20 ms (50 Hz)
STEP = 0.005        # step size for a/d
# ================================================

# ===== WINCH SETTINGS (must match wParms in main_rc8_uart_parm.py) =====
ROTATION_DIR = -1       # -1 or 1
RELEASE_PWR = -0.30     # wParms RELEASE_PWR; servo value = ROTATION_DIR * this
RETRACT_PWR = 0.10      # wParms RETRACT_PWR
NEUTRAL_POS = 0.0
# =======================================================================

BUS_NUM = 1
INA_ADDR = 0x44
ADS_ADDR = 0x48

INA_CONFIG = 0x00
INA_SHUNT = 0x01
INA_BUS = 0x02
INA_MFG_ID = 0xFE
INA_DIE_ID = 0xFF

INA_BUS_LSB_V = 1.25e-3     # 1.25 mV/LSB
INA_SHUNT_LSB_V = 2.5e-6    # 2.5 uV/LSB, +/-81.92 mV full scale

INA_AVG = {1: 0b000, 4: 0b001, 16: 0b010, 64: 0b011,
           128: 0b100, 256: 0b101, 512: 0b110, 1024: 0b111}

ADS_CONV = 0x00
ADS_CONFIG = 0x01
ADS_PGA = {6.144: 0b000, 4.096: 0b001, 2.048: 0b010,
           1.024: 0b011, 0.512: 0b100, 0.256: 0b101}

HZ_CHOICES = [1, 2, 5, 10, 20, 50, 100, 200, 500]


# ---------------------------------------------------------------------------
def rd16(bus, addr, reg):
    d = bus.read_i2c_block_data(addr, reg, 2)
    return (d[0] << 8) | d[1]


def wr16(bus, addr, reg, val):
    bus.write_i2c_block_data(addr, reg, [(val >> 8) & 0xFF, val & 0xFF])


def as_signed(v):
    return v - 65536 if v & 0x8000 else v


def ina_bus_volts(bus):
    return (rd16(bus, INA_ADDR, INA_BUS) & 0x7FFF) * INA_BUS_LSB_V


def ina_shunt_volts(bus):
    return as_signed(rd16(bus, INA_ADDR, INA_SHUNT)) * INA_SHUNT_LSB_V


def ads_read(bus, channel, full_scale=4.096):
    """Single-shot single-ended ADS1115 read, raw counts.

    Matches what adafruit's AnalogIn.value returns at the same gain, so the
    numbers are directly comparable with test_winch.py and with HALL_TARGET /
    HALL_MAX in wParms.
    """
    cfg = ((1 << 15)                        # OS: start conversion
           | ((0b100 | channel) << 12)      # MUX: single-ended AINx
           | (ADS_PGA[full_scale] << 9)     # PGA
           | (1 << 8)                       # MODE: single-shot
           | (0b111 << 5)                   # DR: 860 SPS
           | 0b11)                          # COMP_QUE: disabled
    wr16(bus, ADS_ADDR, ADS_CONFIG, cfg)
    for _ in range(40):
        time.sleep(0.0015)
        if rd16(bus, ADS_ADDR, ADS_CONFIG) & 0x8000:
            break
    return as_signed(rd16(bus, ADS_ADDR, ADS_CONV))


def ina_setup(bus, avg):
    mfg = rd16(bus, INA_ADDR, INA_MFG_ID)
    die = rd16(bus, INA_ADDR, INA_DIE_ID)
    if not (mfg == 0x5449 and die == 0x2260):
        raise RuntimeError(f"0x{INA_ADDR:02X} is not an INA226 "
                           f"(mfg=0x{mfg:04X} die=0x{die:04X})")
    cfg = 0x4000 | (INA_AVG[avg] << 9) | (0b100 << 6) | (0b100 << 3) | 0b111
    wr16(bus, INA_ADDR, INA_CONFIG, cfg)
    time.sleep(0.005)
    back = rd16(bus, INA_ADDR, INA_CONFIG)
    if back != cfg:
        raise RuntimeError(f"config readback 0x{back:04X} != 0x{cfg:04X}")
    conv_s = avg * 2 * 1.1e-3
    print(f"INA226  @ 0x{INA_ADDR:02X}  OK   avg={avg}  "
          f"conversion={conv_s * 1000:.1f} ms  (max useful rate "
          f"{1.0 / conv_s:.0f} Hz)")
    time.sleep(conv_s * 3)
    return conv_s


def tare_shunt(bus, n=64):
    """Average the shunt reading at zero current. Pulses must be stopped."""
    acc = 0.0
    for _ in range(n):
        acc += ina_shunt_volts(bus)
        time.sleep(0.004)
    return acc / n


def value_to_usec(v):
    pw = MIN_PW + ((v + 1.0) / 2.0) * (MAX_PW - MIN_PW)
    return int(round(pw * 1e6))


def is_retracting(v):
    """True when the commanded value drives the winch inward. Same convention
    as is_driving_retract() in the flight code."""
    return v is not None and (ROTATION_DIR * v) > 0.0


def bar(val, lo, hi, width=18):
    if hi == lo:
        return "[" + "-" * width + "]"
    frac = max(0.0, min(1.0, (val - lo) / float(hi - lo)))
    n = int(frac * width)
    return "[" + "#" * n + "-" * (width - n) + "]"


# ---------------------------------------------------------------------------
def main():
    p = argparse.ArgumentParser(
        description="Bench winch test: servo, INA226 rail, Hall, auto-stop")
    p.add_argument("--hz", type=float, default=50.0,
                   help="sample/log rate in Hz (default 50); [ and ] change it live")
    p.add_argument("--avg", type=int, default=4, choices=sorted(INA_AVG),
                   help="INA226 hardware averaging (default 4 = 8.8 ms/sample)")
    p.add_argument("--shunt-mohm", type=float, default=9.091,
                   help="shunt resistance in milliohms (default 9.091 = R100 || R010)")
    p.add_argument("--csv", default="servo_power_log.csv", help="output CSV path")
    p.add_argument("--no-tare", action="store_true", help="skip the startup tare")
    # ---- Hall ----
    p.add_argument("--hall-ch", type=int, default=0,
                   help="ADS1115 channel the Hall sensor is on (default 0)")
    p.add_argument("--hall-fs", type=float, default=4.096, choices=sorted(ADS_PGA),
                   help="ADS1115 full-scale volts; must match how HALL_TARGET "
                        "and HALL_MAX were calibrated")
    p.add_argument("--hall-stop", type=float, default=2550,
                   help="cut pulses when Hall falls to this while RETRACTING. "
                        "Default suits units reading ~1035 retracted; the fifth "
                        "servo reads ~7000, so pass ~7100 for that one.")
    p.add_argument("--hall-max", type=float, default=12285,
                   help="Hall value when fully extended, for the display bar")
    p.add_argument("--no-hall", action="store_true",
                   help="skip the ADS1115 entirely; disables the auto-stop")
    p.add_argument("--no-autostop", action="store_true",
                   help="read and log Hall but never cut pulses automatically")
    args = p.parse_args()

    if not sys.stdin.isatty():
        print("ERROR: stdin is not a terminal, so keypresses cannot be read.\n"
              "Run this from a real terminal or an ssh session, not from an\n"
              "IDE run panel (Thonny, IDLE, VS Code output pane).")
        return 1

    r_shunt = args.shunt_mohm / 1000.0
    rate = args.hz
    release_val = ROTATION_DIR * RELEASE_PWR + NEUTRAL_POS
    retract_val = ROTATION_DIR * RETRACT_PWR + NEUTRAL_POS

    with SMBus(BUS_NUM) as bus:
        conv_s = ina_setup(bus, args.avg)
        print(f"        shunt = {args.shunt_mohm} mohm  ->  "
              f"LSB {INA_SHUNT_LSB_V / r_shunt * 1000:.3f} mA, "
              f"full scale {81.92e-3 / r_shunt:.2f} A")
        if rate > 1.0 / conv_s:
            print(f"        note: {rate:.0f} Hz is faster than the "
                  f"{1.0 / conv_s:.0f} Hz conversion rate -- samples will repeat")

        # ---- Hall ----
        hall_ok = not args.no_hall
        hall0 = None
        if hall_ok:
            try:
                hall0 = ads_read(bus, args.hall_ch, args.hall_fs)
                print(f"ADS1115 @ 0x{ADS_ADDR:02X}  OK   "
                      f"A{args.hall_ch} reads {hall0}")
            except Exception as exc:
                print(f"ADS1115 @ 0x{ADS_ADDR:02X}  *** {exc} ***")
                print("        Hall disabled; the auto-stop will NOT protect you.")
                hall_ok = False

        autostop = hall_ok and not args.no_autostop
        if hall_ok:
            print(f"        hall_stop = {args.hall_stop:.0f}   "
                  f"auto-stop {'ARMED' if autostop else 'DISABLED (--no-autostop)'}")
            # The calibration trap: if the mechanism already sits at or below
            # the threshold, a retract command would be cut instantly, and if
            # the threshold is far below the retracted value it never fires.
            if hall0 is not None and autostop:
                if hall0 <= args.hall_stop:
                    print(f"        WARNING: Hall {hall0} is already at or below "
                          f"the stop threshold. Retract will cut immediately.")
                elif hall0 - args.hall_stop > 5000:
                    print(f"        NOTE: Hall {hall0} is a long way above the "
                          f"threshold. If this unit is already retracted, "
                          f"--hall-stop is set too low to protect it.")

        # 082526: start with pulses stopped, not a 1500 us hold. A servo
        # commanded to hold still still draws current -- a damaged unit was
        # measured dissipating ~12 W in exactly that state.
        factory = PiGPIOFactory()
        srv = Servo(PIN, min_pulse_width=MIN_PW, max_pulse_width=MAX_PW,
                    frame_width=FRAME, pin_factory=factory, initial_value=None)
        v = None
        srv.value = v
        time.sleep(0.5)

        offset_v = 0.0
        if not args.no_tare:
            print("\ntaring shunt offset at zero current (pulses stopped)...")
            offset_v = tare_shunt(bus)
            print(f"        offset = {offset_v * 1e6:+.1f} uV = "
                  f"{offset_v / r_shunt * 1000:+.1f} mA")

        print("\nBench winch test")
        print(f"  [r] RELEASE {release_val:+.2f}   [t] RETRACT {retract_val:+.2f}"
              f"   [z] neutral   [x] STOP")
        print("  [a] nudge -        [d] nudge +        [o] re-tare")
        print("  [[] slower         []] faster         [q] quit")
        print(f"\nlogging to {os.path.abspath(args.csv)}\n")

        fh = open(args.csv, "w", newline="")
        wtr = csv.writer(fh)
        wtr.writerow(["t_s", "servo_value", "pulse_us", "bus_v", "shunt_uv",
                      "current_a", "power_w", "hall", "hz", "event"])

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        tty.setcbreak(fd)

        t0 = time.time()
        next_sample = t0
        nrec = 0
        event = ""
        peak_a = 0.0
        peak_w = 0.0
        hall = ""
        prev_hall = None
        n_autostop = 0
        running = True

        try:
            while running:
                # ---- keyboard: non-blocking, never skips the sampler ----
                while select.select([sys.stdin], [], [], 0)[0]:
                    ch = sys.stdin.read(1)
                    if ch == '':
                        running = False       # stdin closed
                        break
                    c = ch.lower()
                    if c == 'q':
                        running = False
                        break
                    elif c == 'a':
                        # 082526: v is None at startup and after x.
                        v = max(-1.0, (0.0 if v is None else v) - STEP)
                    elif c == 'd':
                        v = min(+1.0, (0.0 if v is None else v) + STEP)
                    elif c == 'r':
                        v = release_val          # rev3: was the 0.4 preset
                    elif c == 't':
                        v = retract_val          # rev3: was the re-tare key
                    elif c == 'z':
                        v = NEUTRAL_POS
                    elif c == 'x':
                        v = None   # 082526: stop pulses, servo draws nothing
                    elif c == 'o':               # rev3: re-tare moved off t
                        v = None
                        srv.value = v
                        time.sleep(0.3)
                        offset_v = tare_shunt(bus)
                        event = f"retare {offset_v * 1e6:+.1f}uV"
                        continue
                    elif c in '[]':
                        lower = [h for h in HZ_CHOICES if h < rate]
                        upper = [h for h in HZ_CHOICES if h > rate]
                        if c == '[' and lower:
                            rate = lower[-1]
                        elif c == ']' and upper:
                            rate = upper[0]
                        next_sample = time.time()
                        event = f"rate {rate:g}Hz"
                        continue
                    else:
                        continue          # unknown key: ignore, keep sampling
                    srv.value = v
                    event = f"key {c}"

                if not running:
                    break

                # ---- sampler, on its own schedule ----
                now = time.time()
                if now < next_sample:
                    time.sleep(min(0.002, next_sample - now))
                    continue
                period = 1.0 / rate
                next_sample += period
                if next_sample < now:
                    next_sample = now + period

                vbus = ina_bus_volts(bus)
                vsh = ina_shunt_volts(bus) - offset_v
                amps = vsh / r_shunt
                watts = vbus * amps
                peak_a = max(peak_a, amps)
                peak_w = max(peak_w, watts)

                if hall_ok:
                    try:
                        hall = ads_read(bus, args.hall_ch, args.hall_fs)
                    except Exception:
                        hall = ""

                # ---- hall auto-stop ----
                # Only while driving INWARD. Extending starts from the
                # retracted end, so an unconditional "stop below threshold"
                # would kill every release command the moment it was given.
                if (autostop and hall != "" and is_retracting(v)
                        and hall <= args.hall_stop):
                    v = None
                    srv.value = v
                    n_autostop += 1
                    event = f"HALL STOP at {hall} {vbus:.3f}V {amps:+.3f}A"
                    print(f"\n  *** HALL STOP: hall {hall} <= {args.hall_stop:.0f}"
                          f"  |  {vbus:.3f} V  {amps:+.3f} A  {watts:.2f} W"
                          f"  |  peak this run {peak_a:.3f} A")

                wtr.writerow([f"{now - t0:.4f}",
                              "" if v is None else f"{v:+.4f}",
                              "" if v is None else value_to_usec(v),
                              f"{vbus:.4f}", f"{vsh * 1e6:.1f}",
                              f"{amps:.4f}", f"{watts:.3f}",
                              hall, f"{rate:g}", event])
                nrec += 1
                if nrec % 25 == 0:
                    fh.flush()

                # ---- display ----
                if v is None:
                    cmd = "STOPPED"
                elif abs(v - release_val) < 1e-6:
                    cmd = f"RELEASE {v:+.3f}"
                elif abs(v - retract_val) < 1e-6:
                    cmd = f"RETRACT {v:+.3f}"
                elif abs(v - NEUTRAL_POS) < 1e-6:
                    cmd = "NEUTRAL"
                else:
                    cmd = f"{v:+.3f} ~{value_to_usec(v)}us"

                if hall == "":
                    hall_s = ""
                else:
                    d = ""
                    if prev_hall is not None:
                        if hall - prev_hall > 80:
                            d = ">>ext"
                        elif hall - prev_hall < -80:
                            d = "<<ret"
                    hall_s = (f" | {hall:6d} "
                              f"{bar(hall, args.hall_stop, args.hall_max)} {d:6s}")
                    prev_hall = hall

                print(f"{cmd:<18} | {vbus:6.3f}V {amps:+7.3f}A {watts:7.2f}W"
                      f"{hall_s} | {rate:>3g}Hz n={nrec:<5} {event:<18}",
                      end='\r', flush=True)
                event = ""

        except KeyboardInterrupt:
            pass
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            srv.value = None
            srv.detach()
            fh.flush()
            fh.close()
            print("\n\nDone. Pulses stopped.")
            print(f"  samples       : {nrec}  -> {os.path.abspath(args.csv)}")
            print(f"  peak current  : {peak_a:.3f} A")
            print(f"  peak power    : {peak_w:.3f} W")
            if hall_ok and hall != "":
                print(f"  final Hall    : {hall}")
            try:
                print(f"  final rail    : {vbus:.3f} V  {amps:+.3f} A  {watts:.2f} W")
            except NameError:
                pass
            if autostop:
                print(f"  auto-stops    : {n_autostop}")
            if v is not None:
                print(f"  last command  : value={v:+.3f} (~{value_to_usec(v)} us)")

    return 0


if __name__ == "__main__":
    sys.exit(main())
