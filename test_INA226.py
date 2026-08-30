#!/usr/bin/env python3
"""
test_INA226.py  --  083026 rev2

Keyboard servo control (as in test_servor.py) with live INA226 V/A/W logging.

Fix in rev2: the keyboard loop is restructured so that an unrecognised key or
a non-interactive stdin can no longer starve the sampler. rev1 would log a
fraction of a second and then spin silently -- which is what you saw.

Keys:
    a  more negative       d  more positive
    z  zero (1500 us)      x  STOP PULSES (servo off)
    r  0.4                 v  -0.12
    [  slower sampling     ]  faster sampling
    t  re-tare shunt offset (stops pulses first)
    q  quit

Usage:
    sudo pigpiod
    python3 test_INA226.py                       # 50 Hz, shunt 9.091 mohm
    python3 test_INA226.py --hz 200 --avg 1      # fast, noisier
    python3 test_INA226.py --hz 10  --avg 64     # slow, very quiet

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
STEP = 0.005        # step size for speed adjustments
# ================================================

BUS_NUM = 1
INA_ADDR = 0x44

INA_CONFIG = 0x00
INA_SHUNT = 0x01
INA_BUS = 0x02
INA_MFG_ID = 0xFE
INA_DIE_ID = 0xFF

INA_BUS_LSB_V = 1.25e-3     # 1.25 mV/LSB
INA_SHUNT_LSB_V = 2.5e-6    # 2.5 uV/LSB, +/-81.92 mV full scale

INA_AVG = {1: 0b000, 4: 0b001, 16: 0b010, 64: 0b011,
           128: 0b100, 256: 0b101, 512: 0b110, 1024: 0b111}

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
    print(f"INA226 @ 0x{INA_ADDR:02X}  OK   avg={avg}  "
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


# ---------------------------------------------------------------------------
def main():
    p = argparse.ArgumentParser(description="Servo bench test with INA226 logging")
    p.add_argument("--hz", type=float, default=50.0,
                   help="sample/log rate in Hz (default 50). Can also be "
                        "changed at runtime with [ and ].")
    p.add_argument("--avg", type=int, default=4, choices=sorted(INA_AVG),
                   help="INA226 hardware averaging (default 4 = 8.8 ms/sample). "
                        "Higher = quieter but slower.")
    p.add_argument("--shunt-mohm", type=float, default=9.091,
                   help="shunt resistance in milliohms (default 9.091 = "
                        "R100 || R010). Affects current/power only, never bus_v.")
    p.add_argument("--csv", default="servo_power_log.csv", help="output CSV path")
    p.add_argument("--no-tare", action="store_true", help="skip the startup tare")
    args = p.parse_args()

    if not sys.stdin.isatty():
        print("ERROR: stdin is not a terminal, so keypresses cannot be read.\n"
              "Run this from a real terminal or an ssh session, not from an\n"
              "IDE run panel (Thonny, IDLE, VS Code output pane).")
        return 1

    r_shunt = args.shunt_mohm / 1000.0
    rate = args.hz

    with SMBus(BUS_NUM) as bus:
        conv_s = ina_setup(bus, args.avg)
        print(f"        shunt = {args.shunt_mohm} mohm  ->  "
              f"LSB {INA_SHUNT_LSB_V / r_shunt * 1000:.3f} mA, "
              f"full scale {81.92e-3 / r_shunt:.2f} A")
        if rate > 1.0 / conv_s:
            print(f"        note: {rate:.0f} Hz is faster than the "
                  f"{1.0 / conv_s:.0f} Hz conversion rate -- samples will repeat")

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

        print("\nServo control + INA226 logging")
        print("  [a] more negative   [d] more positive   [z] zero   [x] STOP")
        print("  [r] 0.4             [v] -0.12           [t] re-tare")
        print("  [[] slower          []] faster          [q] quit")
        print(f"\nlogging to {os.path.abspath(args.csv)}\n")

        fh = open(args.csv, "w", newline="")
        wtr = csv.writer(fh)
        wtr.writerow(["t_s", "servo_value", "pulse_us", "bus_v",
                      "shunt_uv", "current_a", "power_w", "hz", "event"])

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        tty.setcbreak(fd)

        t0 = time.time()
        next_sample = t0
        nrec = 0
        event = ""
        peak_a = 0.0
        peak_w = 0.0
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
                        v = 0.4
                    elif c == 'v':
                        v = -0.12
                    elif c == 'z':
                        v = 0.0
                    elif c == 'x':
                        v = None   # 082526: stop pulses, servo draws nothing
                    elif c == 't':
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

                wtr.writerow([f"{now - t0:.4f}",
                              "" if v is None else f"{v:+.4f}",
                              "" if v is None else value_to_usec(v),
                              f"{vbus:.4f}", f"{vsh * 1e6:.1f}",
                              f"{amps:.4f}", f"{watts:.3f}",
                              f"{rate:g}", event])
                nrec += 1
                if nrec % 25 == 0:
                    fh.flush()

                cmd = ("value=None  STOPPED" if v is None
                       else f"value={v:+.3f} ~{value_to_usec(v)}us")
                print(f"{cmd:<26} | {vbus:6.3f}V {amps:+7.3f}A {watts:7.3f}W "
                      f"| {rate:>3g}Hz n={nrec:<6} {event:<16}",
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
            print(f"  samples      : {nrec}  -> {os.path.abspath(args.csv)}")
            print(f"  peak current : {peak_a:.3f} A")
            print(f"  peak power   : {peak_w:.3f} W")
            if v is not None:
                print(f"  last command : value={v:+.3f} (~{value_to_usec(v)} us)")

    return 0


if __name__ == "__main__":
    sys.exit(main())
