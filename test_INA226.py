#!/usr/bin/env python3
"""
test_INA226.py  --  083026

Winch servo bench test with live power logging.

Merges the keyboard servo control from test_servor.py (082526) with INA226
voltage/current measurement, so every servo command is logged alongside what
the servo actually drew at that moment.

Wiring assumed:
    4S pack --> MP1584 --> shunt --> servo rail (~7.0 V)
    INA226 IN+/IN- across the shunt, VBUS on the LOAD side of the shunt.
    Servo signal on GPIO17 (pigpio), servo BEC/power NOT connected to the Pi.
    Optional: winch Hall sensor on ADS1115 (0x48) for position correlation.

Keys (unchanged from test_servor.py, plus [t]):
    a  more negative      d  more positive
    z  zero (1500 us)     x  STOP PULSES (servo off)
    r  0.4                v  -0.12
    t  re-tare shunt offset (stops pulses first)
    q  quit

Usage:
    sudo pigpiod
    python3 test_INA226.py --shunt-mohm 100
    python3 test_INA226.py --shunt-mohm 0.25 --hall-channel 0 --max-w 15

Requires: pip install smbus2 gpiozero pigpio
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

# ===== SERVO SETTINGS (carried over from test_servor.py) =====
PIN = 17            # your GPIO pin
MIN_PW = 0.0009     # 900 us
MAX_PW = 0.0021     # 2100 us
FRAME = 0.02        # 20 ms (50 Hz)
STEP = 0.005        # step size for speed adjustments
# =============================================================

# ===== I2C =====
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

# AVG code -> (samples averaged, conversion time for one averaged reading)
INA_AVG = {1: 0b000, 4: 0b001, 16: 0b010, 64: 0b011,
           128: 0b100, 256: 0b101, 512: 0b110, 1024: 0b111}

ADS_CONV = 0x00
ADS_CONFIG = 0x01
ADS_PGA = {6.144: 0b000, 4.096: 0b001, 2.048: 0b010,
           1.024: 0b011, 0.512: 0b100, 0.256: 0b101}


# ---------------------------------------------------------------------------
# Raw I2C helpers (both parts are big-endian; smbus word ops are not)
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


def ads_read_raw(bus, channel, full_scale=4.096):
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


# ---------------------------------------------------------------------------
def ina_setup(bus, avg):
    mfg = rd16(bus, INA_ADDR, INA_MFG_ID)
    die = rd16(bus, INA_ADDR, INA_DIE_ID)
    if not (mfg == 0x5449 and die == 0x2260):
        raise RuntimeError(f"0x{INA_ADDR:02X} is not an INA226 "
                           f"(mfg=0x{mfg:04X} die=0x{die:04X})")
    # VBUSCT = VSHCT = 1.1 ms, MODE = shunt+bus continuous
    cfg = 0x4000 | (INA_AVG[avg] << 9) | (0b100 << 6) | (0b100 << 3) | 0b111
    wr16(bus, INA_ADDR, INA_CONFIG, cfg)
    time.sleep(0.005)
    back = rd16(bus, INA_ADDR, INA_CONFIG)
    if back != cfg:
        raise RuntimeError(f"config readback 0x{back:04X} != 0x{cfg:04X}")
    conv_s = avg * 2 * 1.1e-3
    print(f"INA226  @ 0x{INA_ADDR:02X}  OK   avg={avg}  "
          f"conversion={conv_s * 1000:.1f} ms  config=0x{cfg:04X}")
    time.sleep(conv_s * 3)
    return conv_s


def tare_shunt(bus, n=64):
    """Average the shunt reading at zero current. Caller must stop pulses first."""
    acc = 0.0
    for _ in range(n):
        acc += ina_shunt_volts(bus)
        time.sleep(0.005)
    return acc / n


def value_to_usec(v):
    pw = MIN_PW + ((v + 1.0) / 2.0) * (MAX_PW - MIN_PW)
    return int(round(pw * 1e6))


# ---------------------------------------------------------------------------
def main():
    p = argparse.ArgumentParser(description="Servo bench test with INA226 logging")
    p.add_argument("--shunt-mohm", type=float, required=True,
                   help="shunt resistance in milliohms (e.g. 100 for a 0.1 ohm "
                        "onboard shunt, 0.25 for the 200A/50mV bus-bar shunt)")
    p.add_argument("--avg", type=int, default=4, choices=sorted(INA_AVG),
                   help="INA226 hardware averaging (default 4 = 8.8 ms/sample)")
    p.add_argument("--hz", type=float, default=50.0, help="log rate (default 50)")
    p.add_argument("--csv", default="servo_power_log.csv", help="output CSV path")
    p.add_argument("--hall-channel", type=int, default=None,
                   help="ADS1115 channel carrying the winch Hall sensor "
                        "(omit to skip the ADS1115 entirely)")
    p.add_argument("--hall-fs", type=float, default=4.096, choices=sorted(ADS_PGA),
                   help="ADS1115 full-scale range for the Hall channel")
    p.add_argument("--max-w", type=float, default=0.0,
                   help="watchdog: stop pulses if power exceeds this many watts "
                        "for --max-w-samples consecutive samples. 0 = disabled.")
    p.add_argument("--max-w-samples", type=int, default=10,
                   help="consecutive over-power samples before the watchdog trips")
    p.add_argument("--no-tare", action="store_true",
                   help="skip the startup zero-current tare")
    args = p.parse_args()

    r_shunt = args.shunt_mohm / 1000.0

    with SMBus(BUS_NUM) as bus:
        ina_setup(bus, args.avg)
        print(f"        shunt={args.shunt_mohm} mohm  ->  "
              f"current LSB = {INA_SHUNT_LSB_V / r_shunt * 1000:.3f} mA, "
              f"full scale = {81.92e-3 / r_shunt:.1f} A")

        if args.hall_channel is not None:
            try:
                raw = ads_read_raw(bus, args.hall_channel, args.hall_fs)
                print(f"ADS1115 @ 0x{ADS_ADDR:02X}  OK   "
                      f"Hall A{args.hall_channel} raw={raw}")
            except Exception as exc:
                print(f"ADS1115 @ 0x{ADS_ADDR:02X}  *** {exc} *** -- Hall disabled")
                args.hall_channel = None

        # -------------------------------------------------------------------
        # 082526: start with pulses stopped, not a 1500 us hold. A servo
        # commanded to hold still still draws current -- a damaged unit was
        # measured dissipating ~12 W in exactly that state.
        # -------------------------------------------------------------------
        factory = PiGPIOFactory()
        srv = Servo(PIN,
                    min_pulse_width=MIN_PW,
                    max_pulse_width=MAX_PW,
                    frame_width=FRAME,
                    pin_factory=factory,
                    initial_value=None)
        v = None
        srv.value = v
        time.sleep(0.5)

        offset_v = 0.0
        if not args.no_tare:
            print("\ntaring shunt offset at zero current (pulses stopped)...")
            offset_v = tare_shunt(bus)
            print(f"        offset = {offset_v * 1e6:+.1f} uV "
                  f"= {offset_v / r_shunt * 1000:+.1f} mA -- subtracted from here on")

        print("\nContinuous Servo Test with INA226 Power Logging")
        print("Controls: [a] more negative, [d] more positive, [z] zero (1500us),")
        print("          [x] STOP PULSES (servo off), [r] 0.4, [v] -0.12,")
        print("          [t] re-tare, [q] quit")
        if args.max_w > 0:
            print(f"Watchdog: pulses stop above {args.max_w:.1f} W for "
                  f"{args.max_w_samples} consecutive samples.")
        else:
            print("Watchdog: DISABLED (--max-w 0)")
        print(f"Logging to {os.path.abspath(args.csv)}\n")

        fh = open(args.csv, "w", newline="")
        wtr = csv.writer(fh)
        wtr.writerow(["t_s", "servo_value", "pulse_us", "bus_v", "shunt_uv",
                      "current_a", "power_w", "hall_raw", "event"])

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        tty.setcbreak(fd)

        period = 1.0 / args.hz
        t0 = time.time()
        next_sample = t0
        nrec = 0
        over = 0
        event = ""
        peak_w = 0.0
        peak_a = 0.0

        try:
            while True:
                dr, _, _ = select.select([sys.stdin], [], [], 0.005)
                if dr:
                    ch = sys.stdin.read(1)
                    if ch.lower() == 'q':
                        break
                    elif ch.lower() == 'a':
                        # 082526: v is None at startup and after x.
                        v = max(-1.0, (0.0 if v is None else v) - STEP)
                    elif ch.lower() == 'd':
                        v = min(+1.0, (0.0 if v is None else v) + STEP)
                    elif ch.lower() == 'r':
                        v = 0.4
                    elif ch.lower() == 'v':
                        v = -0.12
                    elif ch.lower() == 'z':
                        v = 0.0
                    elif ch.lower() == 'x':
                        v = None      # 082526: stop pulses, servo draws nothing
                    elif ch.lower() == 't':
                        v = None
                        srv.value = v
                        time.sleep(0.3)
                        offset_v = tare_shunt(bus)
                        event = f"retare {offset_v * 1e6:+.1f}uV"
                        over = 0
                    else:
                        continue
                    srv.value = v
                    if not event:
                        event = "key " + ch.lower()
                    over = 0

                now = time.time()
                if now < next_sample:
                    continue
                next_sample += period
                if next_sample < now:
                    next_sample = now + period

                vbus = ina_bus_volts(bus)
                vsh = ina_shunt_volts(bus) - offset_v
                amps = vsh / r_shunt
                watts = vbus * amps
                hall = ""
                if args.hall_channel is not None:
                    try:
                        hall = ads_read_raw(bus, args.hall_channel, args.hall_fs)
                    except Exception:
                        hall = ""

                peak_w = max(peak_w, watts)
                peak_a = max(peak_a, amps)

                # ---- watchdog ----
                if args.max_w > 0 and v is not None:
                    over = over + 1 if watts > args.max_w else 0
                    if over >= args.max_w_samples:
                        v = None
                        srv.value = v
                        event = f"WATCHDOG {watts:.1f}W"
                        over = 0

                wtr.writerow([f"{now - t0:.4f}",
                              "" if v is None else f"{v:+.4f}",
                              "" if v is None else value_to_usec(v),
                              f"{vbus:.4f}", f"{vsh * 1e6:.1f}",
                              f"{amps:.4f}", f"{watts:.3f}", hall, event])
                nrec += 1
                if nrec % 25 == 0:
                    fh.flush()

                cmd = ("value=None  pulses STOPPED"
                       if v is None else
                       f"value={v:+.3f} ~{value_to_usec(v)}us")
                hall_s = f" | hall {hall:>6}" if hall != "" else ""
                print(f"{cmd:<28} | {vbus:6.3f}V {amps:+7.3f}A {watts:7.3f}W"
                      f"{hall_s} | n={nrec:<6} {event:<18}",
                      end='\r', flush=True)
                event = ""

        except KeyboardInterrupt:
            pass
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            srv.value = None
            srv.detach()          # stop driving pulses
            fh.flush()
            fh.close()
            print("\n\nDone. Pulses stopped.")
            print(f"  samples logged : {nrec}  -> {os.path.abspath(args.csv)}")
            print(f"  peak current   : {peak_a:.3f} A")
            print(f"  peak power     : {peak_w:.3f} W")
            print(f"  shunt offset   : {offset_v * 1e6:+.1f} uV "
                  f"({args.shunt_mohm} mohm)")
            if v is not None:
                print(f"  last command   : value={v:+.3f} "
                      f"(~{value_to_usec(v)} us)")

    return 0


if __name__ == "__main__":
    sys.exit(main())
