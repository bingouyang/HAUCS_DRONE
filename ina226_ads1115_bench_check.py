#!/usr/bin/env python3
"""
ina226_ads1115_bench_check.py  --  083026

Bench validation for INA226 (0x44) + ADS1115 (0x48) on a Raspberry Pi 4.

Test condition this is written for:
    A known DC voltage (measured on a multimeter) applied to the INA226
    VBUS pin and IN+, with no motor current flowing.

What it checks:
    1. INA226 identity        -- mfg ID 0x5449 ("TI"), die ID 0x2260
    2. Bus voltage accuracy   -- register reading vs. the multimeter reference
    3. Shunt offset at zero A -- what the differential input reads with no current
    4. ADS1115 cross-check    -- same node measured through the resistor divider

Usage:
    python3 ina226_ads1115_bench_check.py --ref 7.03
    python3 ina226_ads1115_bench_check.py --ref 7.03 --secs 30 --hz 2
    python3 ina226_ads1115_bench_check.py --ref 7.03 --no-ads

Requires: pip install smbus2   (or: sudo apt install python3-smbus2)
"""

import argparse
import sys
import time

from smbus2 import SMBus

# ----------------------------------------------------------------------------
# Bus / device addresses
# ----------------------------------------------------------------------------
BUS_NUM = 1
INA_ADDR = 0x44
ADS_ADDR = 0x48

# ----------------------------------------------------------------------------
# INA226
# ----------------------------------------------------------------------------
INA_CONFIG = 0x00
INA_SHUNT = 0x01
INA_BUS = 0x02
INA_MFG_ID = 0xFE
INA_DIE_ID = 0xFF

INA_BUS_LSB_V = 1.25e-3     # 1.25 mV/LSB, 40.96 V full scale
INA_SHUNT_LSB_V = 2.5e-6    # 2.5 uV/LSB, +/-81.92 mV full scale

# AVG=16 (010), VBUSCT=1.1ms (100), VSHCT=1.1ms (100), MODE=shunt+bus cont (111)
# 0x4000 | (0b010<<9) | (0b100<<6) | (0b100<<3) | 0b111
INA_CONFIG_VAL = 0x4527
INA_CONV_TIME_S = 16 * (1.1e-3 + 1.1e-3)   # ~35 ms per averaged sample

# ----------------------------------------------------------------------------
# ADS1115
# ----------------------------------------------------------------------------
ADS_CONV = 0x00
ADS_CONFIG = 0x01
ADS_PGA = {6.144: 0b000, 4.096: 0b001, 2.048: 0b010,
           1.024: 0b011, 0.512: 0b100, 0.256: 0b101}


# ----------------------------------------------------------------------------
# Raw I2C helpers (both parts are big-endian; smbus word ops are not)
# ----------------------------------------------------------------------------
def rd16(bus, addr, reg):
    d = bus.read_i2c_block_data(addr, reg, 2)
    return (d[0] << 8) | d[1]


def wr16(bus, addr, reg, val):
    bus.write_i2c_block_data(addr, reg, [(val >> 8) & 0xFF, val & 0xFF])


def as_signed(v):
    return v - 65536 if v & 0x8000 else v


# ----------------------------------------------------------------------------
# Device reads
# ----------------------------------------------------------------------------
def ina_bus_volts(bus):
    """Bus voltage in volts. Bit 15 is unused on the INA226."""
    return (rd16(bus, INA_ADDR, INA_BUS) & 0x7FFF) * INA_BUS_LSB_V


def ina_shunt_volts(bus):
    """Differential shunt voltage in volts (signed)."""
    return as_signed(rd16(bus, INA_ADDR, INA_SHUNT)) * INA_SHUNT_LSB_V


def ads_read(bus, channel, full_scale=4.096):
    """Single-shot single-ended read. Returns (raw_counts, volts)."""
    cfg = ((1 << 15)                            # OS: start conversion
           | ((0b100 | channel) << 12)          # MUX: single-ended AINx
           | (ADS_PGA[full_scale] << 9)         # PGA
           | (1 << 8)                           # MODE: single-shot
           | (0b100 << 5)                       # DR: 128 SPS
           | 0b11)                              # COMP_QUE: disabled
    wr16(bus, ADS_ADDR, ADS_CONFIG, cfg)
    time.sleep(0.010)
    for _ in range(50):
        if rd16(bus, ADS_ADDR, ADS_CONFIG) & 0x8000:
            break
        time.sleep(0.002)
    else:
        raise TimeoutError("ADS1115 conversion did not complete")
    raw = as_signed(rd16(bus, ADS_ADDR, ADS_CONV))
    return raw, raw * full_scale / 32768.0


# ----------------------------------------------------------------------------
# Identity / setup
# ----------------------------------------------------------------------------
def check_ina_identity(bus):
    mfg = rd16(bus, INA_ADDR, INA_MFG_ID)
    die = rd16(bus, INA_ADDR, INA_DIE_ID)
    ok = (mfg == 0x5449 and die == 0x2260)
    print(f"INA226  @ 0x{INA_ADDR:02X}   mfg=0x{mfg:04X}  die=0x{die:04X}   "
          f"{'OK' if ok else '*** NOT AN INA226 ***'}")
    return ok


def configure_ina(bus):
    wr16(bus, INA_ADDR, INA_CONFIG, INA_CONFIG_VAL)
    time.sleep(0.005)
    readback = rd16(bus, INA_ADDR, INA_CONFIG)
    print(f"        config written=0x{INA_CONFIG_VAL:04X}  "
          f"readback=0x{readback:04X}  "
          f"{'OK' if readback == INA_CONFIG_VAL else '*** MISMATCH ***'}")
    time.sleep(INA_CONV_TIME_S * 2)


# ----------------------------------------------------------------------------
def main():
    p = argparse.ArgumentParser(description="INA226 + ADS1115 bench check")
    p.add_argument("--ref", type=float, required=True,
                   help="multimeter reference voltage at the VBUS node, e.g. 7.03")
    p.add_argument("--secs", type=float, default=10.0, help="run duration")
    p.add_argument("--hz", type=float, default=2.0, help="sample rate")
    p.add_argument("--ads-channel", type=int, default=1,
                   help="ADS1115 channel carrying the divider output (default A1)")
    p.add_argument("--divider", type=float, default=106.8 / 6.8,
                   help="divider ratio (R1+R2)/R2, default 100k/6.8k = 15.706")
    p.add_argument("--ads-fs", type=float, default=4.096, choices=sorted(ADS_PGA),
                   help="ADS1115 full-scale range in volts")
    p.add_argument("--no-ads", action="store_true", help="skip the ADS1115 channel")
    p.add_argument("--shunt-mohm", type=float, default=None,
                   help="shunt resistance in milliohms; if given, reports the "
                        "zero-current offset as an equivalent current error")
    args = p.parse_args()

    with SMBus(BUS_NUM) as bus:
        print()
        if not check_ina_identity(bus):
            print("Aborting: unexpected device ID at 0x44.")
            return 1
        configure_ina(bus)

        if not args.no_ads:
            try:
                raw, _ = ads_read(bus, args.ads_channel, args.ads_fs)
                print(f"ADS1115 @ 0x{ADS_ADDR:02X}   responding "
                      f"(A{args.ads_channel} raw={raw})")
            except Exception as exc:
                print(f"ADS1115 @ 0x{ADS_ADDR:02X}   *** {exc} ***")
                args.no_ads = True

        print(f"\nReference (multimeter): {args.ref:.4f} V")
        print(f"Divider ratio:          {args.divider:.4f}")
        print()

        hdr = f"{'t(s)':>7} {'INA bus(V)':>11} {'err(mV)':>9} {'err(%)':>8} {'shunt(uV)':>11}"
        if not args.no_ads:
            hdr += f" {'ADS(V)':>9} {'ADS node(V)':>12} {'err(mV)':>9}"
        print(hdr)
        print("-" * len(hdr))

        bus_samples, shunt_samples, ads_node_samples = [], [], []
        t0 = time.time()
        period = 1.0 / args.hz

        try:
            while (t := time.time() - t0) < args.secs:
                vbus = ina_bus_volts(bus)
                vsh = ina_shunt_volts(bus)
                bus_samples.append(vbus)
                shunt_samples.append(vsh)

                err_mv = (vbus - args.ref) * 1000.0
                err_pct = 100.0 * (vbus - args.ref) / args.ref
                line = (f"{t:7.2f} {vbus:11.4f} {err_mv:9.1f} "
                        f"{err_pct:8.3f} {vsh * 1e6:11.1f}")

                if not args.no_ads:
                    _, vads = ads_read(bus, args.ads_channel, args.ads_fs)
                    vnode = vads * args.divider
                    ads_node_samples.append(vnode)
                    line += (f" {vads:9.5f} {vnode:12.4f} "
                             f"{(vnode - args.ref) * 1000.0:9.1f}")

                print(line)
                time.sleep(max(0.0, period - (time.time() - t0 - t)))
        except KeyboardInterrupt:
            print("\ninterrupted")

        # --------------------------------------------------------------------
        # Summary
        # --------------------------------------------------------------------
        def mean(xs):
            return sum(xs) / len(xs) if xs else float("nan")

        def spread(xs):
            return (max(xs) - min(xs)) if xs else float("nan")

        vb, vs = mean(bus_samples), mean(shunt_samples)
        print("\n--- summary ---")
        print(f"n samples            : {len(bus_samples)}")
        print(f"INA bus mean         : {vb:.4f} V   "
              f"(err {(vb - args.ref) * 1000:+.1f} mV, "
              f"{100 * (vb - args.ref) / args.ref:+.3f} %)")
        print(f"INA bus peak-to-peak : {spread(bus_samples) * 1000:.2f} mV")
        print(f"INA shunt mean       : {vs * 1e6:+.1f} uV   "
              f"(peak-to-peak {spread(shunt_samples) * 1e6:.1f} uV)")

        if args.shunt_mohm:
            r = args.shunt_mohm / 1000.0
            print(f"  -> zero-current offset at {args.shunt_mohm} mohm shunt: "
                  f"{vs / r:+.3f} A")

        if ads_node_samples:
            va = mean(ads_node_samples)
            print(f"ADS node mean        : {va:.4f} V   "
                  f"(err {(va - args.ref) * 1000:+.1f} mV)")
            print(f"INA vs ADS delta     : {(vb - va) * 1000:+.1f} mV")
            print(f"implied true ratio   : {args.ref / (va / args.divider):.4f}   "
                  f"(nominal {args.divider:.4f})")

        # --------------------------------------------------------------------
        # Flags
        # --------------------------------------------------------------------
        print("\n--- flags ---")
        clean = True
        if abs(vb - args.ref) > 0.020:
            print("  ! bus voltage is >20 mV from the reference -- check the VBUS "
                  "connection and that the multimeter is on the same node")
            clean = False
        if abs(vs) > 500e-6:
            print(f"  ! shunt reads {vs * 1e6:+.0f} uV with no current flowing. "
                  "IN- is probably floating, or something is loading the shunt. "
                  "Tie IN- to the same node as IN+ (or confirm the breakout has "
                  "an onboard shunt bridging them) and re-run.")
            clean = False
        elif abs(vs) > 50e-6:
            print(f"  ~ shunt offset {vs * 1e6:+.0f} uV is larger than the INA226 "
                  "typical (+/-10 uV) but usable -- record it and subtract it in "
                  "the logging script.")
        if ads_node_samples and abs(vb - mean(ads_node_samples)) > 0.050:
            print("  ! INA and ADS disagree by >50 mV -- most likely the divider "
                  "ratio differs from nominal (resistor tolerance). Use the "
                  "'implied true ratio' above rather than 15.706.")
            clean = False
        if clean:
            print("  all checks passed")
        print()

    return 0


if __name__ == "__main__":
    sys.exit(main())
