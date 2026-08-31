"""
ina_helper.py  --  083026

INA226 rail monitoring for the winch drone companion computer.

Design constraints for this file:
  - It must NEVER raise into the caller's loop. Every I2C access is guarded,
    and after N consecutive failures the object disables itself permanently
    and returns (None, None) forever. A bad I2C connection must not take the
    winch down the way the unguarded ZeroDivisionError took down the DO sensor.
  - It rate-limits internally, so read() can be called every loop iteration
    regardless of how fast that loop runs.
  - It can share an already-open SMBus (the ADS1115 Hall sensor is on the same
    bus 1) or open its own.

Typical use:

    from ina_helper import INA226
    ina = INA226(shunt_ohm=0.009091)      # R100 || R010
    ...
    v, a = ina.read()                     # (None, None) if unavailable
    if v is not None:
        ...

Addresses: INA226 default 0x44 here (0x40-0x4F range). The ADS1115 at 0x48
does not collide.
"""

import time

try:
    from smbus2 import SMBus
    _SMBUS_OK = True
except Exception:
    _SMBUS_OK = False

_CONFIG = 0x00
_SHUNT = 0x01
_BUS = 0x02
_MFG_ID = 0xFE
_DIE_ID = 0xFF

_BUS_LSB_V = 1.25e-3      # 1.25 mV/LSB
_SHUNT_LSB_V = 2.5e-6     # 2.5 uV/LSB, +/-81.92 mV full scale

_AVG = {1: 0b000, 4: 0b001, 16: 0b010, 64: 0b011,
        128: 0b100, 256: 0b101, 512: 0b110, 1024: 0b111}


class INA226(object):

    def __init__(self, shunt_ohm=0.009091, addr=0x44, busnum=1, bus=None,
                 avg=16, rate_hz=5.0, max_fail=5, tare=True):
        """
        shunt_ohm : shunt resistance in ohms (0.009091 = R100 || R010)
        bus       : an existing SMBus to share, or None to open busnum
        avg       : INA226 hardware averaging; 16 -> 35 ms conversion
        rate_hz   : maximum I2C poll rate; read() returns the cached value
                    between polls
        max_fail  : consecutive I2C errors before this object gives up
        tare      : measure and subtract the zero-current shunt offset at
                    construction. Only valid if nothing is drawing current
                    yet -- on the winch that means before the servo is armed.
        """
        self.shunt_ohm = float(shunt_ohm)
        self.addr = addr
        self.avg = avg
        self.period = 1.0 / float(rate_hz) if rate_hz > 0 else 0.0
        self.max_fail = max_fail

        self.available = False
        self.offset_v = 0.0
        self.volts = None
        self.amps = None
        self.watts = None
        self.peak_a = 0.0
        self.peak_w = 0.0
        self.last_poll = 0.0
        self.fails = 0
        self.err = ""

        self._own_bus = False
        self.bus = bus

        if not _SMBUS_OK:
            self.err = "smbus2 not installed"
            return

        try:
            if self.bus is None:
                self.bus = SMBus(busnum)
                self._own_bus = True
            self._configure()
            self.available = True
            if tare:
                self.tare()
        except Exception as e:
            self.err = "init: %s" % e
            self.available = False

    # ------------------------------------------------------------------
    def _rd16(self, reg):
        d = self.bus.read_i2c_block_data(self.addr, reg, 2)
        return (d[0] << 8) | d[1]

    def _wr16(self, reg, val):
        self.bus.write_i2c_block_data(self.addr, reg,
                                      [(val >> 8) & 0xFF, val & 0xFF])

    @staticmethod
    def _signed(v):
        return v - 65536 if v & 0x8000 else v

    def _configure(self):
        mfg = self._rd16(_MFG_ID)
        die = self._rd16(_DIE_ID)
        if not (mfg == 0x5449 and die == 0x2260):
            raise RuntimeError("0x%02X mfg=0x%04X die=0x%04X, not an INA226"
                               % (self.addr, mfg, die))
        # VBUSCT = VSHCT = 1.1 ms, MODE = shunt+bus continuous
        cfg = (0x4000 | (_AVG[self.avg] << 9) | (0b100 << 6)
               | (0b100 << 3) | 0b111)
        self._wr16(_CONFIG, cfg)
        time.sleep(0.005)
        back = self._rd16(_CONFIG)
        if back != cfg:
            raise RuntimeError("config readback 0x%04X != 0x%04X" % (back, cfg))
        time.sleep(self.avg * 2 * 1.1e-3 * 3)

    # ------------------------------------------------------------------
    def tare(self, n=32):
        """Re-measure the zero-current shunt offset. Caller is responsible for
        making sure no current is flowing. Safe to call any time; on failure
        it leaves the previous offset in place."""
        if not self.available:
            return None
        try:
            acc = 0.0
            for _ in range(n):
                acc += self._signed(self._rd16(_SHUNT)) * _SHUNT_LSB_V
                time.sleep(0.004)
            self.offset_v = acc / n
            return self.offset_v
        except Exception as e:
            self.err = "tare: %s" % e
            return None

    def reset_peaks(self):
        self.peak_a = 0.0
        self.peak_w = 0.0

    # ------------------------------------------------------------------
    def read(self, force=False):
        """Return (volts, amps), or (None, None) if unavailable.

        Rate-limited internally: between polls the previous values are
        returned, so this is cheap to call from a fast loop.
        """
        if not self.available:
            return (None, None)

        now = time.time()
        if not force and (now - self.last_poll) < self.period:
            return (self.volts, self.amps)
        self.last_poll = now

        try:
            vbus = (self._rd16(_BUS) & 0x7FFF) * _BUS_LSB_V
            vsh = self._signed(self._rd16(_SHUNT)) * _SHUNT_LSB_V - self.offset_v
        except Exception as e:
            self.fails += 1
            self.err = "read: %s" % e
            if self.fails >= self.max_fail:
                self.available = False
                self.volts = self.amps = self.watts = None
            return (self.volts, self.amps)

        self.fails = 0
        self.volts = vbus
        self.amps = vsh / self.shunt_ohm
        self.watts = vbus * self.amps
        if self.amps > self.peak_a:
            self.peak_a = self.amps
        if self.watts > self.peak_w:
            self.peak_w = self.watts
        return (self.volts, self.amps)

    # ------------------------------------------------------------------
    def close(self):
        if self._own_bus and self.bus is not None:
            try:
                self.bus.close()
            except Exception:
                pass
        self.bus = None
        self.available = False
