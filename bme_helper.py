"""
bme_helper.py  --  092226

BME280 enclosure monitoring for the winch drone: humidity, pressure,
temperature, and dew point. The point of this is LEAK DETECTION -- water got
into the enclosure once already and was only found after the fact.

Same shape as ina_helper.py, and for the same reasons:
  - it must NEVER raise into the caller's loop
  - it rate-limits internally, so read() is cheap to call from a fast loop
  - it can share an already-open SMBus (the ADS1115 and INA226 are on bus 1)
  - 092226: after repeated I2C errors it drops the device and then RETRIES,
    rather than giving up permanently. The INA226 helper originally gave up
    for good, and a single transient glitch silently ended monitoring for a
    whole session with no way back short of restarting the flight script.

Bus map: 0x44 INA226, 0x48 ADS1115, 0x76 BME280. No collision.

WHY DEW POINT. Relative humidity is temperature-dependent, so an enclosure
warming in the sun shows RH FALLING while the actual water content is
unchanged -- and a real leak can hide inside that. Dew point is computed from
T and RH and is very nearly invariant with temperature, so it rises only when
water actually enters. For leak detection it is the trustworthy channel; RH is
the familiar one. Both are reported.

BMP280 vs BME280: boards are routinely sold as BME280 when they are BMP280,
which has NO humidity sensor. The chip ID tells them apart (0x60 vs 0x58).
On a BMP280 this reports has_humidity False and leaves humidity and dew point
as None rather than returning a fabricated number.
"""

import math
import struct
import time

try:
    from smbus2 import SMBus
    _SMBUS_OK = True
except Exception:
    _SMBUS_OK = False

REG_CHIP_ID = 0xD0
REG_CTRL_HUM = 0xF2
REG_CTRL_MEAS = 0xF4
REG_CONFIG = 0xF5
REG_DATA = 0xF7
REG_CALIB1 = 0x88
REG_CALIB2 = 0xE1

CHIP_BME280 = 0x60
CHIP_BMP280 = (0x56, 0x57, 0x58)

_OSRS = {1: 1, 2: 2, 4: 3, 8: 4, 16: 5}
_FILT = {0: 0, 2: 1, 4: 2, 8: 3, 16: 4}


class BME280(object):

    def __init__(self, addr=0x76, busnum=1, bus=None, rate_hz=1.0,
                 osrs=16, filt=16, max_fail=5, retry_sec=60.0):
        """
        addr      0x76 default, 0x77 if SDO is pulled high
        rate_hz   maximum I2C poll rate; read() returns the cached value in
                  between. 1 Hz is ample -- an enclosure's humidity does not
                  change fast, and a leak is a trend over minutes.
        retry_sec after being dropped for I2C errors, try to bring it back
                  this often. 0 restores permanent-drop behaviour.
        """
        self.addr = addr
        self.period = 1.0 / float(rate_hz) if rate_hz > 0 else 0.0
        self.osrs = osrs
        self.filt = filt
        self.max_fail = max_fail
        self.retry_sec = float(retry_sec)

        self.available = False
        self.has_humidity = False
        self.chip_id = None
        self.temp = None            # deg C
        self.pressure = None        # hPa
        self.humidity = None        # % RH, None on a BMP280
        self.dewpoint = None        # deg C, None on a BMP280
        self.last_poll = 0.0
        self.next_retry = 0.0
        self.fails = 0
        self.drops = 0
        self.recoveries = 0
        self.err = ""
        self._cal = {}

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
        except Exception as e:
            self.err = "init: %s" % e
            self.available = False

    # ------------------------------------------------------------------
    def _rd(self, reg, n=1):
        return self.bus.read_i2c_block_data(self.addr, reg, n)

    def _wr(self, reg, val):
        self.bus.write_byte_data(self.addr, reg, val)

    def _configure(self):
        cid = self._rd(REG_CHIP_ID)[0]
        if cid != CHIP_BME280 and cid not in CHIP_BMP280:
            raise RuntimeError("0x%02X chip_id=0x%02X, not a BME280/BMP280"
                               % (self.addr, cid))
        self.chip_id = cid
        self.has_humidity = (cid == CHIP_BME280)
        self._read_calibration()
        if self.has_humidity:
            self._wr(REG_CTRL_HUM, _OSRS[self.osrs])
        self._wr(REG_CONFIG, (_FILT[self.filt] << 2))
        # normal mode, continuous
        self._wr(REG_CTRL_MEAS,
                 (_OSRS[self.osrs] << 5) | (_OSRS[self.osrs] << 2) | 0b11)
        time.sleep(0.1)

    def _read_calibration(self):
        # NOTE: calibration registers are LITTLE-endian while the measurement
        # registers are BIG-endian. Mixing them up yields plausible nonsense
        # rather than an error.
        c = {}
        b1 = bytes(self._rd(REG_CALIB1, 26))
        (c["T1"], c["T2"], c["T3"], c["P1"], c["P2"], c["P3"], c["P4"],
         c["P5"], c["P6"], c["P7"], c["P8"],
         c["P9"]) = struct.unpack("<HhhHhhhhhhhh", b1[0:24])
        c["H1"] = b1[25]
        if self.has_humidity:
            b2 = bytes(self._rd(REG_CALIB2, 7))
            c["H2"] = struct.unpack("<h", b2[0:2])[0]
            c["H3"] = b2[2]
            e4, e5, e6 = b2[3], b2[4], b2[5]
            # H4/H5 are 12-bit signed, packed across three bytes that SHARE
            # the nibbles of e5
            h4 = (e4 << 4) | (e5 & 0x0F)
            h5 = (e6 << 4) | (e5 >> 4)
            c["H4"] = h4 - 4096 if h4 > 2047 else h4
            c["H5"] = h5 - 4096 if h5 > 2047 else h5
            c["H6"] = struct.unpack("<b", b2[6:7])[0]
        self._cal = c
        # all-zero or all-0xFF means the reads are not landing at all
        vals = list(c.values())
        if vals and (all(v == 0 for v in vals)
                     or all(v in (65535, 255, -1) for v in vals)):
            raise RuntimeError("calibration reads look invalid (%s)"
                               % ("all zero" if all(v == 0 for v in vals)
                                  else "all 0xFF"))

    # ---- Bosch compensation, floating point form ---------------------
    def _compensate(self, adc_t, adc_p, adc_h):
        c = self._cal
        v1 = (adc_t / 16384.0 - c["T1"] / 1024.0) * c["T2"]
        v2 = ((adc_t / 131072.0 - c["T1"] / 8192.0) ** 2) * c["T3"]
        t_fine = v1 + v2
        temp = t_fine / 5120.0

        v1 = t_fine / 2.0 - 64000.0
        v2 = v1 * v1 * c["P6"] / 32768.0
        v2 = v2 + v1 * c["P5"] * 2.0
        v2 = v2 / 4.0 + c["P4"] * 65536.0
        v1 = (c["P3"] * v1 * v1 / 524288.0 + c["P2"] * v1) / 524288.0
        v1 = (1.0 + v1 / 32768.0) * c["P1"]
        if v1 == 0:
            pres = None
        else:
            p = 1048576.0 - adc_p
            p = (p - v2 / 4096.0) * 6250.0 / v1
            v1 = c["P9"] * p * p / 2147483648.0
            v2 = p * c["P8"] / 32768.0
            pres = (p + (v1 + v2 + c["P7"]) / 16.0) / 100.0   # hPa

        hum = None
        if adc_h is not None:
            h = t_fine - 76800.0
            h = (adc_h - (c["H4"] * 64.0 + c["H5"] / 16384.0 * h)) * (
                c["H2"] / 65536.0 * (1.0 + c["H6"] / 67108864.0 * h
                                     * (1.0 + c["H3"] / 67108864.0 * h)))
            h = h * (1.0 - c["H1"] * h / 524288.0)
            hum = max(0.0, min(100.0, h))
        return temp, pres, hum

    @staticmethod
    def dew_point(temp_c, rh_pct):
        """Magnus-Tetens. Valid roughly 0-60 C, which covers an enclosure."""
        if temp_c is None or rh_pct is None or rh_pct <= 0:
            return None
        a, b = 17.27, 237.7
        g = (a * temp_c / (b + temp_c)) + math.log(rh_pct / 100.0)
        return (b * g) / (a - g)

    # ------------------------------------------------------------------
    def read(self, force=False):
        """Return (temp_C, pressure_hPa, humidity_pct, dewpoint_C).
        Any element may be None; all four are None when unavailable."""
        now = time.time()

        if not self.available:
            if (self.retry_sec <= 0 or self.next_retry == 0.0
                    or now < self.next_retry):
                return (None, None, None, None)
            self.next_retry = now + self.retry_sec
            try:
                self._configure()
            except Exception as e:
                self.err = "retry failed: %s" % e
                return (None, None, None, None)
            self.available = True
            self.fails = 0
            self.recoveries += 1
            self.err = ""

        if not force and (now - self.last_poll) < self.period:
            return (self.temp, self.pressure, self.humidity, self.dewpoint)
        self.last_poll = now

        try:
            n = 8 if self.has_humidity else 6
            d = self._rd(REG_DATA, n)
            adc_p = (d[0] << 12) | (d[1] << 4) | (d[2] >> 4)
            adc_t = (d[3] << 12) | (d[4] << 4) | (d[5] >> 4)
            adc_h = ((d[6] << 8) | d[7]) if self.has_humidity else None
            temp, pres, hum = self._compensate(adc_t, adc_p, adc_h)
        except Exception as e:
            self.fails += 1
            self.err = "read: %s" % e
            if self.fails >= self.max_fail:
                self.available = False
                self.drops += 1
                self.temp = self.pressure = None
                self.humidity = self.dewpoint = None
                self.next_retry = (now + self.retry_sec
                                   if self.retry_sec > 0 else 0.0)
                self.err = ("dropped after %d consecutive I2C errors: %s"
                            % (self.fails, e))
            return (self.temp, self.pressure, self.humidity, self.dewpoint)

        self.fails = 0
        self.temp = temp
        self.pressure = pres
        self.humidity = hum
        self.dewpoint = self.dew_point(temp, hum)
        return (self.temp, self.pressure, self.humidity, self.dewpoint)

    def close(self):
        if self._own_bus and self.bus is not None:
            try:
                self.bus.close()
            except Exception:
                pass
        self.bus = None
        self.available = False
