"""
adc_sim_faults.py - fault-injecting Hall simulator for main_rc8_uart_*.py.

Drop-in replacement for LinkedHallADC when adc_sim_flag == 1. It behaves
exactly like the normal simulator most of the time, then randomly injects a
fault per cast so the REAL code paths run: release_win() exhausts its latch
pulses, retract_adaptive() times out, the ADC except branches fire, and the
matching log lines and GCS messages are produced.

Unlike fault_sim.py, which emits status codes directly to test the display,
this drives the flight code itself.

Faults, drawn per cast:

    latch_stuck   the outward command produces no motion at all, so both
                  latch pulses fail and release_win() reports "latch did not
                  open". This is the fault that damaged three servos.
    slow_latch    outward motion starts late, so the first pulse fails and
                  the second succeeds. Exercises the retry path.
    retract_jam   inward motion stops partway, so retract runs to
                  RETRACT_SEC and logs the timeout.
    sensor_fault  reads raise, exercising the try/except in
                  retract_adaptive() and release_win().
    sensor_stuck  reads freeze at the last value. Looks like NO-MOTION to
                  winch_audit.py without an exception being raised.

It also MEASURES the stress, which is the point. Every 1/rate_hz slice where
a command is applied but the mechanism cannot move is counted as stall time,
and converted to wear on the same model as fault_sim.py:

    wear += (torque / rated) ** 3 * seconds

A stalled servo pushes to its rating, so a stall contributes 1.0 per second
while a normal 0.5 lb latch pull contributes 3.6e-07 per second. That ratio
is why only the latch direction wore out.

Usage in the flight script, replacing the LinkedHallADC(...) call:

    from adc_sim_faults import FaultyHallADC
    ...
    adc = FaultyHallADC(
        servo=servo,
        retracted_val=1035,
        extended_val=12285,
        rotation_direction=cfg["ROTATION_DIRECTION"],
        speed_release=400000, speed_retract=4000,
        rate_hz=20, noise=5, start_at="retracted",
        fault_rate=0.35,          # fraction of casts that get a fault
        seed=None,                # set an int for a repeatable run
        logger=logger,            # optional; falls back to print
    )

Choosing what goes wrong:

    fault_rate=0.0                          never fault
    fault_rate=1.0                          every cast, type drawn at random
    fault_rate=1.0, fault="latch_stuck"     every cast, always that fault
    fault_rate=0.5, fault="latch_stuck"     half the casts, always that fault
    sequence=[None, "latch_stuck",
              None, "retract_jam"]          fixed repeating cycle; overrides
                                            the other two, None = clean cast

    seed=<int>                              repeatable random runs

    ...and at shutdown:
        logger.info(adc.summary())

083026: rail simulation. FaultyHallADC now also carries a SimINA226 that
produces bus voltage and current matching what the servo is actually doing,
including the stall spikes the faults above create. It implements the same
interface as ina_helper.INA226, so the flight code can hold either one:

    adc = FaultyHallADC(..., ina_sim=True)
    ina = adc.ina                      # instead of INA226(...)
    v, a = ina.read()

Set ina_sim=False (the default) if you are not testing the current path.
"""

import math
import random
import threading
import time

from adc_sim import LinkedHallADC

# Same wear model as fault_sim.py. See its header for the derivation.
LB_N = 0.453592 * 9.81
LATCH_FORCE_N = 0.5 * LB_N
LEVER_M = 0.010
RATED_OZIN = 444.0
WEAR_EXP = 3.0
FAIL_UNITS = 180.0

FRAC_LATCH = (LATCH_FORCE_N * LEVER_M * 141.6) / RATED_OZIN
FRAC_STALL = 1.0

# ---------------------------------------------------------------------------
# 083026: rail / current model.
#
# Numbers are anchored to the 083026 bench log (servo_power_log.csv), taken on
# the known-bad servo through breadboard wiring:
#
#   pulses stopped            0.000 A     bus 7.32 V
#   running, cmd -0.62        0.570 A     bus 7.11 V
#   V-vs-I fit over 107 loaded samples -> 0.81 ohm series resistance
#
# That 0.81 ohm is the breadboard, not the design. R_SERIES defaults to a
# properly wired 0.10 ohm; pass r_series=0.81 to reproduce the bench log.
#
# I_RUN is the free-running draw at saturating command. The bench servo drew
# 0.57 A; a healthy unit is taken as roughly a quarter of that.
V_NOM = 7.40           # MP1584 output setpoint, no load
R_SERIES = 0.10        # ohm, wiring + converter output impedance
I_IDLE = 0.0           # pulses stopped: servo draws nothing
I_HOLD = 0.05          # commanded neutral, healthy unit. A damaged unit was
                       # measured at ~1.6 A / 12 W in exactly this state.
I_RUN_GOOD = 0.15      # free-running, healthy
I_RUN_WORN = 0.60      # free-running, worn/degraded (bench log)
CMD_FULL = 0.35        # |cmd| at which running current saturates
I_STALL = 3.5          # demand when the mechanism cannot move
I_LIMIT = 3.0          # MP1584 rating; above this it goes constant-current
                       # and the rail collapses
I_NOISE = 0.004        # A rms, matches the observed +/-4.6 mA at avg=4
INRUSH_MULT = 2.0      # transient multiplier at command onset
INRUSH_S = 0.15        # how long that transient lasts


class SimINA226(object):
    """Drop-in stand-in for ina_helper.INA226, driven by the servo command and
    the blocked/not-blocked state the fault simulator already computes.

    update() is called from the ADC's own thread at rate_hz, so peaks are
    captured at full rate even though read() is rate-limited the same way the
    real helper is. That mirrors the real part, where the hardware keeps
    converting between polls.
    """

    def __init__(self, v_nom=V_NOM, r_series=R_SERIES, i_run=I_RUN_GOOD,
                 i_hold=I_HOLD, i_stall=I_STALL, i_limit=I_LIMIT,
                 noise=I_NOISE, rate_hz=5.0, fail_after=None, rng=None):
        """
        i_run      free-running draw at saturating command
        fail_after seconds after which read() starts failing, to exercise the
                   helper's max_fail / available=False path. None = never.
        """
        self.v_nom = float(v_nom)
        self.r_series = float(r_series)
        self.i_run = float(i_run)
        self.i_hold = float(i_hold)
        self.i_stall = float(i_stall)
        self.i_limit = float(i_limit)
        self.noise = float(noise)
        self.period = 1.0 / float(rate_hz) if rate_hz > 0 else 0.0
        self.fail_after = fail_after
        self._rng = rng or random.Random()

        self.available = True
        self.shunt_ohm = 0.009091     # R100 || R010, for interface parity
        self.offset_v = 0.0
        self.volts = self.v_nom
        self.amps = 0.0
        self.watts = 0.0
        self.peak_a = 0.0
        self.peak_w = 0.0
        self.last_poll = 0.0
        self.fails = 0
        self.err = ""
        self._t0 = time.time()
        self._cmd = 0.0
        self._blocked = False
        self._cmd_since = 0.0
        self._limiting = False
        self._stall_a_max = 0.0

    # -- driven by the simulator -------------------------------------------
    def update(self, cmd, blocked, dt):
        """cmd is the servo value (None or 0.0 means no drive)."""
        now = time.time()
        if cmd is None:
            cmd = 0.0
        if (cmd == 0.0) != (self._cmd == 0.0):
            self._cmd_since = now
        self._cmd = cmd
        self._blocked = blocked

        mag = abs(cmd)
        if mag == 0.0:
            demand = I_IDLE
        elif blocked:
            demand = self.i_stall
        elif mag < 1e-6:
            demand = self.i_hold
        else:
            demand = self.i_hold + (self.i_run - self.i_hold) * min(
                1.0, mag / CMD_FULL)

        if demand > 0.0 and (now - self._cmd_since) < INRUSH_S:
            demand *= INRUSH_MULT

        # MP1584 constant-current limit: current clamps, rail collapses
        self._limiting = demand > self.i_limit
        if self._limiting:
            amps = self.i_limit
            v_open = self.v_nom * (self.i_limit / demand)
        else:
            amps = demand
            v_open = self.v_nom

        amps += self._rng.gauss(0.0, self.noise)
        volts = v_open - amps * self.r_series

        self.amps = amps
        self.volts = volts
        self.watts = volts * amps
        if amps > self.peak_a:
            self.peak_a = amps
        if self.watts > self.peak_w:
            self.peak_w = self.watts
        if blocked and amps > self._stall_a_max:
            self._stall_a_max = amps

    # -- ina_helper.INA226 interface ---------------------------------------
    def read(self, force=False):
        if self.fail_after is not None and (time.time() - self._t0) > self.fail_after:
            self.fails += 1
            self.err = "simulated I2C failure"
            self.available = False
            return (None, None)
        if not self.available:
            return (None, None)
        now = time.time()
        if not force and (now - self.last_poll) < self.period:
            return (self.volts, self.amps)
        self.last_poll = now
        return (self.volts, self.amps)

    def tare(self, n=32):
        self.offset_v = 0.0
        return 0.0

    def reset_peaks(self):
        self.peak_a = 0.0
        self.peak_w = 0.0

    def close(self):
        self.available = False

    def summary(self):
        return ("ina sim: peak %.2f A / %.1f W, worst stall %.2f A, "
                "converter %s"
                % (self.peak_a, self.peak_w, self._stall_a_max,
                   "HIT ITS %.1f A LIMIT" % self.i_limit if self._limiting
                   or self._stall_a_max >= self.i_limit - 1e-6
                   else "within limit"))


# Relative likelihood once a cast has been selected for a fault.
FAULT_NAMES = ("latch_stuck", "slow_latch", "retract_jam",
               "sensor_fault", "sensor_stuck")

FAULT_MIX = [
    ("latch_stuck",  40),
    ("slow_latch",   25),
    ("retract_jam",  15),
    ("sensor_fault", 10),
    ("sensor_stuck", 10),
]


class FaultyHallADC(LinkedHallADC):

    def __init__(self, *args, **kw):
        # Pull our own arguments out before the parent sees them. The parent
        # starts its thread at the end of __init__, and that thread calls
        # _effective_cmd immediately, so everything it touches must exist
        # before super().__init__() runs.
        # Three ways to choose what goes wrong:
        #   fault_rate  0.0 = never, 1.0 = every cast; type drawn from FAULT_MIX
        #   fault       force one type on every faulty cast, e.g. "latch_stuck"
        #   sequence    cycle a fixed list, e.g. [None,"latch_stuck",None,...]
        #               None in the list means a clean cast. Overrides the
        #               other two, and repeats once exhausted.
        self.fault_rate = float(kw.pop("fault_rate", 0.35))
        self.forced = kw.pop("fault", None)
        self.sequence = kw.pop("sequence", None)
        self._seq_i = 0
        if self.forced is not None and self.forced not in FAULT_NAMES:
            raise ValueError("unknown fault %r; known: %s"
                             % (self.forced, sorted(FAULT_NAMES)))
        if self.sequence:
            bad = [f for f in self.sequence
                   if f is not None and f not in FAULT_NAMES]
            if bad:
                raise ValueError("unknown fault(s) %s; known: %s"
                                 % (bad, sorted(FAULT_NAMES)))
        self.slow_delay = float(kw.pop("slow_delay", 0.40))
        self.jam_at = kw.pop("jam_at", None)
        # release_win() pulses the latch with the servo neutral between
        # attempts, so an outward command alone does not mean a new cast.
        # Anything within this gap of the previous outward command is a retry.
        self.cast_gap = float(kw.pop("cast_gap", 1.0))
        self._log_fn = kw.pop("logger", None)
        seed = kw.pop("seed", None)
        self._rng = random.Random(seed)

        self._fault = None
        self._cast_n = 0
        self._was_outward = False
        self._last_outward = 0.0
        self._fault_t0 = 0.0
        self._stall_s = 0.0
        self._drive_s = 0.0
        self._wear = 0.0
        self._counts = {}
        self._flock = threading.Lock()

        # 083026: the parent starts its thread at the end of __init__ and that
        # thread calls _effective_cmd immediately, so self.ina must exist first.
        self.ina_sim = bool(kw.pop("ina_sim", False))
        self.ina = None
        if self.ina_sim:
            self.ina = SimINA226(
                v_nom=kw.pop("ina_v_nom", V_NOM),
                r_series=kw.pop("ina_r_series", R_SERIES),
                i_run=kw.pop("ina_i_run",
                             I_RUN_WORN if kw.pop("ina_worn", False)
                             else I_RUN_GOOD),
                i_stall=kw.pop("ina_i_stall", I_STALL),
                i_limit=kw.pop("ina_i_limit", I_LIMIT),
                fail_after=kw.pop("ina_fail_after", None),
                rng=self._rng)
        else:
            for k in ("ina_v_nom", "ina_r_series", "ina_i_run", "ina_worn",
                      "ina_i_stall", "ina_i_limit", "ina_fail_after"):
                kw.pop(k, None)

        super().__init__(*args, **kw)

        if self.jam_at is None:
            # stop the retract roughly two thirds of the way home
            self.jam_at = self.retracted_val + int(
                0.35 * (self.extended_val - self.retracted_val))
        if self.sequence:
            how = "sequence=%s" % (list(self.sequence),)
        elif self.forced:
            how = "always %s at rate=%.2f" % (self.forced, self.fault_rate)
        else:
            how = "random from FAULT_MIX at rate=%.2f" % self.fault_rate
        self._log("fault sim armed: %s, seed=%s, jam_at=%d"
                  % (how, seed, self.jam_at))
        if self.ina is not None:                                   # 083026
            self._log("ina sim armed: run=%.2f A, stall=%.2f A, "
                      "limit=%.2f A, r_series=%.2f ohm"
                      % (self.ina.i_run, self.ina.i_stall,
                         self.ina.i_limit, self.ina.r_series))

    # ---- logging ----------------------------------------------------------
    def _log(self, msg):
        text = "[faultsim] " + msg
        if self._log_fn is not None:
            try:
                self._log_fn.info(text)
                return
            except Exception:
                pass
        print(text)

    # ---- fault selection --------------------------------------------------
    def _draw(self):
        if self.sequence:
            f = self.sequence[self._seq_i % len(self.sequence)]
            self._seq_i += 1
            return f
        if self._rng.random() >= self.fault_rate:
            return None
        if self.forced is not None:
            return self.forced
        total = sum(w for _, w in FAULT_MIX)
        r = self._rng.uniform(0, total)
        acc = 0
        for name, w in FAULT_MIX:
            acc += w
            if r <= acc:
                return name
        return None

    # ---- the injection point ---------------------------------------------
    def _effective_cmd(self, raw_cmd):
        """Called by the parent's loop once per slice. Returning 0.0 means the
        mechanism does not move, which is exactly what a stuck latch or a
        jammed spool looks like to the flight code."""
        c = super()._effective_cmd(raw_cmd)
        now = time.time()
        dt = 1.0 / self.rate_hz

        outward = (self.rotation_direction * c) < 0.0
        inward = (self.rotation_direction * c) > 0.0

        # A new cast starts on the first outward command after a real gap.
        # A retry pulse arrives within cast_gap and must not re-roll the fault,
        # or a stuck latch would silently "fix" itself on the second attempt.
        if outward and not self._was_outward:
            if (now - self._last_outward) > self.cast_gap:
                with self._flock:
                    self._cast_n += 1
                    self._fault = self._draw()
                    self._fault_t0 = now
                    if self._fault:
                        self._counts[self._fault] = self._counts.get(self._fault, 0) + 1
                if hasattr(self, "_stuck_at"):
                    del self._stuck_at
                self._log("cast %d: %s" % (self._cast_n, self._fault or "no fault"))
            else:
                self._log("  retry pulse (fault %s still active)"
                          % (self._fault or "none"))
        if outward:
            self._last_outward = now
        self._was_outward = outward

        blocked = False
        if self._fault == "latch_stuck" and outward:
            blocked = True
        elif (self._fault == "slow_latch" and outward
              and (now - self._fault_t0) < self.slow_delay):
            # blocks long enough to defeat the first pulse; the retry succeeds
            blocked = True
        elif self._fault == "retract_jam" and inward and self.value <= self.jam_at:
            blocked = True

        # Account for stress. A command with no resulting motion is a stall;
        # a command that does move the mechanism is the ordinary latch load.
        if c != 0.0:
            with self._flock:
                self._drive_s += dt
                if blocked:
                    self._stall_s += dt
                    self._wear += (FRAC_STALL ** WEAR_EXP) * dt
                else:
                    self._wear += (FRAC_LATCH ** WEAR_EXP) * dt

        # 083026: feed the rail model. Done for every slice, including the
        # ones with no command, so idle current and the return to idle after a
        # stall both appear on the trace.
        if self.ina is not None:
            self.ina.update(c, blocked, dt)

        return 0.0 if blocked else c

    # ---- reads, so sensor faults reach the caller but not our own thread ---
    @property
    def value(self):
        mine = threading.current_thread() is getattr(self, "_thr", None)
        if not mine:
            if self._fault == "sensor_fault":
                raise IOError("faultsim: simulated ADS1115 read failure")
            if self._fault == "sensor_stuck":
                return getattr(self, "_stuck_at", LinkedHallADC.value.fget(self))
        v = LinkedHallADC.value.fget(self)
        if self._fault == "sensor_stuck" and not hasattr(self, "_stuck_at"):
            self._stuck_at = v
        return v

    def read(self):
        return self.value

    # ---- reporting --------------------------------------------------------
    def stats(self):
        with self._flock:
            return {
                "casts": self._cast_n,
                "faults": dict(self._counts),
                "drive_s": round(self._drive_s, 2),
                "stall_s": round(self._stall_s, 2),
                "wear": round(self._wear, 4),
                "wear_pct": round(100.0 * self._wear / FAIL_UNITS, 2),
                "peak_a": round(self.ina.peak_a, 3) if self.ina else None,
                "peak_w": round(self.ina.peak_w, 2) if self.ina else None,
            }

    def summary(self):
        s = self.stats()
        ina_txt = ""
        if self.ina is not None:                                   # 083026
            ina_txt = " | " + self.ina.summary()
        parts = ", ".join("%s=%d" % kv for kv in sorted(s["faults"].items()))
        return ("faultsim: %d casts, %.1fs driven of which %.1fs STALLED, "
                "wear %.3f units (%.2f%% of modelled servo life)%s"
                % (s["casts"], s["drive_s"], s["stall_s"], s["wear"],
                   s["wear_pct"], (" [" + parts + "]") if parts else "")
                + ina_txt)
