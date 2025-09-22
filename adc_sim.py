import time, threading, random, math

class ServoSim:
    """Minimal servo stand-in (mimics gpiozero.Servo.value)."""
    def __init__(self, name="ServoSim"):
        self.name = name
        self._value = 0.0  # -1..+1
        self._lock = threading.Lock()
    @property
    def value(self):
        with self._lock:
            return self._value
    @value.setter
    def value(self, v):
        v = max(-1.0, min(1.0, float(v)))
        with self._lock:
            self._value = v

class LinkedHallADC:
    """
    Hall-ADC simulator.
    - Servo release  -> moves toward extended_val
    - Servo Neutral  -> holds position
    - Servo retract  -> moves toward retracted_val
    - Supports neutral offset (creep) and deadband around that offset
    - Separate speeds for release vs retract
    """
    def __init__(self,
                 servo,
                 retracted_val: int,
                 extended_val: int,
                 rotation_direction: int = -1,
                 speed_release: float = 8000.0,   # units/s at |cmd|=1 for outward
                 speed_retract: float = 4000.0,   # units/s at |cmd|=1 for inward
                 rate_hz: float = 20.0,
                 noise: int = 0,
                 start_at: str = "retracted",     # "retracted" or "extended"
                 neutral_center: float = 0.0,     # servo's real neutral offset (e.g., +0.03)
                 neutral_deadband: float = 0.03   # +/- around neutral_center treated as zero
                 ):
        self.servo = servo
        self.retracted_val = int(retracted_val)
        self.extended_val  = int(extended_val)
        self.rotation_direction = 1 if rotation_direction >= 0 else -1
        self.speed_release = float(speed_release)
        self.speed_retract = float(speed_retract)
        self.rate_hz = max(1.0, float(rate_hz))
        self.noise = int(noise)
        self.neutral_center = float(neutral_center)
        self.deadband = max(0.0, float(neutral_deadband))

        self.lo = min(self.retracted_val, self.extended_val)
        self.hi = max(self.retracted_val, self.extended_val)

        self._lock = threading.Lock()
        self._value = int(self.retracted_val if start_at == "retracted" else self.extended_val)

        self._stop_evt = threading.Event()
        self._thr = threading.Thread(target=self._loop, name="HallSim", daemon=True)
        self._thr.start()

    # --- public reads ---
    @property
    def value(self) -> int:
        with self._lock:
            return self._value

    def read(self) -> int:
        with self._lock:
            return self._value

    # --- internal write ---
    def _set_value(self, v: float) -> None:
        with self._lock:
            self._value = int(max(self.lo, min(self.hi, v)))

    def stop(self):
        self._stop_evt.set()
        self._thr.join(timeout=1.0)

    # --- helper: deadbanded, centered command in [-1..+1] with zero in deadband ---
    def _effective_cmd(self, raw_cmd: float) -> float:
        # shift by neutral center (creep compensation)
        c = float(raw_cmd) - self.neutral_center
        # deadband
        if abs(c) <= self.deadband:
            return 0.0
        # re-scale so that beyond deadband maps smoothly toward full scale
        span = 1.0 - self.deadband
        c = (abs(c) - self.deadband) / span * (1.0 if c >= 0 else -1.0)

        return max(-1.0, min(1.0, c))

    def _loop(self):
        dt = 1.0 / self.rate_hz
        while not self._stop_evt.is_set():
            raw = self.servo.value or 0.0
            cmd = self._effective_cmd(raw)  # centered & deadbanded

            if cmd == 0.0:
                # True neutral hold: no motion, no noise
                time.sleep(dt)
                continue

            inward = (self.rotation_direction * cmd) > 0.0
            target = self.retracted_val if inward else self.extended_val
            speed  = abs(cmd) * (self.speed_retract if inward else self.speed_release)
            step   = speed * dt

            old = self.value
            diff = target - old
            if diff != 0.0 and step > 0.0:
                move = math.copysign(min(abs(diff), step), diff)
                new  = old + move
            else:
                new  = old

            # add noise but avoiding neutral drift
            if self.noise and new != old:
                new += random.randint(-self.noise, self.noise)

            self._set_value(new)
            time.sleep(dt)

'''
class LinkedHallADC:
    def __init__(self,
                 servo,
                 retracted_val: int,
                 extended_val: int,
                 rotation_direction: int = -1,
                 speed_release: float = 8000.0,   # units/s at |servo|=1.0
                 speed_retract: float = 4000.0,   # units/s at |servo|=1.0
                 rate_hz: float = 20.0,
                 noise: int = 0,
                 start_at: str = "retracted"):
        self.servo = servo
        self.retracted_val = int(retracted_val)
        self.extended_val  = int(extended_val)
        self.rotation_direction = 1 if rotation_direction >= 0 else -1
        self.speed_release = float(speed_release)
        self.speed_retract = float(speed_retract)
        self.rate_hz = max(1.0, float(rate_hz))
        self.noise = int(noise)

        self.lo = min(self.retracted_val, self.extended_val)
        self.hi = max(self.retracted_val, self.extended_val)

        # thread-safe storage for the reading
        self._lock = threading.Lock()
        self._value = int(self.retracted_val if start_at == "retracted" else self.extended_val)

        self._stop_evt = threading.Event()
        self._thr = threading.Thread(target=self._loop, name="HallSim", daemon=True)
        self._thr.start()

    # --- public read API ---
    @property
    def value(self) -> int:
        with self._lock:
            return self._value

    def read(self) -> int:
        """Same as .value, explicit method form."""
        with self._lock:
            return self._value

    # --- internal write helper ---
    def _set_value(self, v: float) -> None:
        with self._lock:
            self._value = int(v)

    def stop(self):
        self._stop_evt.set()
        self._thr.join(timeout=1.0)

    # --- simulation loop ---
    def _loop(self):
        dt = 1.0 / self.rate_hz
        while not self._stop_evt.is_set():
            cmd = self.servo.value or 0.0  # -1..+1
            inward = (self.rotation_direction * cmd) > 0.0
            target = self.retracted_val if inward else self.extended_val
            speed  = abs(cmd) * (self.speed_retract if inward else self.speed_release)
            step   = speed * dt

            old = self.value
            diff = target - old
            if diff != 0 and step > 0.0:
                move = math.copysign(min(abs(diff), step), diff)
                new  = old + move
            else:
                new  = old

            # clamp + noise
            if self.noise:
                new = max(self.lo, min(self.hi, new + random.randint(-self.noise, self.noise)))
            else:
                new = max(self.lo, min(self.hi, new))
            print (f"adc value in adc sim loop: {new}")
            self._set_value(new)
            time.sleep(dt)
'''
