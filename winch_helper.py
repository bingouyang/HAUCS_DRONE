import threading, queue, time, logging, sys
import traceback
from builtins import range
import math
from dataclasses import dataclass

import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero import Servo

# hall effect settings (calibrate for every system)
HALL_MIN = 1035
HALL_MAX = 12285

# winch_worker.py
import time, math, queue, logging, threading, traceback

# Hardware libs
from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero import Servo
import board, busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
'''
winch_parms={
    'SERVO_PIN': 17,
    'RELEASE_VAL': -0.4,                  # -ROTATION * 0.4
    'RETRACT_VAL': 0.30,                  # gentle retract
    'NEUTRAL_VAL': 0,                     # neutral position
    'ROTATION':      -1,                  # ROTATION = -1  (CW/CCW)
    'HALL_MIN': 1035,                     # need to be calibrated
    'HALL_MAX': 12285,                    # need to be calibrated
    'HALL_TARGET': 1035,                  # "retracted" target near hall_min
    'PWR_LIMIT': 0.30,                    # cap servo power while retracting
    'SAFETY_RELEASE_SEC': 0.7,            # your safety timer
    'RETRACT_SETTLE_CM': 50,              # threshold around target 
    'MID_RETRACT_GAT': 8000,              # flip RETRACTED=0 beyond this gap
}

# global variables
winch_st={
    'CYCLE_COUNT': 0,
    'CYCLE_LIMIT': 0,
    'RETRACTED': 1,
    'AUTO_STATE': "idle",
    'AUTO_DROP' :15,  # drop time for auto cycling
    'AUTO_PWR' :1.0,  # retrieve power
    'ROTATION' :-1,  # CW, -1 CCW
}
'''

def neutral(servo, cfg):
    servo.value = cfg["neutral_val"]

def hall_raw(chan):
    return int(chan.value)

def release_once(servo, chan, cfg, st, stop_evt):
    servo.value = -cfg["rotation_sign"] * abs(cfg["release_val"])
    t0 = time.time()
    while not stop_evt.is_set():
        if (time.time() - t0) > cfg["safety_release_sec"]:
            neutral(servo, cfg)
            break
        dist = hall_raw(chan) - cfg["hall_target"]
        if dist > cfg["mid_retract_gate"]:
            break
        time.sleep(0.02)
    neutral(servo, cfg)

def retract_feedback_step(servo, chan, cfg, st, stop_evt):
    """
    Pull inward toward hall_target with power limiting & easing.
    Returns True only once when we *reach* retracted.
    """
    try:
        dist = hall_raw() - cfg["hall_target"]
    except Exception:
        neutral()
        time.sleep(0.25)
        return False

    pwr = dist / float(cfg["hall_max"] - cfg["hall_min"])
    pwr = max(0.0, min(cfg["pwr_limit"], pwr * cfg["pwr_limit"]))
    if pwr > 0.0:
        pwr = math.pow(pwr, 1.0/3.0)

    # Close enough to target → mark retracted
    if dist < (cfg["hall_target"] + cfg["retract_settle_cm"]):
        if RETRACTED != 1:
            RETRACTED = 1
            # stop inward motion if currently driving inward
            if (cfg["rotation_sign"] * (servo.value or 0.0)) > 0.0:
                neutral()
            return True
        # already retracted: ensure neutral if still pushing inward
        if (cfg["rotation_sign"] * (servo.value or 0.0)) > 0.0:
            neutral()
    elif (dist < 10_000) and ((cfg["rotation_sign"] * (servo.value or 0.0)) > 0.0):
        # keep pulling inward with bounded power
        set_servo(cfg["rotation_sign"] * pwr)
    elif dist > cfg["mid_retract_gate"]:
        if RETRACTED != 0:
            RETRACTED = 0
    return False

def winch_thread(stop_evt: threading.Event, q_winch, ):
    # --- servo (gpiozero + pigpio backend) ---
    try:
        servo = Servo(winchParms["SERVO_PIN"], pin_factory=PiGPIOFactory())
        servo.value = winchParms["neutral_val"]
    except Exception as e:
        print(f"Servo init failed: {e}")
        return

    # --- ADS1115 (Adafruit) ---
    try:
        i2c = busio.I2C(board.SCL, board.SDA)
        ads = ADS.ADS1115(i2c)
        chan = AnalogIn(ads, ADS.P0)  # A0
    except Exception as e:
        print(f"ADS1115 init failed: {e}")
        return

    # --- internal state (no globals) ---
    RETRACTED = 1              # 1 near target, 0 extended
    AUTO_STATE = "idle"        # "idle" | "released" | "retrieving" | "retracted"
    CYCLE_COUNT = 0
    CYCLE_LIMIT = 0
    AUTO_DROP = 15.0
    AUTO_PWR = 1.0
    drop_timer = time.time()
    retrieve_timer = time.time()

    def hall_raw() -> int:
        return int(chan.value)

    def set_servo(val: float):
        # clamp to [-1, +1]
        servo.value = max(-1.0, min(1.0, float(val)))

    def neutral():
        set_servo(winchParms["neutral_val"])

    def release_once():
        nonlocal RETRACTED
        set_servo(-winchParms["rotation_sign"] * abs(winchParms["release_val"]))
        t0 = time.time()
        while not stop_evt.is_set():
            if (time.time() - t0) > cfg["safety_release_sec"]:
                neutral()
                print(f"{logp}safety timer during release")
                break
            dist = hall_raw() - cfg["hall_target"]
            if dist > cfg["mid_retract_gate"]:
                RETRACTED = 0
                break
            time.sleep(0.02)
        neutral()

    print(f"{logp}ready")

    try:
        while not stop_evt.is_set():
            # 1) dequeue command (non-blocking-ish)
            try:
                cmd = q_winch.get(timeout=0.05)
            except queue.Empty:
                cmd = None

            if cmd:
                act = (cmd.get("action") or "").upper()
                if act == "RELEASE":
                    print(f"{logp}RELEASE")
                    release_once()
                    AUTO_STATE = "released"
                    drop_timer = time.time()

                elif act == "RETRACT":
                    dur = float(cmd.get("duration", 0.0))
                    print(f"{logp}RETRACT for {dur:.2f}s")
                    t0 = time.time()
                    set_servo(cfg["rotation_sign"] * cfg["servo_retract_val"] * AUTO_PWR)
                    while not stop_evt.is_set() and (time.time() - t0) < dur:
                        if retract_feedback_step():
                            break
                        time.sleep(0.02)
                    neutral()

                elif act == "NEUTRAL":
                    neutral()

                elif act == "AUTO_START":
                    CYCLE_COUNT = 0
                    CYCLE_LIMIT = int(cmd.get("cycles", 1))
                    AUTO_DROP   = float(cmd.get("drop_sec", 15.0))
                    AUTO_PWR    = float(cmd.get("pwr", 1.0))
                    print(f"{logp}AUTO_START cycles={CYCLE_LIMIT} drop={AUTO_DROP:.1f}s pwr={AUTO_PWR:.2f}")
                    AUTO_STATE = "retracted"  # next loop: do a release

                elif act == "AUTO_STOP":
                    AUTO_STATE = "idle"
                    print(f"{logp}AUTO_STOP")

                q_winch.task_done()

            # 2) auto state machine (optional; runs even without commands)
            if AUTO_STATE == "idle":
                pass

            elif AUTO_STATE == "retracted":
                # kick a release
                release_once()
                drop_timer = time.time()
                AUTO_STATE = "released"

            elif AUTO_STATE == "released":
                if (time.time() - drop_timer) >= AUTO_DROP:
                    # start retrieving: push inward & let feedback finish it
                    set_servo(cfg["rotation_sign"] * cfg["servo_retract_val"] * AUTO_PWR)
                    retrieve_timer = time.time()
                    AUTO_STATE = "retrieving"

            elif AUTO_STATE == "retrieving":
                if retract_feedback_step():
                    CYCLE_COUNT += 1
                    print(f"{logp}finished cycle {CYCLE_COUNT} of {CYCLE_LIMIT}")
                    neutral()
                    time.sleep(0.2)
                    AUTO_STATE = "idle" if CYCLE_COUNT >= CYCLE_LIMIT else "retracted"
                elif (time.time() - retrieve_timer) > 35.0:
                    print(f"{logp}retrieve timeout")
                    neutral()
                    AUTO_STATE = "idle"

            time.sleep(0.01)

    except Exception:
        print(f"{logp}crash:\n{traceback.format_exc()}")
    finally:
        try:
            neutral()
            time.sleep(0.2)
        finally:
            # gpiozero cleans itself up; pigpio daemon stays up for reuse
            pass

def release():
    servo.value = -ROTATION * 0.4
    safety_timer = time.time()
    # TODO: Check BLE connection to determine location of probe
    while RETRACTED == 1:
        if (time.time() - safety_timer) > 0.7:
            servo.value = 0
            print("safety timer triggered")
            logger.warning("safety timer on release")
            break
        pass
    servo.value = 0


def winch_control(stop_evt, q_winch):
    """
    Low-level winch control. Sets motor power and retract state.
    """
    # Initialize Servo
    servo = Servo(17, pin_factory=PiGPIOFactory())
    servo.value = 0
    # Initialize ADC
    # Create the I2C bus
    i2c = busio.I2C(board.SCL, board.SDA)
    adc = ADS.ADS1115(i2c)
    adc.gain=1
    dist = adc.readADC(0)
    #global RETRACTED
    
    target_dist = HALL_MIN
    pwr_limit = 0.3
    while not stop_evt.is_set():
        try:
            dist = adc.readADC(0) - target_dist
        except:
            servo.value = 0
            logger.warning("failed to read hall effect")
            time.sleep(5)

        pwr = dist / (HALL_MAX - HALL_MIN)
        pwr = pwr * pwr_limit
        if pwr > 0:
            pwr = math.pow(pwr, 1 / 3)

        # limit min/max power
        if pwr > pwr_limit:
            pwr = pwr_limit
        elif pwr < 0:
            pwr = 0

        if dist < (target_dist + 50):
            RETRACTED = 1
            if (ROTATION * servo.value) > 0.0:
                servo.value = 0
        elif (dist < 10_000) and ((ROTATION * servo.value) > 0.0):
            servo.value = ROTATION * pwr
        elif dist > 8_000:
            if RETRACTED != 0:
                RETRACTED = 0

def state_machine(stop_evt, q_winch):
    """
    High-level winch state machine.
    """
    #global RETRACTED, CYCLE_COUNT, CYCLE_LIMIT
    #global AUTO_STATE, AUTO_PWR
    drop_timer = time.time()
    retrieve_timer = time.time()
    while not stop_evt.is_set():
        if AUTO_STATE == "idle":
            pass
        elif AUTO_STATE == "released":
            if AUTO_DROP < (time.time() - drop_timer):
                if RETRACTED == 0:
                    AUTO_STATE = "retrieving"
                    print("retrieve started")
                    logger.info("retrieve started")
                    servo.value = ROTATION * AUTO_PWR
                    retrieve_timer = time.time()
                else:
                    print("release failed")
                    logger.warning("release failed")
                    AUTO_STATE = "idle"

        elif AUTO_STATE == "retrieving":
            if RETRACTED:
                AUTO_STATE = "retracted"
                CYCLE_COUNT += 1
                print(f"finished cycle {CYCLE_COUNT} of {CYCLE_LIMIT}")
                logger.info("cylce finished: %s", CYCLE_COUNT)
                if CYCLE_COUNT >= CYCLE_LIMIT:
                    AUTO_STATE = "idle"
                    print(f"program ended at {CYCLE_COUNT} cycles")
                    logger.info("program ended at %s cycles", CYCLE_COUNT)
                time.sleep(10)
            elif 35 < (time.time() - retrieve_timer):
                servo.value = 0
                AUTO_STATE = "idle"
                logger.warning("ran out of time to retrieve")
                print("ran out of time to retrieve")

        elif AUTO_STATE == "retracted":
            release()
            drop_timer = time.time()
            AUTO_STATE = "released"
'''
wcontrol = threading.Thread(target=winch_control)
smachine = threading.Thread(target=state_machine)
wcontrol.start()
smachine.start()
while True:
    # handle user inputs
    cmd = input()
    cmd = cmd.split(",")
    if len(cmd) < 1:
        continue
    elif (len(cmd) == 2) and (cmd[0] == "s"):
        print("setting servo to: ", float(cmd[1]))
        servo.value = float(cmd[1])
    elif cmd[0] == "q":
        print("stopping")
        AUTO_STATE = "idle"
        servo.value = 0
    elif cmd[0] == "r":
        print("releasing")
        release()
    elif cmd[0] == "p":
        print(f"servo pwr: {servo.value}")
        print(f"hall efct: {adc.readADC(0)}")
        print(f"    state: {AUTO_STATE}")
        print(f"retracted: {'yes' if RETRACTED == 1 else 'no'}")
    elif cmd[0] == "start":
        CYCLE_COUNT = 0
        CYCLE_LIMIT = 1
        if len(cmd) == 2:
            CYCLE_LIMIT = int(cmd[1])
        print(f"running test for {CYCLE_LIMIT} cycles")
        AUTO_STATE = "retracted"
'''