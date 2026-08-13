from pymavlink import mavutil
import threading
import queue
import time
import logging
import sys
import traceback
from builtins import range
import math
import signal
import os
import csv
from datetime import datetime
from threading import Lock as QMutex

import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero import Servo

from winch_helper import *
from encoder_helper import *
from bt_helper import *

from adc_sim import ServoSim, LinkedHallADC

# simulator flags
data_sim_flag = False
adc_sim_flag = 0

COPTER_MODES = {
    0: "STABILIZE",
    1: "ACRO",
    2: "ALT_HOLD",
    3: "AUTO",
    4: "GUIDED",
    5: "LOITER",
    6: "RTL",
    7: "CIRCLE",
    9: "LAND",
    11: "DRIFT",
    13: "SPORT",
    14: "FLIP",
    15: "AUTOTUNE",
    16: "POSHOLD",
    17: "BRAKE",
    18: "THROW",
    19: "AVOID_ADSB",
    20: "GUIDED_NOGPS",
    21: "SMART_RTL",
    22: "FLOWHOLD",
    23: "FOLLOW",
    24: "ZIGZAG",
    25: "SYSTEMID",
    26: "AUTOROTATE",
    27: "AUTO_RTL",
}

# ---------------- CONFIG ----------------
# Primary flight controller link: Pi to Cube Orange over UART
FC_CONN_STR = "/dev/serial0"
FC_BAUD = 115200

# Optional forwarding to PC or GCS
# Set to None to disable
GCS_UDP_OUT = None

# Mavlink constants
NAV_IN_AIR = mavutil.mavlink.MAV_LANDED_STATE_IN_AIR
NAV_ON_GROUND = mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND
NAV_LANDING = mavutil.mavlink.MAV_LANDED_STATE_LANDING
NAV_SAFETY_ARMED = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
PING_SEC = 2
TOUCH_CONFIRM_SEC = 2

wParms = {
    "SERVO_PIN": 17,
    "ADC_PIN": 0,
    "HALL_MIN": 2500,
    "HALL_MAX": 12285,
    # 081226: measured stop. Under continuous retract the reading pins at
    # 2662 (min 2644, max 2682, stdev 6.7) and drifts +17 counts over 58s,
    # i.e. it is hard against the stop, not creeping in. The old settle line
    # of 2550 sat below that floor, so retract could never succeed and every
    # cycle timed out and blocked deploy. Settle line is now 2800, 118 clear
    # of the observed max 2682. RETRACT_SETTLE is left at 50 on purpose - it
    # is also the band for both polarity checks, so widening it there would
    # have quietly changed two unrelated safeguards.
    "HALL_TARGET": 2750,
    "RETRACT_PWR": 0.1,
    "RELEASE_PWR": -0.40,
    "NEUTRAL_POS": 0.0,
    "ROTATION_DIRECTION": -1,
    "RELEASE_SEC": 20,      # 071426: motor drives payload down; overridden by SCR_USER1
    "RETRACT_SETTLE": 50,
    # 081226: no-progress detector. If the line jams, the old code drove into
    # it for the full RETRACT_SEC (60s in the 21:43 log). Stop after 5s of no
    # movement instead: close to target means the goal is met, far from it
    # means something is stuck and grinding will not help.
    "RETRACT_STALL_SEC": 5.0,     # seconds of no progress before deciding
    "RETRACT_PROGRESS": 30,       # counts of inward movement that count as progress
    "RETRACT_STALL_CLOSE": 500,   # within this of target, a stall means "done"
    "RETRACT_TH": 8000,
    "PWD_ADP_TH": 10000,
    "LOG_PREFIX": "[WINCH] ",
    "PAUSE_SEC": 2,         # 071426: idle at bottom before retract; overridden by SCR_USER2
    "RETRACT_SEC": 35,      # 071426: motor drives payload up; overridden by SCR_USER3
}

# BLE parms
# The sensor boots in "auto" mode and this code never switches it -- it starts/stops
# sampling itself off its own pressure sensor. A BLE "manual start" command isn't
# reliable right at splashdown (the link can be marginal or momentarily lost at exactly
# that instant), so there's no sample_type toggling here at all anymore.
BLE_SAMPLE_SIZE = 512
BLE_SAMPLE_RATE = 2
BLE_PMODE = "high"
BLE_FETCH_TIMEOUT = 5  # seconds of BLE inactivity during sample download

ble_st = {
    "ble": None,
    "ble_mutex": QMutex(),
    "last_cols": None,
    "gcs_ready": False,
    "c_status": "init",
}

# Buffer json
BUFFER_PATH = "outbox.json"
csv_path = "input.csv"

# Directory where each sampling operation's data is cached as its own CSV file.
CACHE_DIR = "cache"

# sensor data upload status and sequence id
sensor_upload_failed = {}
sensor_state = {"seq": 0}

timestamp = datetime.now().strftime("%y%m%d_%H%M")

logging.basicConfig(
    format="%(asctime)s %(levelname)s: %(message)s",
    filename=f"logs/cc_{timestamp}.log",
    encoding="utf-8",
    level=logging.INFO,
)
logger = logging.getLogger(__name__)
logger.info("Starting")

# Mavlink Status
mv_state = {
    "last_landed": None,
    "og_count": 0,
    "seen_init": False,
    "armed": None,
    "awaiting_final_td": False,
    "ground_ready": True,
    "td_fired": False,
    "landing_fired": False,
    # None until the first flight-controller HEARTBEAT is processed.
    "auto_mode": None,
    "mode_name": None,
}

# Winch status
winch_st = {
    "RETRACTED": 1,
}

# FSM triggered by servo
fsm_st = {
    "cycle_active": False,
    "cycle_activated": False,
    "cycle_deactivated": False,
    "deploy_needed": False,
    "deploy_allowed": True,
    "send_to_gcs": False,
    "deploy_in_progress": False,
    # Keep independent last-PWM values for manual RC input and mission servo output.
    # This prevents low RC input messages from canceling a high SERVO_OUTPUT_RAW command.
    "_last_pwm": None,
    "_last_pwm_rc": None,
    "_last_pwm_servo": None,
    "_last_rel_t": 0.0,
    "_last_ret_t": 0.0,
}

# Trigger detection
# RC input channel to monitor from MAVLink RC_CHANNELS.
# This is 1-based: 8 means RC8 / chan8_raw.
TRIGGER_RC_CH = 8
PWM_RELEASE = 1800
PWM_RETRACT = 1200
DEBOUNCE_S = 0.1

# Set True to log every trigger-channel PWM update with a wall-clock timestamp and the
# gap since the previous update of the same source (RC vs SERVO). Useful for tracking
# down "switch flip to detection" lag: if the gap between consecutive prints is much
# larger than the RC/telemetry link's actual update rate, the delay is upstream of this
# code (link backlog or FC stream-rate config), not in this script's processing.
DEBUG_TRIGGER_TIMING = False

def trigger_pwm_value(msg, ch):
    # Return PWM in microseconds for 1-based trigger channel ch.
    # RC_CHANNELS is manual RC input: chan1_raw ... chan18_raw.
    # SERVO_OUTPUT_RAW is autopilot output from mission DO_SET_SERVO: servo1_raw ... servo16_raw.
    msg_type = msg.get_type()

    if msg_type == "RC_CHANNELS":
        if ch < 1 or ch > 18:
            return None
        return getattr(msg, "chan%d_raw" % ch, None)

    if msg_type == "SERVO_OUTPUT_RAW":
        if ch < 1 or ch > 16:
            return None
        return getattr(msg, "servo%d_raw" % ch, None)

    return None

def fsm_consume_flags(fsm):
    keys = ("deploy_needed", "send_to_gcs", "cycle_activated", "cycle_deactivated")
    out = {k: fsm.get(k, False) for k in keys}
    for k in keys:
        fsm[k] = False
    return out

def handle_trigger_pwm(
    msg,
    fsm,
    ch=TRIGGER_RC_CH,
    thresh_high=PWM_RELEASE,
    thresh_low=PWM_RETRACT,
    debounce_s=DEBOUNCE_S,
):
    msg_type = msg.get_type()
    pwm = trigger_pwm_value(msg, ch)

    if pwm is None:
        logger.info("Trigger channel %d is not present in %s" % (ch, msg_type))
        return

    if pwm == 0 or pwm == 65535:
        return

    if pwm > thresh_high:
        state = "HIGH"
    elif pwm < thresh_low:
        state = "LOW"
    else:
        state = "MID"

    if msg_type == "SERVO_OUTPUT_RAW":
        last_key = "_last_pwm_servo"
        src = "SERVO"
    else:
        last_key = "_last_pwm_rc"
        src = "RC"

    now_m = time.monotonic()
    last_pwm = fsm.get(last_key)

    if DEBUG_TRIGGER_TIMING:
        dbg_key = "_dbg_last_call_" + src
        prev_call = fsm.get(dbg_key)
        gap = (now_m - prev_call) if prev_call is not None else 0.0
        logger.info(
            "[TRIG-DEBUG] src=%s ch=%d pwm=%s wall=%.3f gap_since_last_%s_msg=%.3fs"
            % (src, ch, pwm, time.time(), src, gap)
        )
        fsm[dbg_key] = now_m

    if last_pwm is not None:
        if last_pwm < thresh_high <= pwm and (now_m - fsm["_last_rel_t"]) > debounce_s:
            fsm["_last_rel_t"] = now_m
            if not fsm["cycle_active"]:
                fsm["cycle_active"] = True
                fsm["cycle_activated"] = True
            fsm["deploy_needed"] = True
            logger.info("%s%d rising edge detected: %s -> %s (%s)" % (src, ch, last_pwm, pwm, state))

        if last_pwm > thresh_low >= pwm and (now_m - fsm["_last_ret_t"]) > debounce_s:
            fsm["_last_ret_t"] = now_m
            if fsm["cycle_active"]:
                fsm["cycle_active"] = False
                fsm["cycle_deactivated"] = True
            fsm["send_to_gcs"] = True
            logger.info("%s%d falling edge detected: %s -> %s (%s)" % (src, ch, last_pwm, pwm, state))

    fsm[last_key] = pwm
    fsm["_last_pwm"] = pwm

# Backward-compatible name used by older code.
def handle_rc_channels(msg, fsm, ch=TRIGGER_RC_CH, thresh_high=PWM_RELEASE, thresh_low=PWM_RETRACT, debounce_s=DEBOUNCE_S):
    handle_trigger_pwm(msg, fsm, ch, thresh_high, thresh_low, debounce_s)

def process_heartbeat(msg, st):
    base_mode = getattr(msg, "base_mode", 0)
    custom_mode_id = msg.custom_mode
    custom_mode_name = COPTER_MODES.get(custom_mode_id, "UNKNOWN(%s)" % custom_mode_id)
    armed = bool(base_mode & NAV_SAFETY_ARMED)

    if st["armed"] is None:
        st["armed"] = armed
    elif st["armed"] and not armed:
        st["seen_init"] = False
        st["og_count"] = 0
        st["last_landed"] = NAV_ON_GROUND
        st["awaiting_final_td"] = False
        st["ground_ready"] = True
    elif (not st["armed"]) and armed:
        st["ground_ready"] = True

    st["armed"] = armed

    # CHANGED: Use mission SERVO_OUTPUT_RAW only in actual AUTO mode.
    # Every other flight mode uses manual RC_CHANNELS input.
    st["mode_name"] = custom_mode_name
    st["auto_mode"] = (custom_mode_name == "AUTO")

def process_statustext(txt, st):
    if not txt:
        return
    s = txt.lower()
    if "mission" in s and "complete" in s:
        st["awaiting_final_td"] = True

def hall_raw(c):
    if adc_sim_flag == 1:
        return int(c.read())
    else:
        return int(c.value)

def release_win(servo, adc, cfg, st, stop_evt):
    # 071426: Hall sensor check removed from early-exit logic. Mechanism is
    # direct-drive descent at low power (RELEASE_PWR) for RELEASE_SEC duration.
    # However, Hall sensor is still used as a polarity/direction safeguard:
    #   - Pre-check: if sensor reads high before release, polarity may be reversed
    #   - Mid-check: if sensor goes DOWN during release, motor is running backwards

    # --- Pre-release Hall sensor sanity check ---
    RECOVERY_SEC = 10.0
    try:
        hall_start = hall_median(adc)
        if hall_start > cfg["RETRACT_TH"]:
            # Line still out. This is recoverable: pull it in, then release.
            # Aborting used to strand the winch, since nothing else retracts it,
            # so every later cast hit the same check and the session produced
            # flat surface-only profiles.
            logger.info("Hall reads %d before release (expected near %d). Line "
                        "appears still out - attempting recovery retract, up to %.0fs."
                        % (hall_start, cfg["HALL_TARGET"], RECOVERY_SEC))

            retract_adaptive._hall_start = None      # fresh polarity baseline
            st["RETRACTED"] = 0                      # it is out, record that
            rvalue = cfg["ROTATION_DIRECTION"] * cfg["RETRACT_PWR"] + cfg["NEUTRAL_POS"]  # 081226: sign was inverted; recovery retract paid line out
            servo.value = float(rvalue)

            t_rec = time.time()
            recovered = False
            while not stop_evt.is_set() and (time.time() - t_rec) < RECOVERY_SEC:
                if retract_adaptive(servo, adc, cfg, st):
                    recovered = True
                    break
                time.sleep(0.02)

            neutral(servo, cfg)
            hall_start = hall_median(adc)

            if not recovered and hall_start > cfg["RETRACT_TH"]:
                logger.info("SAFETY ABORT: recovery retract did not clear in %.0fs "
                            "(hall still %d). Aborting release."
                            % (RECOVERY_SEC, hall_start))
                # Leave RETRACTED at 0 so the next cycle retries recovery
                # instead of assuming the line is already in.
                st["RETRACTED"] = 0
                st["last_abort"] = "recovery_failed"
                return

            logger.info("Recovery retract OK: hall now %d, proceeding with release."
                        % hall_start)
        else:
            logger.info("Hall pre-check OK: %d (threshold %d)"
                        % (hall_start, cfg["RETRACT_TH"]))
    except Exception as e:
        logger.info("Hall pre-check failed: %s - proceeding with release" % e)
        hall_start = None

    svalue = cfg["ROTATION_DIRECTION"] * (cfg["RELEASE_PWR"]) + cfg["NEUTRAL_POS"]
    t0 = time.time()
    logger.info("Release, winch servo open: %s, duration: %ss" % (svalue, cfg["RELEASE_SEC"]))
    servo.value = float(svalue)

    while not stop_evt.is_set():
        if (time.time() - t0) > cfg["RELEASE_SEC"]:
            logger.info("Release complete: RELEASE_SEC=%ss reached" % cfg["RELEASE_SEC"])
            break

        # --- Mid-release Hall sensor direction check ---
        # If sensor reading is decreasing, motor is running backwards (polarity reversed)
        if hall_start is not None and (time.time() - t0) > 1.0:
            try:
                hall_now = hall_raw(adc)
                if hall_now < (hall_start - cfg["RETRACT_SETTLE"]):
                    logger.info("SAFETY ABORT: Hall sensor decreased during release "
                                "(%d -> %d). Motor running backwards - possible "
                                "polarity reversal. Aborting release." %
                                (hall_start, hall_now))
                    neutral(servo, cfg)
                    return
            except Exception as e:
                logger.info("Hall mid-check failed: %s" % e)

        time.sleep(0.1)

    st["RETRACTED"] = 0  # mark as extended after descent completes
    neutral(servo, cfg)

def hall_median(adc, n=3, gap=0.02):
    """
    Median of n reads. The pre-check acts on one sample, so a single bad read
    would either abort a good cast or trigger recovery that is not needed.
    n=3 rejects a lone outlier without masking real motion.
    """
    vals = []
    for i in range(n):
        try:
            vals.append(hall_raw(adc))
        except Exception:
            pass
        if i < n - 1:
            time.sleep(gap)
    if not vals:
        raise RuntimeError("no hall readings")
    vals.sort()
    return vals[len(vals) // 2]


def retract_adaptive(servo, adc, cfg, st):
    try:
        hall_now = hall_raw(adc)
        retract_adaptive._last_hall = hall_now   # 081226: for the stall check
        dist = hall_now - cfg["HALL_TARGET"]

        # --- Retract polarity safeguard ---
        # During retract the Hall reading should decrease toward HALL_TARGET.
        # If it is already near HALL_TARGET or below, something is wrong -
        # either polarity is reversed (motor extending instead of retracting)
        # or the sensor is reading incorrectly. Stop immediately.
        hall_start = getattr(retract_adaptive, "_hall_start", None)
        if hall_start is None:
            retract_adaptive._hall_start = hall_now
        elif hall_now > hall_start + cfg["RETRACT_SETTLE"]:
            logger.info("SAFETY ABORT: Hall sensor increased during retract "
                        "(%d -> %d). Motor running backwards - possible "
                        "polarity reversal. Aborting retract." %
                        (hall_start, hall_now))
            neutral(servo, cfg)
            retract_adaptive._hall_start = None
            return False

    except Exception:
        neutral(servo, cfg)
        time.sleep(0.25)
        return False

    pwr = dist / float(cfg["HALL_MAX"] - cfg["HALL_MIN"])
    pwr = pwr * cfg["RETRACT_PWR"]
    logger.info("currnt adaptive dist: %s" % dist)
    logger.info("currnt hall_raw val: %s" % hall_raw(adc))
    logger.info("pwr0: %s" % pwr)

    if pwr > 0.0:
        pwr = math.pow(pwr, 1.0 / 3.0)
        logger.info("pwr1: %s" % pwr)

    if pwr > cfg["RETRACT_PWR"]:
        pwr = cfg["RETRACT_PWR"]
    elif pwr < 0:
        pwr = 0

    # dist is already hall_now - HALL_TARGET, so compare it directly
    # with RETRACT_SETTLE. Do not add HALL_TARGET a second time.
    if dist < cfg["RETRACT_SETTLE"]:
        if st["RETRACTED"] != 1:
            st["RETRACTED"] = 1
            retract_adaptive._hall_start = None  # reset for next cycle
            fsm_st["deploy_allowed"] = True
            logger.info("Deploy re-enabled (fully retracted)")
            if (cfg["ROTATION_DIRECTION"] * servo.value) > 0.0:
                neutral(servo, cfg)
            return True

        if (cfg["ROTATION_DIRECTION"] * servo.value) > 0.0:
            neutral(servo, cfg)

    elif (dist < cfg["PWD_ADP_TH"]) and ((cfg["ROTATION_DIRECTION"] * servo.value) > 0.0):
        servo.value = cfg["ROTATION_DIRECTION"] * pwr + cfg["NEUTRAL_POS"]
        logger.info("currnt adaptive pwr: %s" % servo.value)

    elif dist > cfg["RETRACT_TH"]:
        if st["RETRACTED"] != 0:
            st["RETRACTED"] = 0

    return False

def neutral(servo, cfg):
    servo.value = cfg["NEUTRAL_POS"]
    logger.info('inside neutral')

def broadcast_value(x, n):
    return [] if n <= 0 else [x] * n

def winch_thread(stop_evt, q_winch, cfg, st):
    try:
        # setup servo
        if adc_sim_flag == 1:
            servo = ServoSim()  # to use servo simulator
        else:
            servo = Servo(
                cfg["SERVO_PIN"],
                min_pulse_width=0.0009,
                max_pulse_width=0.0021,
                frame_width=0.02,
                pin_factory=PiGPIOFactory(),
                initial_value=cfg["NEUTRAL_POS"],
            )
    except Exception as e:
        logger.info("Servo init failed: %s" % e)
        return

    try:
        if adc_sim_flag == 1:
            adc = LinkedHallADC(
                servo=servo,
                retracted_val=1035,
                extended_val=12285,
                rotation_direction=cfg["ROTATION_DIRECTION"],
                speed_release=400000,
                speed_retract=4000,
                rate_hz=20,
                noise=5,
                start_at="retracted",
            )
        else:
            i2c = busio.I2C(board.SCL, board.SDA)
            ads = ADS.ADS1115(i2c)
            adc = AnalogIn(ads, 0)
    except Exception as e:
        logger.info("ADS1115 init failed: %s" % e)
        return

    logger.info("winch ready")
    try:
        while not stop_evt.is_set():
            try:
                cmd = q_winch.get(timeout=0.25)
            except queue.Empty:
                cmd = None

            if cmd:
                act = (cmd.get("action") or "").upper()
                if act == "RELEASE":
                    release_win(servo, adc, cfg, st, stop_evt)

                    if stop_evt.is_set():
                        continue

                    # SCR_USER2 starts only after release_win() actually finishes.
                    pause_sec = float(cmd.get("pause", cfg["PAUSE_SEC"]))
                    retract_sec = float(cmd.get("retract_duration", cfg["RETRACT_SEC"]))
                    logger.info("Release finished. Idle at bottom for %.1f sec" % pause_sec)

                    if stop_evt.wait(pause_sec):
                        continue

                    logger.info("Bottom pause finished. Queue RETRACT with timeout %.1f sec" % retract_sec)
                    q_winch.put({"action": "RETRACT", "duration": retract_sec})

                elif act == "RETRACT":
                    dur = float(cmd.get("duration", 0.0))
                    logger.info("RETRACT for %ss" % dur)
                    t0 = time.time()
                    # Start each retract with a fresh Hall-direction baseline.
                    retract_adaptive._hall_start = None
                    servo.value = cfg["ROTATION_DIRECTION"] * cfg["RETRACT_PWR"]
                    logger.info("servo.value %s" % servo.value)
                    time.sleep(0.25)

                    best_hall = None          # 081226: lowest reading so far
                    last_progress = time.time()
                    stalled = False
                    while not stop_evt.is_set() and (time.time() - t0) < dur:
                        retract_flag = retract_adaptive(servo, adc, cfg, st)
                        if retract_flag is True:
                            logger.info("Fully retractd!")
                            break

                        h = getattr(retract_adaptive, "_last_hall", None)
                        if h is not None:
                            if best_hall is None or h < (best_hall - cfg["RETRACT_PROGRESS"]):
                                best_hall = h
                                last_progress = time.time()
                            elif (time.time() - last_progress) > cfg["RETRACT_STALL_SEC"]:
                                gap = h - cfg["HALL_TARGET"]
                                stalled = True
                                if gap < cfg["RETRACT_STALL_CLOSE"]:
                                    logger.info("Retract stalled at hall %d (%d from "
                                                "target) with no progress for %.1fs - "
                                                "close enough, treating as retracted"
                                                % (h, gap, cfg["RETRACT_STALL_SEC"]))
                                    st["RETRACTED"] = 1
                                    fsm_st["deploy_allowed"] = True
                                else:
                                    logger.info("Retract stalled at hall %d (%d from "
                                                "target) with no progress for %.1fs - "
                                                "too far out, stopping. Line may be "
                                                "jammed. Deploy stays blocked."
                                                % (h, gap, cfg["RETRACT_STALL_SEC"]))
                                break
                        time.sleep(0.1)
                    if (not stalled) and (time.time() - t0) >= dur:
                        logger.info("WARNING: Retract timeout, not fully retracted, future release prevented for now")
                    retract_adaptive._hall_start = None
                    neutral(servo, cfg)

                elif act == "NEUTRAL":
                    neutral(servo, cfg)

            time.sleep(0.1)

    except Exception as e:
        logger.info("winch thread crashed:%s" % e)
        neutral(servo, cfg)

    finally:
        neutral(servo, cfg)
        time.sleep(0.1)

def cache_sample_csv(cols, cache_dir=CACHE_DIR):
    """Write one sampling operation's data to its own timestamped CSV file."""
    try:
        os.makedirs(cache_dir, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        fname = os.path.join(cache_dir, "sample_%s.csv" % ts)
        # NOTE: "DO" and "pressure" (not "do"/"press") to match encoder_helper.py's
        # VAR_MAP/SEND_ORDER exactly -- send_payload() silently drops any column whose
        # key doesn't match one of those names, with no error. "lat"/"lon" are cached
        # here for the local CSV but are NOT in VAR_MAP, so they still won't be sent to
        # the base station (see note below where last_cols is built).
        fields = ["time", "DO", "temp", "pressure", "init_DO", "init_pressure", "batt_v", "lat", "lon"]
        rows = zip(*[cols.get(f, []) for f in fields])

        with open(fname, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(fields)
            writer.writerows(rows)

        logger.info("cached sampling data to %s" % fname)
        return fname
    except Exception as e:
        logger.info("failed to cache sampling data: %s" % e)
        return None

def ble_thread(stop_evt, q_ble, q_mav, st):
    ble = st["ble"] = BluetoothReader(st["ble_mutex"])

    try:
        st["c_status"] = "disconnected"

        while not stop_evt.is_set() and (ble is None or not ble.check_connection_status()):
            if ble.connect():
                ble.set_lights("navigation")
                logger.debug("connected to sensor, activated lights")
            ble.init_sensor_status()
            time.sleep(0.2)
            ble.set_calibration_pressure()
            time.sleep(0.2)
            ble.set_maxsize(BLE_SAMPLE_SIZE)
            time.sleep(0.2)
            ble.set_sample_hz(BLE_SAMPLE_RATE)
            time.sleep(0.2)
            logger.info(
                "BLE sensor init: max_size=%d sample_hz=%.1f"
                % (BLE_SAMPLE_SIZE, BLE_SAMPLE_RATE)
            )            
            # No sample_type call needed here -- the sensor already boots in "auto" and
            # nothing in this code ever switches it away from that anymore.
            st["c_status"] = "connected"

        last_poll = 0.0

        while not stop_evt.is_set():
            time.sleep(0.2)
            now = time.time()

            if now - last_poll >= 1:
                last_poll = now
                try:
                    ok = bool(ble.check_connection_status())
                except Exception:
                    ok = False

                if not ok:
                    st["c_status"] = "disconnected"
                    try:
                        if ble.reconnect():
                            st["c_status"] = "connected"
                    except Exception:
                        pass
                else:
                    st["c_status"] = "connected"

            try:
                cmd = q_ble.get(timeout=0.25)
                logger.info("cmd:%s" % cmd)
            except queue.Empty:
                cmd = None

            if cmd:
                action = (cmd.get("action") or "").upper()
                # Sensor stays in "auto" mode permanently now -- it starts/stops sampling
                # itself from its own pressure sensor. There's no BLE START command anymore:
                # requiring a live BLE round-trip at the exact splashdown instant was the
                # unreliable part. mav_thread just triggers the winch and records the deploy
                # location locally (see deploy_lat / deploy_lon below).
                if action == "FETCH":
                    logger.info("BLE action:FETCH: %s" % ble.sdata.get("connection"))
                    if ble.sdata.get("connection"):
                        try:

                            ble.set_sampl_flag(0)
                            time.sleep(0.1)
                            sflag = ble.get_sampl_flag()
                            logger.info("stop sampling sampling flag: %s" % sflag)
                            s_size = ble.get_sample_size()
                            ok = bool(ble.get_sample_data(BLE_FETCH_TIMEOUT))
                            st["sampling"] = False
                            logger.info(
                                "finish sampling - queue mav cmd to upload data, ok: %s, sample_size:%s"
                                % (ok, s_size)
                            )
                            do_list = ble.sdata.get("do_vals") or []
                            if ok and len(do_list) > 0:
                                temp_list = ble.sdata.get("temp_vals") or []
                                press_list = ble.sdata.get("pressure_vals") or []
                                n = len(do_list)
                                logger.info("sampling finished, length:%s" % n)
                                ts_list = list(range(n))

                                # lat/lon were locked in mav_thread at the moment the winch
                                # released (deploy rising edge), not re-read here, so this is
                                # where the sensor actually went, not where the drone drifted
                                # to by the time fetch happens.
                                # Keys here must match encoder_helper.VAR_MAP/SEND_ORDER
                                # exactly ("DO"/"pressure", not "do"/"press") --
                                # prepare_per_var_queues() does `if name not in data_cols:
                                # continue`, so a mismatched key is dropped with no error
                                # or warning at all. This was silently dropping DO and
                                # pressure from every upload.
                                # lat/lon are NOT in VAR_MAP, so they're included here for
                                # the local CSV cache only -- they still won't reach the
                                # base station until encoder_helper.py is extended to
                                # carry them (see chat for why that needs more than just
                                # adding the key -- the residue encoding as-is isn't
                                # precise enough for GPS coordinates).
                                st["last_cols"] = {
                                    "time": ts_list,
                                    "DO": do_list,
                                    "temp": temp_list,
                                    "pressure": press_list,
                                    "init_DO": broadcast_value(ble.sdata.get("init_do"), n),
                                    "init_pressure": broadcast_value(ble.sdata.get("init_pressure"), n),
                                    "batt_v": broadcast_value(ble.sdata.get("battv"), n),
                                    "lat": broadcast_value(st.get("deploy_lat"), n),
                                    "lon": broadcast_value(st.get("deploy_lon"), n),
                                }
                                cache_sample_csv(st["last_cols"])
                                st["gcs_ready"] = True
                                st["c_status"] = "fetched"
                                q_mav.put({"action": "sendpayload"})
                            else:
                                st["c_status"] = "fetch_empty"
                                logger.info("BLE fetch returned no samples; upload skipped")
                        except Exception as e:
                            st["c_status"] = "fetch_failed"
                            logger.info("fetch failed: %s" % e)
                        finally:
                            # Re-arm the buffer for the next arm. auto_sensing()
                            # on the sensor only re-fires when sample_count == 0, so this reset
                            # is what lets the next drop trigger itself again. No sample_type
                            # call needed -- it's never switched away from "auto".
                            try:
                                ble.set_sample_reset()
                                time.sleep(0.1)
                                logger.info("re-armed for next sample")
                            except Exception as e:
                                logger.info("failed to re-arm sensor for next sample: %s" % e)

                    else:
                        st["c_status"] = "fetch_skipped_disconnected"

                elif action == "DISCONNECT":
                    ble_close(ble)
                    st["c_status"] = "disconnected_by_request"

    except Exception as e:
        logger.info("BLE thread crashed: %s" % e)

    finally:
        logger.info("BLE thread close")
        ble_close(ble)


def ble_close(ble):
    try:
        ble.stop_scan()
    except Exception:
        pass

    try:
        conn = getattr(ble, "uart_connection", None)
        if conn and getattr(conn, "connected", False):
            conn.disconnect()
    except Exception:
        pass

    try:
        ble.sdata["connection"] = False
    except Exception:
        pass


# 071426: ArduPilot SCR_USER parameter mapping.
# Set these in Mission Planner Full Parameter List before each mission.
# SCR_USER1 = RELEASE_SEC  (how long motor drives payload down)
# SCR_USER2 = PAUSE_SEC    (idle time at bottom before retract)
# SCR_USER3 = RETRACT_SEC  (maximum retract duration / safety timeout)
# SCR_USER4 = shutdown flag (set to 1 in Mission Planner to cleanly shut down Pi)
#             Pi polls this each loop; triggers "sudo shutdown -h now" when == 1
# If a timing parameter is 0 or unset, the local wParms default is kept.
SCR_USER_MAP = {
    "SCR_USER1": "RELEASE_SEC",
    "SCR_USER2": "PAUSE_SEC",
    "SCR_USER3": "RETRACT_SEC",
}
SCR_USER_SHUTDOWN = "SCR_USER4"   # set to 1 in Mission Planner to trigger Pi shutdown
SCR_USER_SHUTDOWN_POLL_SEC = 5.0  # how often to poll SCR_USER4 (seconds)

def fetch_scr_user_params(m_fc, wincfg):
    # 071426: Called at startup AND right before each release so Mission Planner
    # changes take effect without restarting the script.
    for param_name, cfg_key in SCR_USER_MAP.items():
        try:
            m_fc.param_fetch_one(param_name)
            msg = m_fc.recv_match(type="PARAM_VALUE", blocking=True, timeout=3)
            if msg and msg.param_id.strip('\x00') == param_name and msg.param_value > 0:
                old = wincfg[cfg_key]
                wincfg[cfg_key] = float(msg.param_value)
                logger.info("SCR_USER: %s -> %s = %.1f (was %.1f)" % (
                    param_name, cfg_key, wincfg[cfg_key], old))
            else:
                logger.info("SCR_USER: %s not set or zero, keeping default %s=%.1f" % (
                    param_name, cfg_key, wincfg[cfg_key]))
        except Exception as e:
            logger.info("071426 SCR_USER: failed to fetch %s: %s" % (param_name, e))


def mav_thread(stop_evt, q_winch, q_ble, q_mav, wincfg, winst, blest):
    if "failed" not in sensor_state:
        sensor_state["failed"] = load_buffer(BUFFER_PATH)

    logger.info("MAVLINK: starting on FC link %s @ %s" % (FC_CONN_STR, FC_BAUD))

    try:
        m_fc = mavutil.mavlink_connection(FC_CONN_STR, baud=FC_BAUD)

        logger.info("MAVLINK: waiting for HEARTBEAT from Cube...")
        hb = m_fc.wait_heartbeat(timeout=10)

        if not hb:
            logger.info("MAVLINK: no HEARTBEAT in 10s (check UART wiring, baud, serial port)")
        else:
            logger.info(
                "MAVLINK: connected: sys=%s comp=%s"
                % (hb.get_srcSystem(), hb.get_srcComponent())
            )
            process_heartbeat(hb, mv_state)
            logger.info(
                "MAVLINK: initial flight mode=%s, trigger source=%s"
                % (
                    mv_state.get("mode_name"),
                    "SERVO_OUTPUT_RAW" if mv_state.get("auto_mode") else "RC_CHANNELS",
                )
            )

            # Explicitly request GLOBAL_POSITION_INT. Companion-computer links don't
            # always stream this by default (depends on the FC's stream-rate config for
            # this serial port), so without asking for it lat/lon just silently sit at
            # None forever. MAV_CMD_SET_MESSAGE_INTERVAL asks for it directly regardless
            # of whatever the default rates are.
            try:
                m_fc.mav.command_long_send(
                    m_fc.target_system,
                    m_fc.target_component,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                    0,
                    mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
                    200000,  # microseconds -> 5 Hz
                    0, 0, 0, 0, 0,
                )
                logger.info("MAVLINK: requested GLOBAL_POSITION_INT at 5 Hz")
            except Exception as e:
                logger.info("MAVLINK: failed to request GLOBAL_POSITION_INT: %s" % e)

            # 071426: Fetch SCR_USER timing params from FC at startup.
            fetch_scr_user_params(m_fc, wincfg)

        # CHANGED: UART-only. Send payloads back through the flight-controller link.
        payload_link = m_fc
        # 071426: Store m_fc ref so fetch_scr_user_params can be called before each release.
        mav_fc_ref = m_fc

        sensor_state["failed"] = resend_buffer(payload_link, sensor_state.get("failed", {}))

        last_lat = None
        last_lon = None
        last_ping = time.time()
        dbg_prev_iter_start = time.monotonic()

        while not stop_evt.is_set():

            if DEBUG_TRIGGER_TIMING:
                iter_start = time.monotonic()
                idle_gap = iter_start - dbg_prev_iter_start
                if idle_gap > 0.5:
                    # Time between the END of the previous iteration and the START of
                    # this one. If this is large, something AFTER recv_match in the
                    # previous pass was slow (BLE queue puts, file I/O, prints, etc.) --
                    # i.e. the bottleneck is in this script, not the MAVLink link.
                    logger.info("[LOOP-DEBUG] %.3fs elapsed since previous loop iteration finished" % idle_gap)
                dbg_prev_iter_start = iter_start

            # CHANGED: HEARTBEAT selects exactly one trigger source.
            # AUTO mode: process only SERVO_OUTPUT_RAW from mission DO_SET_SERVO.
            # Non-AUTO modes: process only RC_CHANNELS from the manual radio.
            msg = m_fc.recv_match(
                type=["HEARTBEAT", "RC_CHANNELS", "SERVO_OUTPUT_RAW", "GLOBAL_POSITION_INT"],
                blocking=True,
                timeout=2,
            )

            if DEBUG_TRIGGER_TIMING:
                recv_dt = time.monotonic() - iter_start
                if recv_dt > 0.5:
                    # Time recv_match() itself took to return. If this is large, the
                    # bottleneck is upstream: the FC isn't sending matching messages
                    # promptly, or there's a backlog on the link.
                    logger.info(
                        "[LOOP-DEBUG] recv_match blocked %.3fs, returned %s"
                        % (recv_dt, msg.get_type() if msg else None)
                    )

            flags = {
                "deploy_needed": False,
                "send_to_gcs": False,
                "cycle_activated": False,
                "cycle_deactivated": False,
            }

            if msg is None:
                logger.info("No HEARTBEAT, RC_CHANNELS, or SERVO_OUTPUT_RAW message received...")
            else:
                msg_type = msg.get_type()

                if msg_type == "HEARTBEAT":
                    # Ignore heartbeats from a GCS or another MAVLink component.
                    is_fc_heartbeat = (
                        msg.get_srcSystem() == m_fc.target_system
                        and msg.get_srcComponent() == m_fc.target_component
                    )

                    if is_fc_heartbeat:
                        previous_auto = mv_state.get("auto_mode")
                        process_heartbeat(msg, mv_state)
                        current_auto = mv_state.get("auto_mode")

                        if previous_auto != current_auto:
                            if current_auto:
                                # Establish a fresh servo-output baseline after entering AUTO.
                                fsm_st["_last_pwm_servo"] = None
                                trigger_source = "SERVO_OUTPUT_RAW"
                            else:
                                # Establish a fresh manual-RC baseline after leaving AUTO.
                                fsm_st["_last_pwm_rc"] = None
                                trigger_source = "RC_CHANNELS"

                            logger.info(
                                "MAVLINK: flight mode=%s, trigger source=%s"
                                % (mv_state.get("mode_name"), trigger_source)
                            )

                elif msg_type == "SERVO_OUTPUT_RAW":
                    if mv_state.get("auto_mode") is True:
                        handle_trigger_pwm(msg, fsm_st)
                        flags = fsm_consume_flags(fsm_st)

                elif msg_type == "RC_CHANNELS":
                    if mv_state.get("auto_mode") is False:
                        handle_trigger_pwm(msg, fsm_st)
                        flags = fsm_consume_flags(fsm_st)

                elif msg_type == "GLOBAL_POSITION_INT":
                    # Keep a running current position so it can be locked in at the exact
                    # moment of deploy below, not re-read later at fetch/upload time once
                    # the drone may have already lifted off and moved on.
                    if last_lat is None:
                        logger.info(
                            "MAVLINK: first GLOBAL_POSITION_INT received, lat=%s lon=%s"
                            % (msg.lat / 1e7, msg.lon / 1e7)
                        )
                    last_lat = msg.lat / 1e7
                    last_lon = msg.lon / 1e7

                if flags["cycle_activated"]:
                    pass

                if flags["deploy_needed"]:
                    if not fsm_st["deploy_allowed"] or winst["RETRACTED"] != 1:
                        logger.info("Deploy ignored: not allowed or not retracted")
                    else:
                        fsm_st["deploy_allowed"] = False   # lock it immediately

                        # Lock the sampling location in now, at the rising edge, while it is
                        # still fresh, not later once the drone may have already taken back
                        # off. No BLE command needed here: the sensor starts sampling itself.
                        blest["deploy_lat"] = last_lat
                        blest["deploy_lon"] = last_lon

                        logger.info(
                            "EVENT: winch release, sensor auto-starts sampling, lat=%s lon=%s"
                            % (last_lat, last_lon)
                        )
                        # 071426: Re-fetch SCR_USER params right before release so any
                        # Mission Planner changes since startup are picked up immediately.
                        fetch_scr_user_params(mav_fc_ref, wincfg)
                        q_winch.put({
                            "action": "RELEASE",
                            "pause": wincfg["PAUSE_SEC"],
                            "retract_duration": wincfg["RETRACT_SEC"],
                        })

                if flags["send_to_gcs"]:
                    logger.info(
                        "EVENT: requesting data to send to gcs, lat=%s lon=%s"
                        % (last_lat, last_lon)
                    )
                    sensor_state["failed"] = resend_buffer(payload_link, sensor_state.get("failed", {}))

                    if data_sim_flag is True:
                        cols = prep_sim_data(csv_path)
                        send_payload(payload_link, cols, sensor_state)
                    else:
                        logger.info("MAV to BLE: fetch data from BLE sensor")
                        q_ble.put({"action": "FETCH"})

                if flags["cycle_deactivated"]:
                    logger.info("EVENT: cycle_deactivated")
                    q_winch.put({"action": "NEUTRAL"})

            # 071426: Poll SCR_USER4 periodically for Pi shutdown request.
            # Set SCR_USER4 = 1 in Mission Planner to trigger a clean shutdown.
            # Use this before cutting drone power to protect the SD card.
            if not hasattr(mav_thread, '_last_shutdown_poll'):
                mav_thread._last_shutdown_poll = 0.0
            if time.time() - mav_thread._last_shutdown_poll > SCR_USER_SHUTDOWN_POLL_SEC:
                mav_thread._last_shutdown_poll = time.time()
                try:
                    m_fc.param_fetch_one(SCR_USER_SHUTDOWN)
                    pmsg = m_fc.recv_match(type="PARAM_VALUE", blocking=True, timeout=2)
                    if (pmsg and pmsg.param_id.strip('\x00') == SCR_USER_SHUTDOWN
                            and int(pmsg.param_value) == 1):
                        logger.info("SCR_USER4 == 1: Pi shutdown requested from Mission Planner")
                        # 071426: Reset SCR_USER4 back to 0 before shutting down
                        # so the next boot does not immediately trigger another shutdown.
                        try:
                            m_fc.param_set_send(SCR_USER_SHUTDOWN, 0)
                            logger.info("SCR_USER4 reset to 0 on FC")
                        except Exception as _re:
                            logger.info("SCR_USER4 reset failed: %s" % _re)
                        logger.info("Running: sudo shutdown -h now")
                        import os as _os
                        _os.system("sudo shutdown -h now")
                except Exception as _e:
                    logger.info("SCR_USER4 poll failed: %s" % _e)

            # Non-blocking: recv_match() above already provides the natural pacing (it
            # blocks up to 2s only when the FC link is genuinely idle, and returns
            # immediately when messages are already buffered). Blocking here for up to
            # 0.1s on every single loop pass artificially capped how fast this loop could
            # drain incoming MAVLink traffic to ~10/sec -- fine over a real 115200-baud
            # UART, which can't produce messages faster than that anyway, but against a
            # simulator/network link pushing messages much faster, a backlog builds up
            # behind that cap and the lag between a real RC/servo event and this code
            # seeing it grows the longer it runs.
            try:
                cmd = q_mav.get_nowait()
                logger.info("cmd:%s" % cmd)
            except queue.Empty:
                continue

            action = (cmd.get("action") or "").upper()
            logger.info("mav action:%s" % action)

            if action == "SENDPAYLOAD":
                cols = blest.get("last_cols")

                if not cols:
                    logger.info("SENDPAYLOAD skipped: no BLE data available")
                    continue

                n = len(cols.get("time", []))
                if n <= 0:
                    logger.info("SENDPAYLOAD skipped: BLE data is empty")
                    continue

                try:
                    logger.info("Uploading fetched BLE data: cols:%s" % cols)
                    send_payload(payload_link, cols, sensor_state)
                except Exception as e:
                    logger.info("SENDPAYLOAD failed, MAV thread will continue: %s" % e)
                    logger.info(traceback.format_exc())
                    continue

    except Exception as e:
        logger.info("MAVLINK thread crashed: %s" % e)
        logger.info(traceback.format_exc())


def main():
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")

    logging.info("adc_sim_flag: %s" % adc_sim_flag)
    stop_evt = threading.Event()
    q_winch = queue.Queue()
    q_ble = queue.Queue()
    q_mav = queue.Queue()

    t_mav = threading.Thread(
        target=mav_thread,
        name="MAVLINK",
        args=(stop_evt, q_winch, q_ble, q_mav, wParms, winch_st, ble_st),
    )
    t_win = threading.Thread(
        target=winch_thread,
        name="WINCH",
        args=(stop_evt, q_winch, wParms, winch_st),
    )
    t_ble = threading.Thread(
        target=ble_thread,
        name="BLE",
        args=(stop_evt, q_ble, q_mav, ble_st),
    )

    def cleanup(*_args):
        logger.info("Stopping")
        stop_evt.set()
        q_ble.put({"action": "DISCONNECT"})
        t_mav.join(timeout=1)
        t_win.join(timeout=0.5)
        t_ble.join(timeout=0.5)
        sys.exit(0)

    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    #t_mav.daemon = True
    #t_win.daemon = True
    #t_ble.daemon = True

    t_win.start()
    t_mav.start()
    t_ble.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        logger.info("Ctrl C stopping")
        stop_evt.set()
    finally:
        cleanup()


if __name__ == "__main__":
    main()
