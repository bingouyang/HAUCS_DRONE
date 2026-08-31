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

# 083026: rail monitoring. Guarded so a missing ina_helper.py or smbus2 leaves
# the winch fully functional with the current feature simply switched off.
try:
    from ina_helper import INA226
    _INA_IMPORT_ERR = None
except Exception as _e:          # pragma: no cover
    INA226 = None
    _INA_IMPORT_ERR = _e

# ---- Script version -------------------------------------------------------
# 083026: single place to confirm which build is running. Logged at startup, so
# the head of logs/cc_*.log identifies it without grepping for change markers.
# Bump the trailing number on any deploy; change the date on a new day's work.
#   latch-083026.1   wincfg NameError fixed in ble_thread (was discarding every
#                    fetched cast), frame-count var_id lookup fixed, fault
#                    injector wired in behind adc_fault_flag
#   direct-083026.2   wire contract v2 (FRAME_END scale)
#   direct-083026.3   INA226 rail monitoring (WVOLT/WAMP/WPKA) and the HAUCS
#                     numeric status code on the Mission Planner HUD
SCRIPT_VERSION = "direct-083026.3"

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
# 082426: Mission Planner promotes STATUSTEXT at WARNING (4) or lower to the HUD
# message line. NOTICE (5) and INFO (6) land in the Messages tab only.
SEV_ALERT = 4
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
    "HALL_TARGET": 2500,
    "RETRACT_PWR": 0.1,
    "RELEASE_PWR": -0.30,
    "NEUTRAL_POS": 0.0,
    "ROTATION_DIRECTION": -1,
    "RELEASE_SEC": 20,      # 071426: motor drives payload down; overridden by SCR_USER1
    "RETRACT_SETTLE": 50,
    # 081426: ascent sampling scheme.
    # STEP_MOVE_SEC = 0 keeps the existing continuous retract, unchanged.
    # STEP_MOVE_SEC > 0 switches the ascent to stepped: drive for
    # STEP_MOVE_SEC, then sit still for STEP_HOLD_SEC so the DO sensor can
    # settle before the next stage.
    #
    # Why: the probe has a first-order lag of about 2.9 s. Ascending
    # continuously at ~3.5 in/s smears every reading by v*tau, roughly 10
    # inches, so features thinner than that are lost. Holding still lets the
    # reading relax to the true local value; residual error is
    # step_size * exp(-HOLD/tau).
    #
    #   MOVE  HOLD   step    error   stages  total
    #   1.0   6.0    3.5in   0.44in   11      77s
    #   0.5   6.0    1.8in   0.22in   21     136s
    #   2.0   3.0    7.0in   2.49in    6      30s
    #
    # RETRACT_SEC must be at least the total, or the ascent is cut short.
    "STEP_MOVE_SEC": 0.0,     # SCR_USER5; 0 = continuous
    "STEP_HOLD_SEC": 6.0,     # SCR_USER6; settle time per stage
    "STATUS_TO_GCS": 1,       # 081426: 0 disables all operator messages
    "STATUS_MIN_GAP": 1.0,    # min seconds between routine messages
    "STATUS_REPEAT_SEC": 30.0,  # suppress an identical error inside this window
    "RETRACT_TH": 8000,
    "PWD_ADP_TH": 10000,
    "LOG_PREFIX": "[WINCH] ",
    "PAUSE_SEC": 2,         # 071426: idle at bottom before retract; overridden by SCR_USER2
    # 083026: INA226 rail monitoring on the Pi I2C bus, alongside the ADS1115.
    # INA_ENABLE 0 disables every part of it; nothing else changes.
    "INA_ENABLE": 1,
    "INA_ADDR": 0x44,
    "INA_SHUNT_OHM": 0.009091,  # R100 || R010; re-measure in place and update
    "INA_AVG": 16,              # 35 ms conversion
    "INA_POLL_HZ": 20.0,        # polled next to every Hall read
    "INA_HUD_HZ": 2.0,          # WVOLT / WAMP to the GCS
    # Current above this while the servo is driving means the mechanism is not
    # moving. Reported only - nothing is aborted on it. Healthy free-running
    # draw measured 0.15 A, the worn bench unit 0.60 A, and the MP1584 limits
    # around 3 A, so 1.5 A is clear of normal and below the converter ceiling.
    "INA_STALL_A": 1.5,
    "INA_STALL_SEC": 0.75,      # sustained for this long before it is reported
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
    # 083026: the rail is polled here so voltage and current are sampled within
    # milliseconds of the Hall value. That adjacency is the point: Hall frozen
    # with normal current is a dead sensor, Hall frozen with stall current is a
    # real jam, and the two are indistinguishable if the reads drift apart.
    # The helper rate-limits internally, so calling it from this hot path costs
    # a comparison on most passes.
    ina_poll(wParms)
    if adc_sim_flag == 1:
        return int(c.read())
    else:
        return int(c.value)

def release_win(servo, adc, cfg, st, stop_evt):
    # 071426: Hall sensor check removed from early-exit logic. Mechanism is
    # direct-drive descent at low power (RELEASE_PWR) for RELEASE_SEC duration.
    # 082326: Hall pre-check retained, but it is a line-position check, not a
    # polarity check: if the line reads still out, recover it before releasing.

    # --- Pre-release Hall sensor sanity check ---
    RECOVERY_SEC = 10.0
    try:
        hall_start = hall_median(adc)
        if hall_start > cfg["RETRACT_TH"]:
            # Line still out. This is recoverable: pull it in, then release.
            # Aborting used to strand the winch, since nothing else retracts it,
            # so every later cast hit the same check and the session produced
            # flat surface-only profiles.
            gcs_status("line still out (hall %d), recovering" % hall_start,
                       cfg, force=True)
            logger.info("Hall reads %d before release (expected near %d). Line "
                        "appears still out - attempting recovery retract, up to %.0fs."
                        % (hall_start, cfg["HALL_TARGET"], RECOVERY_SEC))

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
                haucs_code(8, "ABORT release, line stuck out", cfg,
                           sev=SEV_ALERT)                    # 083026
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
    # 083026: new cast. Reset the peak so WPKA reports this cast, not the last.
    if _ina["dev"] is not None:
        try:
            _ina["dev"].reset_peaks()
        except Exception:
            pass
    haucs_code(1, "release %.0fs, sampling started" % cfg["RELEASE_SEC"], cfg)
    servo.value = float(svalue)
    haucs_code(2)                                            # 083026 descending

    while not stop_evt.is_set():
        if (time.time() - t0) > cfg["RELEASE_SEC"]:
            logger.info("Release complete: RELEASE_SEC=%ss reached" % cfg["RELEASE_SEC"])
            break

        # 082326: mid-release polarity check removed; the servo no longer
        # flips direction on its own.
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


def retract_stepped(servo, adc, cfg, st, stop_evt, dur):
    """
    081426: stepped ascent. Drive STEP_MOVE_SEC, stop, wait STEP_HOLD_SEC,
    repeat. Returns True if the winch reached the settle window.

    Deliberately does NOT call retract_adaptive. That function owns the
    continuous path and carries a polarity guard baselined on a single
    motor start; a stepped ascent restarts the motor at every stage and the
    ~366 count start transient would trip it. Position is read only during
    the holds, when the motor is off and the reading is quiet.
    """
    move_s = float(cfg.get("STEP_MOVE_SEC", 0.0))
    hold_s = float(cfg.get("STEP_HOLD_SEC", 6.0))
    drive = cfg["ROTATION_DIRECTION"] * cfg["RETRACT_PWR"] + cfg["NEUTRAL_POS"]
    n_est = max(1, int(dur // max(0.1, move_s + hold_s)))

    # 081426: cap stage chatter. Mission Planner's message pane scrolls, so a
    # fast cycle time could push a real EKF or battery warning out of view
    # even though severity INFO can never preempt one. Emit at most ~8 stage
    # messages per ascent whatever the stage count; start / done / abort are
    # force=True and always get through.
    msg_every = max(1, n_est // 8)

    t0 = time.time()
    step = 0

    logger.info("081426 stepped ascent: move %.2fs hold %.2fs, up to %d stages"
                % (move_s, hold_s, n_est))
    gcs_status("ascent stepped %.1f/%.1fs x%d" % (move_s, hold_s, n_est),
               cfg, force=True)

    while not stop_evt.is_set() and (time.time() - t0) < dur:
        step += 1

        servo.value = float(drive)
        m_end = time.time() + move_s
        while time.time() < m_end and not stop_evt.is_set():
            time.sleep(0.02)
        neutral(servo, cfg)

        # Motor is off now, so this reading is the quiet one.
        time.sleep(0.15)
        try:
            hall = hall_median(adc)
        except Exception as e:
            logger.info("081426 stepped: hall read failed: %s" % e)
            hall = None

        if hall is not None:
            dist = hall - cfg["HALL_TARGET"]
            if dist < cfg["RETRACT_SETTLE"]:
                st["RETRACTED"] = 1
                fsm_st["deploy_allowed"] = True
                logger.info("081426 stepped: retracted after %d stages, %.0fs"
                            % (step, time.time() - t0))
                gcs_status("ascent done %d stages %.0fs" %
                           (step, time.time() - t0), cfg, force=True)
                return True

            # 082426: backward-direction abort removed with the other polarity
            # guards; the servo memory flag is disabled, so direction no longer
            # flips on its own.

        if step % msg_every == 0:
            gcs_status("stage %d/%d hall %s hold %.1fs"
                       % (step, n_est,
                          hall if hall is not None else "??", hold_s), cfg)

        h_end = time.time() + hold_s
        while time.time() < h_end and not stop_evt.is_set():
            if (time.time() - t0) >= dur:
                break
            time.sleep(0.05)

    logger.info("081426 stepped: timeout after %d stages" % step)
    haucs_code(9)                                            # 083026
    gcs_status("ascent timeout %d stages" % step, cfg, force=True,
               sev=SEV_ALERT)
    return False


def retract_adaptive(servo, adc, cfg, st):
    try:
        hall_now = hall_raw(adc)
        dist = hall_now - cfg["HALL_TARGET"]

    # 082326: retract polarity safeguard removed. Direction reversal is now
    # prevented at the servo, whose polarity-flip flag is disabled, so the
    # _hall_start baseline and its SAFETY ABORT are no longer needed.
    except Exception:
        haucs_code(10, None, cfg)                            # 083026 hall read failed
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
            fsm_st["deploy_allowed"] = True
            logger.info("Deploy re-enabled (fully retracted)")
            if is_driving_retract(servo, cfg):   # 082526
                neutral(servo, cfg)
            return True

        if is_driving_retract(servo, cfg):   # 082526
            neutral(servo, cfg)

    elif (dist < cfg["PWD_ADP_TH"]) and is_driving_retract(servo, cfg):   # 082526
        servo.value = cfg["ROTATION_DIRECTION"] * pwr + cfg["NEUTRAL_POS"]
        logger.info("currnt adaptive pwr: %s" % servo.value)

    elif dist > cfg["RETRACT_TH"]:
        if st["RETRACTED"] != 0:
            st["RETRACTED"] = 0

    return False

# 081426: operator-facing status to Mission Planner.
# MAV_SEVERITY_INFO (6) sits below every mission-critical severity (0-4), so
# these land in the message pane and can never displace a real warning.
# Rate limited, and capped at the 50 byte STATUSTEXT payload.
_status = {"link": None, "last": 0.0}


# ---------------------------------------------------------------------------
# 083026: numeric status code on the HUD.
#
# NAMED_VALUE_FLOAT reaches both Mission Planner's HUD user items and the Quick
# tab, and MP holds the last value it received, so the number stays on screen.
# STATUSTEXT does not persist, which is why the code goes on this channel and
# the wording stays in the Messages tab.
#
# Under 8 is normal progress, 8 and above is a fault: one rule for the operator.
HAUCS_CODES = {
    0:  "idle",
    1:  "release commanded",
    2:  "payload descending",
    3:  "sampling at depth",
    4:  "retract running",
    5:  "cast complete, transmitting",
    8:  "LATCH/RELEASE FAILED",
    9:  "RETRACT TIMEOUT",
    10: "SENSOR FAULT",
    11: "NO SAMPLES",
    12: "OVERCURRENT / STALL",
}

HAUCS_FIELD = b"HAUCS"          # NAMED_VALUE_FLOAT names are capped at 10 bytes

# code   last value sent, resent at HAUCS_REPEAT_S so a dropped packet cannot
#        leave a stale number on the operator's screen
_hstat = {"code": 0, "last_send": 0.0}
HAUCS_REPEAT_S = 1.0


def _named_float(m, name, value):
    """One NAMED_VALUE_FLOAT. Never raises."""
    try:
        m.mav.named_value_float_send(
            int(time.time() * 1000) & 0xFFFFFFFF, name, float(value))
        return True
    except Exception as e:
        logger.info("083026 named_value_float %s failed: %s" % (name, e))
        return False


def haucs_code(code, detail=None, cfg=None, sev=None):
    """Set the HUD status code. Sends immediately on a change and lets
    haucs_tick() keep it alive. detail, if given, goes to the Messages tab
    through the existing gcs_status path."""
    m = _status["link"]
    changed = (code != _hstat["code"])
    _hstat["code"] = int(code)
    if m is not None:
        _named_float(m, HAUCS_FIELD, code)
        _hstat["last_send"] = time.time()
    if changed:
        logger.info("HAUCS code %d (%s)" % (code, HAUCS_CODES.get(code, "?")))
    if detail:
        gcs_status(detail, cfg, force=True, sev=sev)


def haucs_tick(m, now=None):
    """Resend the current code at HAUCS_REPEAT_S. Named floats are one-shot and
    MP simply keeps the last value, so without this a single dropped packet
    would leave the wrong number on screen until the next transition."""
    if m is None:
        return
    now = now or time.time()
    if (now - _hstat["last_send"]) < HAUCS_REPEAT_S:
        return
    _hstat["last_send"] = now
    _named_float(m, HAUCS_FIELD, _hstat["code"])


# ---------------------------------------------------------------------------
# 083026: INA226 rail monitoring.
#
# One owner polls: winch_thread, via hall_raw(), so voltage and current are
# sampled within milliseconds of the Hall value they have to be interpreted
# against. mav_thread never calls read(); it reads the cached attributes, so
# the two threads cannot race the helper's rate limiter or failure counter.
_ina = {"dev": None, "stall_since": 0.0, "stall_reported": False,
        "hud_last": 0.0}


def set_ina(dev):
    _ina["dev"] = dev


def ina_poll(cfg=None):
    """Poll the rail. Rate-limited inside the helper, so this is cheap to call
    from the Hall path. Returns (volts, amps), or (None, None)."""
    dev = _ina["dev"]
    if dev is None:
        return (None, None)
    try:
        v, a = dev.read()
    except Exception as e:              # the helper guards itself; belt and braces
        logger.info("083026 ina read raised: %s" % e)
        return (None, None)
    if v is None or a is None or cfg is None:
        return (v, a)

    # Sustained overcurrent while driving means the mechanism is not moving.
    # Reported only. Nothing is aborted on it, because a false positive here
    # would strand the winch, and the retract timeout already bounds the drive.
    now = time.time()
    if a >= float(cfg.get("INA_STALL_A", 1.5)):
        if _ina["stall_since"] == 0.0:
            _ina["stall_since"] = now
        elif (not _ina["stall_reported"]
              and (now - _ina["stall_since"]) >= float(cfg.get("INA_STALL_SEC", 0.75))):
            _ina["stall_reported"] = True
            logger.info("WARNING: winch overcurrent %.2f A at %.2f V for %.1fs "
                        "- mechanism may be stalled"
                        % (a, v, now - _ina["stall_since"]))
            haucs_code(12, "STALL %.1fA %.1fV" % (a, v), cfg, sev=SEV_ALERT)
    else:
        _ina["stall_since"] = 0.0
        _ina["stall_reported"] = False
    return (v, a)


def ina_hud_tick(m, cfg, now=None):
    """Send WVOLT / WAMP / WPKA. Called from mav_thread's loop; sends at
    INA_HUD_HZ regardless of how fast that loop runs. Reads cached values
    only - no I2C from this thread."""
    dev = _ina["dev"]
    if m is None or dev is None or not cfg.get("INA_ENABLE", 1):
        return
    now = now or time.time()
    period = 1.0 / float(cfg.get("INA_HUD_HZ", 2.0) or 2.0)
    if (now - _ina["hud_last"]) < period:
        return
    _ina["hud_last"] = now
    if not getattr(dev, "available", False) or dev.volts is None:
        return
    _named_float(m, b"WVOLT", dev.volts)
    _named_float(m, b"WAMP", dev.amps)
    # Peak accumulates at the poll rate, not this one, so a spike shorter than
    # the send interval is still reported. Reset at the start of each cast.
    _named_float(m, b"WPKA", dev.peak_a)


def send_payload_reported(link, cols, state, cfg):
    """
    081426: wrapper around send_payload that tells the operator what went out.

    encoder_helper.send_payload uses print(), which the log handler cannot
    see, so the DATA96 transmission was invisible on the ground. Frame count
    is derived the same way the encoder chunks: per-variable, using that
    variable's residue width (pressure is int16, so half the samples fit).
    """
    n_frames = 0
    n_samples = 0
    try:
        for name in SEND_ORDER:
            vals = cols.get(name) or []
            if not vals:
                continue
            # 083026: was preferring VAR_MAP[name], but encoder_helper's
            # VAR_MAP is a column template of empty lists ({"DO": [], ...})
            # that only defines SEND_ORDER - the name->id map lives in
            # sampling_helper on the GCS side. int([]) raised, and the whole
            # count was lost. prepare_per_var_queues() derives the id this way,
            # so match it.
            width = max_samples(SEND_ORDER.index(name))
            n_frames += (len(vals) + width - 1) // width
            if name == "DO":
                n_samples = len(vals)
    except Exception as e:
        logger.info("081426 frame count failed: %s" % e)

    before = sum(len(v) for v in (state.get("failed") or {}).values())
    gcs_status("TX %d frames, %d samples" % (n_frames, n_samples), cfg,
               force=True)

    send_payload(link, cols, state)

    after = sum(len(v) for v in (state.get("failed") or {}).values())
    if after > before:
        logger.info("TX incomplete: %d frames buffered to outbox" % after)
    else:
        gcs_status("TX done %d frames" % n_frames, cfg, force=True)


def set_status_link(m):
    _status["link"] = m


# 081426: anything that looks like a failure goes to the operator's screen.
# Most of this file reports failures with logger.info rather than warning or
# error, so level alone is not enough - match on wording as well.
# Bare "timeout" is deliberately absent: "Queue RETRACT with timeout 60s" is
# routine. Match the failure wording instead.
ERROR_WORDS = ("abort", "error", " fail", "failed", "corrupt", "warning",
               "not fully retracted", "prevented", "stuck", "backwards",
               "no heartbeat", "no response", "no samples", "timed out",
               "ignored", "exception", "unable", "invalid", "not connected")

_errstate = {"last": {}, "in_emit": False}


class GcsLogHandler(logging.Handler):
    """
    Forwards failures to Mission Planner as STATUSTEXT.

    Deduplicated: an identical message repeating inside STATUS_REPEAT_SEC is
    counted, not resent, and the count is shown when it does go out. Without
    that, a winch stuck in a retract-fail loop would repeat the same line
    every cycle and scroll everything else off the pane.
    """

    def __init__(self, cfg):
        logging.Handler.__init__(self)
        self.cfg = cfg

    def emit(self, record):
        if _errstate["in_emit"]:
            return                      # gcs_status logs on failure; no recursion
        try:
            msg = record.getMessage()
            low = msg.lower()
            if record.levelno < logging.WARNING and \
                    not any(w in low for w in ERROR_WORDS):
                return

            key = low[:40]
            now = time.time()
            seen = _errstate["last"].get(key)
            window = self.cfg.get("STATUS_REPEAT_SEC", 30.0)
            if seen and (now - seen[0]) < window:
                seen[1] += 1
                return
            count = (seen[1] if seen else 0)
            _errstate["last"][key] = [now, 0]

            text = msg.strip()
            if count:
                text = "%s (x%d)" % (text, count + 1)
            _errstate["in_emit"] = True
            try:
                gcs_status(text, self.cfg, force=True, sev=SEV_ALERT)  # 082426
            finally:
                _errstate["in_emit"] = False
        except Exception:
            pass


def gcs_status(text, cfg=None, force=False, sev=None):
    # 082426: sev picks where Mission Planner shows it. Default INFO keeps
    # routine progress in the Messages tab; SEV_ALERT puts it on the HUD too.
    m = _status["link"]
    if m is None:
        return
    cfg = cfg or {}
    if not cfg.get("STATUS_TO_GCS", 1):
        return
    now = time.time()
    if not force and (now - _status["last"]) < cfg.get("STATUS_MIN_GAP", 1.0):
        return
    _status["last"] = now
    try:
        m.mav.statustext_send(
            sev if sev is not None else mavutil.mavlink.MAV_SEVERITY_INFO,
            ("HAUCS: " + text)[:49].encode("ascii", "replace"))
    except Exception as e:
        logger.info("081426 statustext failed: %s" % e)


def is_driving_retract(servo, cfg):
    """082526: True if the servo is currently commanded in the retract
    direction. neutral() now stops the pulse train, which leaves servo.value
    as None, and None * int raises TypeError - caught by the bare except in
    retract_adaptive() and therefore silent. Guard it in one place."""
    v = servo.value
    if v is None:
        return False
    return (cfg["ROTATION_DIRECTION"] * v) > 0.0


def neutral(servo, cfg):
    # 082526: was servo.value = NEUTRAL_POS, a continuous 1500us pulse train.
    # That keeps the servo powered and commanded whenever the winch is idle:
    # between casts, through the bottom pause, and for the whole time the
    # script runs with no mission. Bench measurement showed a damaged unit
    # dissipating ~12 W in that state while a healthy one drew nothing
    # measurable, and the servo runs cool whenever the pulses stop.
    # servo.value = None stops the pulse train, which is exactly the state the
    # servo is in when this script is not running. Safe here because the
    # payload holds position with the drone powered down, i.e. with no signal.
    #
    # The simulator keeps the old behaviour: ServoSim/LinkedHallADC read
    # servo.value to model the winch, and None is not a value they expect.
    if adc_sim_flag == 1:
        servo.value = cfg["NEUTRAL_POS"]
        logger.info('inside neutral (sim, holding neutral value)')
    else:
        servo.value = None
        logger.info('inside neutral (pulses stopped)')

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

    # 083026: rail monitor. Separate try block, and set_ina() is only reached
    # on success, so any failure here leaves _ina["dev"] as None and every
    # call site degrades to a no-op. The winch runs exactly as before.
    if cfg.get("INA_ENABLE", 1):
        try:
            if adc_sim_flag == 1 and getattr(adc, "ina", None) is not None:
                # FaultyHallADC(ina_sim=True) carries its own simulated rail,
                # driven by the same blocked/not-blocked state it injects.
                set_ina(adc.ina)
                logger.info("083026 rail monitor: simulated")
            elif INA226 is None:
                logger.info("083026 rail monitor unavailable: %s" % _INA_IMPORT_ERR)
            else:
                # tare=True is valid here: neutral() has not run yet and the
                # servo is not being driven, so the shunt genuinely sees zero.
                dev = INA226(shunt_ohm=cfg["INA_SHUNT_OHM"],
                             addr=cfg["INA_ADDR"],
                             avg=cfg["INA_AVG"],
                             rate_hz=cfg["INA_POLL_HZ"],
                             tare=True)
                if dev.available:
                    set_ina(dev)
                    v, a = dev.read(force=True)
                    logger.info("083026 rail monitor ready: %.3f V, %.3f A idle, "
                                "offset %+.1f uV, shunt %.4f ohm"
                                % (v or 0.0, a or 0.0, dev.offset_v * 1e6,
                                   dev.shunt_ohm))
                else:
                    logger.info("083026 rail monitor init failed: %s" % dev.err)
        except Exception as e:
            logger.info("083026 rail monitor init raised: %s" % e)
    else:
        logger.info("083026 rail monitor disabled (INA_ENABLE=0)")

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
                    haucs_code(3, None, cfg)                 # 083026 at depth

                    if stop_evt.wait(pause_sec):
                        continue

                    logger.info("Bottom pause finished. Queue RETRACT with timeout %.1f sec" % retract_sec)
                    q_winch.put({"action": "RETRACT", "duration": retract_sec})

                elif act == "RETRACT":
                    dur = float(cmd.get("duration", 0.0))
                    logger.info("RETRACT for %ss" % dur)
                    haucs_code(4, None, cfg)                 # 083026 retracting
                    # 081426: stepped ascent when SCR_USER5 > 0, otherwise the
                    # original continuous retract below, untouched.
                    if float(cfg.get("STEP_MOVE_SEC", 0.0)) > 0.0:
                        retract_stepped(servo, adc, cfg, st, stop_evt, dur)
                        neutral(servo, cfg)
                        q_winch.task_done()
                        continue

                    t0 = time.time()
                    servo.value = cfg["ROTATION_DIRECTION"] * cfg["RETRACT_PWR"]
                    logger.info("servo.value %s" % servo.value)
                    time.sleep(0.25)

                    # 082426: no-progress stall guard removed. On this Hall
                    # sensor a moving winch far from home reads flat for
                    # seconds, so "no progress" cannot be told from "stalled",
                    # and any fixed threshold breaks at a different cast depth.
                    while not stop_evt.is_set() and (time.time() - t0) < dur:
                        retract_flag = retract_adaptive(servo, adc, cfg, st)
                        if retract_flag is True:
                            logger.info("Fully retractd!")
                            break

                        time.sleep(0.1)
                    if (time.time() - t0) >= dur:
                        haucs_code(9, None, cfg)             # 083026
                        logger.info("WARNING: Retract timeout, not fully retracted, future release prevented for now")
                    neutral(servo, cfg)

                elif act == "NEUTRAL":
                    neutral(servo, cfg)
                    haucs_code(0, None, cfg)                 # 083026 idle

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
                            # 081426: the message the operator is waiting for.
                            try:
                                _n = int(s_size[1])
                            except Exception:
                                _n = -1
                            # 083026: was wincfg, which is mav_thread's name for
                            # the wParms dict and does not exist in ble_thread.
                            # The NameError was raised while evaluating the
                            # argument, before gcs_status ran, and unwound the
                            # whole fetch handler below - no CSV cached, no
                            # upload queued, on every cast that reached here.
                            haucs_code(5, "CAST COMPLETE, %d samples" % _n,
                                       wParms)               # 083026
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
                                haucs_code(11, "CAST COMPLETE but 0 samples",
                                           wParms, sev=SEV_ALERT)   # 083026
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
# SCR_USER5 = STEP_MOVE_SEC  (0 = continuous ascent, >0 = stepped)
# SCR_USER6 = STEP_HOLD_SEC  (settle time at each stage)
#             Pi polls this each loop; triggers "sudo shutdown -h now" when == 1
# If a timing parameter is 0 or unset, the local wParms default is kept.
SCR_USER_MAP = {
    "SCR_USER1": "RELEASE_SEC",
    "SCR_USER2": "PAUSE_SEC",
    "SCR_USER3": "RETRACT_SEC",
    # 081426: SCR_USER4 is the shutdown flag, so the ascent scheme uses 5 and 6.
    # SCR_USER5 = 0 leaves STEP_MOVE_SEC at its 0.0 default, i.e. continuous.
    "SCR_USER5": "STEP_MOVE_SEC",
    "SCR_USER6": "STEP_HOLD_SEC",
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
        #m_fc = mavutil.mavlink_connection(FC_CONN_STR, baud=FC_BAUD)
        ## 082426 - using system=1 and component=191.
        m_fc = mavutil.mavlink_connection(FC_CONN_STR, baud=FC_BAUD,
                                  source_system=1,
                                  source_component=191)

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
        set_status_link(m_fc)   # 081426: operator status to Mission Planner
        # 083026: publish the code once so HAUCS appears in Mission Planner's
        # HUD user-item list straight away. MP only lists names it has seen.
        haucs_code(0, None, wincfg)
        logger.addHandler(GcsLogHandler(wincfg))
        # 071426: Store m_fc ref so fetch_scr_user_params can be called before each release.
        mav_fc_ref = m_fc

        sensor_state["failed"] = resend_buffer(payload_link, sensor_state.get("failed", {}))

        last_lat = None
        last_lon = None
        last_ping = time.time()
        dbg_prev_iter_start = time.monotonic()

        while not stop_evt.is_set():

            # 083026: HUD telemetry. Both are self-rate-limiting, so putting
            # them at the top of the loop covers the recv_match timeout path
            # too - the numbers keep updating even when no messages arrive.
            _now = time.time()
            ina_hud_tick(m_fc, wincfg, _now)
            haucs_tick(m_fc, _now)

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
                        logger.info("Deploy ignored: winch not retracted "
                                    "(RETRACTED=%s, hall not at target). "
                                    "Sampling is blocked until a retract "
                                    "completes." % winst["RETRACTED"])
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
                        send_payload_reported(payload_link, cols, sensor_state, wincfg)
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
                    send_payload_reported(payload_link, cols, sensor_state, wincfg)
                except Exception as e:
                    logger.info("SENDPAYLOAD failed, MAV thread will continue: %s" % e)
                    logger.info(traceback.format_exc())
                    continue

    except Exception as e:
        logger.info("MAVLINK thread crashed: %s" % e)
        logger.info(traceback.format_exc())


def main():
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")

    logging.info("SCRIPT_VERSION: %s" % SCRIPT_VERSION)          # 083026
    try:                                                          # 083026
        logging.info("wire contract: %s" % contract_id())
    except Exception as e:
        logging.info("wire contract: unavailable (%s)" % e)
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
