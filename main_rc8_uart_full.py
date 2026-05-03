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
GCS_UDP_OUT = "udpout:10.113.32.16:14555"

# Mavlink constants
NAV_IN_AIR = mavutil.mavlink.MAV_LANDED_STATE_IN_AIR
NAV_ON_GROUND = mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND
NAV_LANDING = mavutil.mavlink.MAV_LANDED_STATE_LANDING
NAV_SAFETY_ARMED = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
PING_SEC = 2
TOUCH_CONFIRM_SEC = 2

# Winch constants
wParms = {
    "SERVO_PIN": 17,
    "ADC_PIN": 0,
    "HALL_MIN": 1035,
    "HALL_MAX": 12285,
    "HALL_TARGET": 1035,
    "RELEASE_PWR": -0.4,
    "RETRACT_PWR": 0.30,
    "PWR_LIMIT": 0.30,
    "NEUTRAL_POS": -0.07,
    "ROTATION_DIRECTION": -1,
    "SAFETY_TIMEOUT": 0.7,
    "RETRACT_SETTLE": 50,
    "RETRACT_TH": 8000,
    "PWD_ADP_TH": 10000,
    "LOG_PREFIX": "[WINCH] ",
    "FREEFALL_SEC": 20,
    "RETRACT_SEC": 35,
}

# BLE parms
BLE_SAMPLE_SIZE = 120
BLE_SAMPLE_TYPE = "manual"
BLE_PMODE = "high"
BLE_FETCH_TIMEOUT = 5

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

# sensor data upload status and sequence id
sensor_upload_failed = {}
sensor_state = {"seq": 0}

logging.basicConfig(
    format="%(asctime)s %(levelname)s: %(message)s",
    filename="companion_computer.log",
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
    "auto_mode": True,
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
    "send_to_gcs": False,
    "_last_pwm": None,
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

def rc_channel_value(msg, ch):
    # Return PWM in microseconds for 1-based RC input channel ch from RC_CHANNELS.
    # MAVLink RC_CHANNELS fields are chan1_raw ... chan18_raw.
    if ch < 1 or ch > 18:
        return None
    return getattr(msg, "chan%d_raw" % ch, None)


def fsm_consume_flags(fsm):
    keys = ("deploy_needed", "send_to_gcs", "cycle_activated", "cycle_deactivated")
    out = {k: fsm.get(k, False) for k in keys}
    for k in keys:
        fsm[k] = False
    return out


def handle_rc_channels(
    msg,
    fsm,
    ch=TRIGGER_RC_CH,
    thresh_high=PWM_RELEASE,
    thresh_low=PWM_RETRACT,
    debounce_s=DEBOUNCE_S,
):
    pwm = rc_channel_value(msg, ch)

    if pwm is None:
        print("RC%d is not present in this RC_CHANNELS message" % ch)
        return

    if pwm == 0 or pwm == 65535:
        return

    if pwm > thresh_high:
        state = "HIGH"
    elif pwm < thresh_low:
        state = "LOW"
    else:
        state = "MID"

    #print("RC%d PWM: %s -> %s" % (ch, pwm, state))

    now_m = time.monotonic()
    last_pwm = fsm["_last_pwm"]

    if last_pwm is not None:
        if last_pwm < thresh_high <= pwm and (now_m - fsm["_last_rel_t"]) > debounce_s:
            fsm["_last_rel_t"] = now_m
            if not fsm["cycle_active"]:
                fsm["cycle_active"] = True
                fsm["cycle_activated"] = True
            fsm["deploy_needed"] = True
            print("RC rising edge detected")

        if last_pwm > thresh_low >= pwm:
            fsm["_last_ret_t"] = now_m
            if fsm["cycle_active"]:
                fsm["cycle_active"] = False
                fsm["cycle_deactivated"] = True
            fsm["send_to_gcs"] = True
            print("RC falling edge detected")

    fsm["_last_pwm"] = pwm

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
    st["auto_mode"] = True

    if custom_mode_name in ["RTL", "ACRO", "ALT_HOLD", "STABILIZE", "LOITER", "MANUAL", "Unknown"]:
        st["auto_mode"] = False
    else:
        st["auto_mode"] = True


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
    servo.value = -cfg["ROTATION_DIRECTION"] * abs(cfg["RELEASE_PWR"]) + cfg["NEUTRAL_POS"]
    t0 = time.time()
    print("Release, winch servo open: %s" % servo.value)

    while not stop_evt.is_set():
        if (time.time() - t0) > cfg["SAFETY_TIMEOUT"]:
            neutral(servo, cfg)
            print("safety timeout triggered during release")
            break

        try:
            dist = hall_raw(adc) - cfg["HALL_TARGET"]
        except Exception:
            neutral(servo, cfg)
            time.sleep(0.25)
            break

        if dist > cfg["RETRACT_TH"]:
            st["RETRACTED"] = 0
            break

        time.sleep(0.25)

    neutral(servo, cfg)


def retract_adaptive(servo, adc, cfg, st):
    try:
        dist = hall_raw(adc) - cfg["HALL_TARGET"]
    except Exception:
        neutral(servo, cfg)
        time.sleep(0.25)
        return False

    pwr = dist / float(cfg["HALL_MAX"] - cfg["HALL_MIN"])
    pwr = pwr * cfg["PWR_LIMIT"]

    if pwr > 0.0:
        pwr = math.pow(pwr, 1.0 / 3.0)

    if pwr > cfg["PWR_LIMIT"]:
        pwr = cfg["PWR_LIMIT"]
    else:
        pwr = 0

    if dist < (cfg["HALL_TARGET"] + cfg["RETRACT_SETTLE"]):
        if st["RETRACTED"] != 1:
            st["RETRACTED"] = 1
            if (cfg["ROTATION_DIRECTION"] * servo.value) > 0.0:
                neutral(servo, cfg)
            return True

        if (cfg["ROTATION_DIRECTION"] * servo.value) > 0.0:
            neutral(servo, cfg)

    elif (dist < 10000) and ((cfg["ROTATION_DIRECTION"] * servo.value) > 0.0):
        servo.value = cfg["ROTATION_DIRECTION"] * pwr + cfg["NEUTRAL_POS"]

    elif dist > cfg["RETRACT_TH"]:
        if st["RETRACTED"] != 0:
            st["RETRACTED"] = 0

    return False


def neutral(servo, cfg):
    servo.value = cfg["NEUTRAL_POS"]


def broadcast_value(x, n):
    return [] if n <= 0 else [x] * n


# simulator flags
data_sim_flag = False
adc_sim_flag = 0


def winch_thread(stop_evt, q_winch, cfg, st):
    try:
        servo = Servo(
            cfg["SERVO_PIN"],
            min_pulse_width=0.0009,
            max_pulse_width=0.0021,
            frame_width=0.02,
            pin_factory=PiGPIOFactory(),
            initial_value=cfg["NEUTRAL_POS"],
        )
    except Exception as e:
        print("Servo init failed: %s" % e)
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
            #adc = AnalogIn(ads, ADS.P0)
            adc = AnalogIn(ads, 0)
    except Exception as e:
        print("ADS1115 init failed: %s" % e)
        return

    print("winch ready")

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

                elif act == "RETRACT":
                    dur = float(cmd.get("duration", 0.0))
                    print("RETRACT for %ss" % dur)
                    t0 = time.time()
                    servo.value = cfg["ROTATION_DIRECTION"] * cfg["RETRACT_PWR"]

                    while not stop_evt.is_set() and (time.time() - t0) < dur:
                        retract_flag = retract_adaptive(servo, adc, cfg, st)
                        if retract_flag is True:
                            break
                        time.sleep(0.05)

                    neutral(servo, cfg)

                elif act == "NEUTRAL":
                    neutral(servo, cfg)

            time.sleep(0.1)

    except Exception as e:
        print("winch thread crashed:%s" % e)
        neutral(servo, cfg)

    finally:
        neutral(servo, cfg)
        time.sleep(0.1)


def ble_thread(stop_evt, q_ble, q_mav, st):
    ble = st["ble"] = BluetoothReader(st["ble_mutex"])

    try:
        st["c_status"] = "disconnected"

        while ble is None or not ble.check_connection_status():
            if ble.connect():
                ble.set_lights("navigation")
                logger.debug("connected to sensor, activated lights")
            ble.init_sensor_status()
            time.sleep(0.2)
            ble.set_calibration_pressure()
            time.sleep(0.2)
            print("set sample_type:%s" % BLE_SAMPLE_TYPE)
            ble.set_smpl_type(BLE_SAMPLE_TYPE)
            time.sleep(0.1)
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
                print("cmd:%s" % cmd)
            except queue.Empty:
                cmd = None

            if cmd:
                action = (cmd.get("action") or "").upper()

                if action == "START":
                    print("BLE action:START")
                    if ble.sdata.get("connection"):
                        try:
                            ble.set_sample_reset()
                            ble.set_sampl_flag(1)
                            sflag = ble.get_sampl_flag()
                            print("start sampling, sampling flag: %s" % sflag)
                            s_type = ble.get_smpl_type()
                            print("sample_type:%s" % s_type)
                            st["sampling"] = True
                            st["c_status"] = "sampling_started"
                        except Exception:
                            st["c_status"] = "start_failed"
                    else:
                        st["c_status"] = "start_skipped_disconnected"

                elif action == "FETCH":
                    print("BLE action:FETCH: %s" % ble.sdata.get("connection"))
                    if ble.sdata.get("connection"):
                        try:
                            ble.set_sampl_flag(0)
                            time.sleep(0.1)
                            sflag = ble.get_sampl_flag()
                            print("stop sampling sampling flag: %s" % sflag)
                            s_size = ble.get_sample_size()
                            ok = bool(ble.get_sample_data())
                            st["sampling"] = False
                            print(
                                "finish sampling - queue mav cmd to upload data, ok: %s, sample_size:%s"
                                % (ok, s_size)
                            )

                            if ok:
                                do_list = ble.sdata.get("do_vals") or []
                                temp_list = ble.sdata.get("temp_vals") or []
                                press_list = ble.sdata.get("pressure_vals") or []
                                n = len(do_list)
                                print("sampling finished, length:%s" % n)
                                ts_list = list(range(n))

                                st["last_cols"] = {
                                    "time": ts_list,
                                    "do": do_list,
                                    "temp": temp_list,
                                    "press": press_list,
                                    "init_DO": broadcast_value(ble.sdata.get("init_do"), n),
                                    "init_pressure": broadcast_value(ble.sdata.get("init_pressure"), n),
                                    "batt_v": broadcast_value(ble.sdata.get("battv"), n),
                                }
                                st["gcs_ready"] = True
                                st["c_status"] = "fetched"
                                q_mav.put({"action": "sendpayload"})
                            else:
                                st["c_status"] = "fetch_empty"

                        except Exception as e:
                            st["c_status"] = "fetch_failed"
                            print("fetch failed: %s" % e)
                    else:
                        st["c_status"] = "fetch_skipped_disconnected"

                elif action == "DISCONNECT":
                    ble_close(ble)
                    st["c_status"] = "disconnected_by_request"

    except Exception as e:
        print("BLE thread crashed: %s" % e)

    finally:
        print("BLE thread close")
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


def mav_thread(stop_evt, q_winch, q_ble, q_mav, wincfg, winst, blest):
    if "failed" not in sensor_state:
        sensor_state["failed"] = load_buffer(BUFFER_PATH)

    print("MAVLINK: starting on FC link %s @ %s" % (FC_CONN_STR, FC_BAUD))

    try:
        m_fc = mavutil.mavlink_connection(FC_CONN_STR, baud=FC_BAUD)

        print("MAVLINK: waiting for HEARTBEAT from Cube...")
        hb = m_fc.wait_heartbeat(timeout=10)

        if not hb:
            print("MAVLINK: no HEARTBEAT in 10s (check UART wiring, baud, serial port)")
        else:
            print(
                "MAVLINK: connected: sys=%s comp=%s"
                % (hb.get_srcSystem(), hb.get_srcComponent())
            )

        # CHANGED: UART-only. Send payloads back through the flight-controller link.
        payload_link = m_fc

        sensor_state["failed"] = resend_buffer(payload_link, sensor_state.get("failed", {}))

        last_lat = None
        last_lon = None
        last_ping = time.time()

        while not stop_evt.is_set():
            # CHANGED: For debugging and control, block specifically on RC_CHANNELS.
            # This prevents printing every MAVLink message and makes missing RC stream obvious.
            msg = m_fc.recv_match(type="RC_CHANNELS", blocking=True, timeout=2)

            if msg is None:
                print("No RC_CHANNELS message received...")
            else:
                handle_rc_channels(msg, fsm_st)
                flags = fsm_consume_flags(fsm_st)

                if flags["cycle_activated"]:
                    pass

                if flags["deploy_needed"]:
                    print("EVENT: start BLE sampling")
                    q_ble.put({"action": "START"})
                    q_winch.put({"action": "RELEASE"})

                    freefall_sec = wincfg["FREEFALL_SEC"]
                    retract_sec = wincfg["RETRACT_SEC"]

                    def enqueue_retract():
                        q_winch.put({"action": "RETRACT", "duration": retract_sec})

                    pending_retract_timer = threading.Timer(freefall_sec, enqueue_retract)
                    pending_retract_timer.daemon = True
                    pending_retract_timer.start()

                if flags["send_to_gcs"]:
                    sensor_state["failed"] = resend_buffer(payload_link, sensor_state.get("failed", {}))

                    if data_sim_flag is True:
                        cols = prep_sim_data(csv_path)
                        send_payload(payload_link, cols, sensor_state)
                    else:
                        print("MAV to BLE: fetch data from BLE sensor")
                        q_ble.put({"action": "FETCH"})

                if flags["cycle_deactivated"]:
                    print("EVENT: cycle_deactivated")
                    q_winch.put({"action": "NEUTRAL"})

            try:
                cmd = q_mav.get(timeout=0.1)
                print("cmd:%s" % cmd)
            except queue.Empty:
                continue

            action = (cmd.get("action") or "").upper()
            print("mav action:%s" % action)

            if action == "SENDPAYLOAD":
                cols = blest["last_cols"]
                print("Uploading fetched BLE data: cols:%s" % cols)
                send_payload(payload_link, cols, sensor_state)

    except Exception as e:
        print("MAVLINK thread crashed: %s" % e)
        print(traceback.format_exc())


def main():
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")

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
        print("Stopping")
        stop_evt.set()
        q_ble.put({"action": "DISCONNECT"})
        t_mav.join(timeout=1)
        t_win.join(timeout=0.5)
        t_ble.join(timeout=0.5)
        sys.exit(0)

    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    t_win.start()
    t_mav.start()
    t_ble.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Ctrl C stopping")
        stop_evt.set()
    finally:
        cleanup()


if __name__ == "__main__":
    main()
