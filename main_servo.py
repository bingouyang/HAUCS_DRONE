from pymavlink import mavutil
import threading, queue, time, logging, sys
import traceback
from builtins import range
import math
from dataclasses import dataclass
import signal
from threading import Lock as QMutex
from typing import Dict, Any

import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero import Servo
from winch_helper   import *
from encoder_helper import *
from bt_helper import *
############# ADC Simulator ###############
from adc_sim import ServoSim, LinkedHallADC 
###########################################

COPTER_MODES = {
    0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO",
    4: "GUIDED", 5: "LOITER", 6: "RTL", 7: "CIRCLE",
    9: "LAND", 11: "DRIFT", 13: "SPORT", 14: "FLIP",
    15: "AUTOTUNE", 16: "POSHOLD", 17: "BRAKE", 18: "THROW",
    19: "AVOID_ADSB", 20: "GUIDED_NOGPS", 21: "SMART_RTL",
    22: "FLOWHOLD", 23: "FOLLOW", 24: "ZIGZAG", 25: "SYSTEMID",
    26: "AUTOROTATE", 27: "AUTO_RTL",
}

# ---------------- CONFIG ----------------
# testing against simulator event stream:
#CONN_STR = 'udpin:0.0.0.0:14551'
CONN_STR_RX = 'udpin:0.0.0.0:14551'
CONN_STR_TX = 'udpout:10.120.239.168:14555'
#CONN_STR_TX = 'udpout:10.113.55.239:14555'
#CONN_STR_TX = 'udpout:192.168.1.160:14555'

# wired to Pixhawk TELEM2:
# CONN_STR = '/dev/serial0'
#BAUD = 115200

# Mavlink constants
NAV_IN_AIR       = mavutil.mavlink.MAV_LANDED_STATE_IN_AIR
NAV_ON_GROUND    = mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND
NAV_LANDING      = mavutil.mavlink.MAV_LANDED_STATE_LANDING
NAV_SAFETY_ARMED = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
PING_SEC = 2          # interval to check heartbeat
TOUCH_CONFIRM_SEC = 2    # interval to be sure drone is touch down

# Winch constants
wParms = {
    "SERVO_PIN": 17,
    "ADC_PIN": 0,                 # ADS1115 A0
    "HALL_MIN": 1035,
    "HALL_MAX": 12285,
    "HALL_TARGET": 1035,          # retracted target
    "RELEASE_PWR": -0.4,          # release power
    "RETRACT_PWR": 0.30,          # retract power
    "PWR_LIMIT":   0.30,          # cap retract power
    "NEUTRAL_POS": -0.07,         # adjust for servo creeping
    "ROTATION_DIRECTION": -1,     # -1 or +1 depending on wiring
    "SAFETY_TIMEOUT": 0.7,
    "RETRACT_SETTLE": 50,         # tolerance near target (raw units)
    "RETRACT_TH": 8000,
    "PWD_ADP_TH": 10000,          # adjust power when below this threshold(?)
    "LOG_PREFIX": "[WINCH] ",
    "FREEFALL_SEC":  20,           # how long to wait after release the winch
    "RETRACT_SEC":   35,           # how long to wait for the winch to fully retracted. 
}

#BLE parms
BLE_SAMPLE_SIZE  = 120 # two min of data
BLE_SAMPLE_TYPE  = "manual"
BLE_PMODE  = "high"
BLE_FETCH_TIMEOUT = 5

ble_st ={
    "ble": None,
    "ble_mutex": QMutex(),
    "last_cols": None,
    "gcs_ready": False,
    "c_status": "init",
}

# buffer json
BUFFER_PATH = "outbox.json"
csv_path = "input.csv"
# sensor data upload status and sequence id
sensor_upload_failed = {}  # { "seq_id": [ { "var_type": int, "payload": [ints] }, ... ] }
sensor_state = {"seq": 0} #initialize sequence counter
##### LOGGING #####
logging.basicConfig(
    format="%(asctime)s %(levelname)s: %(message)s",
    filename="companion_computer.log",
    encoding="utf-8",
    level=logging.INFO,
)
logger = logging.getLogger(__name__)
logger.info("Starting")

#########################
# Mavlink Status
mv_state = {
    'last_landed': None,
    'og_count': 0,
    'seen_init': False,
    'armed': None,
    'awaiting_final_td': False,  # set True after "Mission complete"
    'ground_ready': True,
    'td_fired': False,           # prevent repeated TOUCHDOWN
    'landing_fired': False,      # prevent repeated LANDING_START
    'auto_mode': True,           # assume we are in an auto mode
}

# =================== Winch status
winch_st = {
    "RETRACTED": 1,          # 1=retracted, 0=extended (update elsewhere if you want)
}
########################## FSM triggerd by servo ##############################
fsm_st = {
    "cycle_active": False,        # persistent: True after rising, False after falling
    "cycle_activated": False,     # set when cycle_active becomes True
    "cycle_deactivated": False,   # set when cycle_active becomes False
    
    "deploy_needed": False,       # set on rising edge
    "send_to_gcs": False,         # set on falling edge
    # internals (private)
    "_last_pwm": None,
    "_last_rel_t": 0.0,
    "_last_ret_t": 0.0,
}

# Trigger detection (RC or mission)
TRIGGER_CH = 9          # output channel for the "winch"
PWM_RELEASE = 1800    # rising through => Release ON
PWM_RETRACT = 1200    # falling through => Release OFF
DEBOUNCE_S = 0.1
####################################################
#Return PWM (µs) for servo channel ch from SERVO_OUTPUT_RAW
def _servo_out_value(msg, ch):
    """Return PWM (µs) for 1-based OUTPUT channel ch from SERVO_OUTPUT_RAW, else None."""
    if ch <= 8:
        port = 0
    elif ch <= 16:
        port = 1
    else:
        port = 2
    if getattr(msg, "port", 0) != port:
        return None
    idx = ((ch - 1) % 8) + 1
    return getattr(msg, f"servo{idx}_raw", None)

def fsm_consume_flags(fsm):
    #Read & clear one-shot flags.
    keys = ("deploy_needed", "send_to_gcs", "cycle_activated", "cycle_deactivated")
    out = {k: fsm.get(k, False) for k in keys}
    for k in keys:
        fsm[k] = False
    return out

def handle_servo_output_raw(msg, fsm,
                            ch=TRIGGER_CH,
                            pwm_release=PWM_RELEASE,
                            pwm_retract=PWM_RETRACT,
                            debounce_s=DEBOUNCE_S):
    """Edge detector: sets deploy/send flags and cycle activate/deactivate flags."""
    if "_last_pwm" not in fsm:
        fsm["_last_pwm"] = None
        fsm["_last_rel_t"] = 0.0
        fsm["_last_ret_t"] = 0.0

    pwm = _servo_out_value(msg, ch)
    if pwm is None:
        return

    now_m = time.monotonic()
    last_pwm = fsm["_last_pwm"]
    if last_pwm is not None:
        if (last_pwm != pwm):
                print(f"pwm changed: last_pwm:{last_pwm}, pwm:{pwm} pwm_retract:{pwm_retract}, pwm_release:{pwm_release}")
        # Rising edge (Release ON) -> activate cycle + deploy flag
        if last_pwm < pwm_release <= pwm and (now_m - fsm["_last_rel_t"]) > debounce_s:
            fsm["_last_rel_t"] = now_m
            if not fsm["cycle_active"]:
                fsm["cycle_active"] = True
                fsm["cycle_activated"] = True
            fsm["deploy_needed"] = True
            print("rising edge detected")
        # Falling edge (Release OFF) -> deactivate cycle + send-to-GCS flag
        if last_pwm > pwm_retract >= pwm: #and (now_m - fsm["_last_ret_t"]) > debounce_s:
            fsm["_last_ret_t"] = now_m
            if fsm["cycle_active"]:
                fsm["cycle_active"] = False
                fsm["cycle_deactivated"] = True
            fsm["send_to_gcs"] = True
            print("falling edge detected")
    fsm["_last_pwm"] = pwm

############################################################################
def process_heartbeat(msg, st):
    """Track ARMED/DISARMED edges and reset init-takeoff on DISARM."""
    base_mode = getattr(msg, 'base_mode', 0)
    custom_mode_id = msg.custom_mode
    custom_mode_name = COPTER_MODES.get(custom_mode_id, f"UNKNOWN({custom_mode_id})")
    armed = bool(base_mode & NAV_SAFETY_ARMED)
    if st['armed'] is None:
        st['armed'] = armed
    elif st['armed'] and not armed:
        st['seen_init'] = False
        st['og_count'] = 0
        st['last_landed'] = NAV_ON_GROUND
        st['awaiting_final_td'] = False
        st['ground_ready'] = True       # require ground before counting next INIT_TAKEOFF
    elif (not st['armed']) and armed:
        st['ground_ready'] = True       # still require ground->air edge
    st['armed'] = armed
    st['auto_mode']=True  # or False ???????????????????????????????????????
    # check if the drone is currently in manual mode 
    if custom_mode_name in ['RTL', 'ACRO', 'ALT_HOLD', 'STABILIZE','LOITER','MANUAL','Unknown']:
        st['auto_mode']=False
    else: # this should be auto mode
        st['auto_mode']=True

def process_statustext(txt, st):
    """expecting a final touchdown."""
    if not txt:
        return
    s = txt.lower()
    if "mission" in s and "complete" in s:
        st['awaiting_final_td'] = True

# =========================
# Winch helpers
# =========================
def hall_raw(c) -> int:
    if adc_sim_flag == 1:
        #print (f"adc value in hall_raw: {c.read()}")
        return int(c.read())
    else:
        return int(c.value)  # ADS1115

def release_win(servo, adc, cfg, st, stop_evt):
    # set the servo position to "open", so that the tether will be released.
    servo.value = -cfg["ROTATION_DIRECTION"] * abs(cfg["RELEASE_PWR"])+cfg["NEUTRAL_POS"]
    t0 = time.time()
    print(f"Release, winch servo open: {servo.value}")
    while not stop_evt.is_set():
        if (time.time() - t0) > cfg["SAFETY_TIMEOUT"]:
            neutral(servo, cfg)
            print("safety timeout triggered during release")
            break
        try:
            dist = hall_raw(adc) - cfg["HALL_TARGET"]
        except Exception:
            neutral(servo, cfg) #return servo to neutral
            time.sleep(0.25)
            break
        #print(f"hall_raw(adc): {hall_raw(adc)}, dist: {dist}")
        if dist > cfg["RETRACT_TH"]:
            st["RETRACTED"] = 0     #set retracted flag to false.
            break
        time.sleep(0.25)
    neutral(servo, cfg) #move servo back to neutral after let the payload fall

def retract_adpative(servo, adc, cfg, st):
    """
    servo retract toward HALL_TARGET, reducing power when getting close to HALL_TARGET.
    """  
    try:
        dist = hall_raw(adc) - cfg["HALL_TARGET"]
    except Exception:
        neutral(servo, cfg)
        time.sleep(0.25)
        return False

    pwr = dist / float(cfg["HALL_MAX"] - cfg["HALL_MIN"])
    pwr = pwr * cfg["PWR_LIMIT"]
    # exponetially reducing the power when the payload is being retracted(?) is shorten
    if pwr > 0.0:
        pwr = math.pow(pwr, 1.0/3.0) # to capture the behavior of the magnetic field
    if (pwr > cfg["PWR_LIMIT"]):
        pwr = cfg["PWR_LIMIT"]
    else:
        pwr = 0
    if dist < (cfg["HALL_TARGET"] + cfg["RETRACT_SETTLE"]):
        # near target → retracted
        if st["RETRACTED"] != 1:
            st["RETRACTED"] = 1
            if (cfg["ROTATION_DIRECTION"] * servo.value) > 0.0:
                neutral(servo, cfg)
            return True
        # already retracted: ensure neutral if still pushing inward
        if (cfg["ROTATION_DIRECTION"] * servo.value) > 0.0:
            neutral(servo, cfg)
    elif (dist < 10000) and ((cfg["ROTATION_DIRECTION"] * servo.value) > 0.0): # target is far and winch is being retracted
        # keep retracting with bounded power
        servo.value = cfg["ROTATION_DIRECTION"] *  pwr + cfg["NEUTRAL_POS"]
    elif dist > cfg["RETRACT_TH"]: # if distance is long but servo is not rotating in the right direction(?) 

        if st["RETRACTED"] != 0:
            st["RETRACTED"] = 0
    return False

def neutral(servo, cfg):
    servo.value = cfg["NEUTRAL_POS"]

def _servo_out_value(msg, ch):
    """Return PWM (µs) for 1-based output channel ch from SERVO_OUTPUT_RAW, else None."""
    port = 0 if ch <= 8 else 1 if ch <= 16 else 2
    if getattr(msg, 'port', 0) != port:
        return None
    idx = ((ch - 1) % 8) + 1
    return getattr(msg, f"servo{idx}_raw", None)

###### BLE...####################################
def _broadcast(x, n):
    return [] if n <= 0 else [x] * n

# use simulator to generate data
data_sim_flag = False
adc_sim_flag = 1 # hall effect sensor using ADC simulator

# =========================
def winch_thread(stop_evt, q_winch, cfg, st):
    try:  
        servo=Servo(cfg["SERVO_PIN"],
            min_pulse_width=0.0009,
            max_pulse_width=0.0021,
            frame_width=0.02,
            pin_factory=PiGPIOFactory(),
            initial_value=cfg["NEUTRAL_POS"])  # try neutral
    except Exception as e:
        print(f"Servo init failed: {e}")
        return
    # set up ADC (HALL SENSOR)
    try:
        if adc_sim_flag == 1: # using simulated ADC           
            adc  = LinkedHallADC(
                servo = servo,
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
            adc = AnalogIn(ads, ADS.P0)
    except Exception as e:
        print(f"ADS1115 init failed: {e}")
        return
    print("winch ready")

    # local timers
    drop_timer = time.time()
    retrieve_timer = time.time()
    try:
        while not stop_evt.is_set():
            # handle incoming command
            try:
                cmd = q_winch.get(timeout=0.25)
            except queue.Empty:
                cmd = None
            if cmd:
                act = (cmd.get("action") or "").upper()
                if act == "RELEASE":
                    #print("RELEASE")
                    release_win(servo, adc, cfg, st, stop_evt)
                    drop_timer = time.time()

                elif act == "RETRACT":
                    dur = float(cmd.get("duration", 0.0))
                    print(f"RETRACT for {dur}s")
                    t0 = time.time()
                    servo.value = cfg["ROTATION_DIRECTION"] * cfg["RETRACT_PWR"]
                    while not stop_evt.is_set() and (time.time() - t0) < dur:
                        retract_flag= retract_adpative(servo, adc, cfg, st)
                        if retract_flag == True:
                            break
                        time.sleep(0.05)
                    neutral(servo, cfg)

                elif act == "NEUTRAL":
                    neutral(servo, cfg)
            time.sleep(0.1)
    except Exception as e:
        print(f"winch thread crashed:{e}")
        neutral(servo, cfg)

    finally:
        neutral(servo, cfg)
        time.sleep(0.1)

########################################################        
def ble_thread(stop_evt, q_ble, q_mav, st):
    ble = st['ble'] = BluetoothReader(st['ble_mutex'])

    try:
        st['c_status'] = 'disconnected'
        while ble is None or not ble.check_connection_status():
            if ble.connect():
                ble.set_lights('navigation')
                logger.debug('connected to sensor, activated lights')
            ble.init_sensor_status()
            time.sleep(0.2)
            ble.set_calibration_pressure()
            time.sleep(0.2)
            #ble.set_maxsize(BLE_SAMPLE_SIZE)
            print (f"set smple_type:{BLE_SAMPLE_TYPE}")
            ble.set_smpl_type(BLE_SAMPLE_TYPE)
            time.sleep(0.1)
            st['c_status'] = 'connected'
        # retry till connect
        last_poll = 0.0
        while not stop_evt.is_set():
            time.sleep(0.2)
            now = time.time()
            # periodic connection maintenance (fixed-rate retry)
            if now - last_poll >= 1:
                last_poll = now
                try:
                    ok = bool(ble.check_connection_status())
                except Exception:
                    ok = False
                if not ok:
                    st['c_status']  = 'disconnected'
                    try:
                        if ble.reconnect():
                            st['c_status']  = 'connected'
                    except Exception:
                        pass
                else:
                    st['c_status']  = 'connected'
                        # Handle queued commands
            try:
                cmd = q_ble.get(timeout=0.25)
                print(f"cmd:{cmd}")
            except queue.Empty:
                cmd = None
            if cmd:
                action = (cmd.get("action") or "").upper()
                if action == 'START':
                    print("BLE action:START")
                    if ble.sdata.get('connection'): 
                        try:
                            # reset sampling counter
                            ble.set_sample_reset()
                            ble.set_sampl_flag(1) # set low threhold to start sampling
                            sflag= ble.get_sampl_flag()
                            print(f"start sampling, sampling flag: {sflag}")
                            s_type=ble.get_smpl_type()
                            print(f"sample_type:{s_type}")
                            st['sampling'] = True
                            st['c_status'] = 'sampling_started'  # or 'reset_done'                       
                        except Exception:
                            st['c_status'] = 'start_failed'
                    else:
                        st['c_status'] = 'start_skipped_disconnected'
                elif action == 'FETCH':
                    print(f"BLE action:FETCH: {ble.sdata.get('connection')}")
                    if ble.sdata.get('connection'):
                        try:
                            ble.set_sampl_flag(0) # stop sampling
                            time.sleep(0.1)
                            sflag= ble.get_sampl_flag()
                            print(f"stop sampling sampling flag: {sflag}")
                            s_size=ble.get_sample_size()
                            ok = bool(ble.get_sample_data())
                            st['sampling'] = False
                            print(f"finish sampling - queue mav cmd to upload data, ok: {ok}, sample_size:{s_size}")
                            if ok:
                                do_list   = ble.sdata.get('do_vals')        or []
                                temp_list = ble.sdata.get('temp_vals')      or []
                                press_list= ble.sdata.get('pressure_vals')  or []
                                n = len(do_list)         # sample length
                                print(f"sampling finished, length:{n}")
                                ts_list = list(range(n)) # generate sampling order

                                st['last_cols'] = {
                                    'time': ts_list,'do': do_list,'temp': temp_list,'press': press_list,
                                    'init_DO':       _broadcast(ble.sdata.get('init_do'), n),
                                    'init_pressure': _broadcast(ble.sdata.get('init_pressure'), n),
                                    'batt_v':        _broadcast(ble.sdata.get('battv'), n),
                                }
                                st['gcs_ready'] = True
                                st['c_status']  = 'fetched'
                                q_mav.put({"action": "sendpayload"}) # set winch to neutral at the end of a cycle
                            else:
                                st['c_status'] = 'fetch_empty'
                        except Exception as e:
                            st['c_status'] = 'fetch_failed'
                            print(f"fetch failed: {e}")
                    else:
                        st['c_status'] = 'fetch_skipped_disconnected'
    
                elif action == 'DISCONNECT':
                    _ble_close(ble)
                    st['c_status'] = 'disconnected_by_request'
    except Exception as e:
        print(f"BLE thread crashed: {e}")

    finally:
        print("BLE thread close")
        _ble_close(ble)   # <-- guarantees graceful shutdown on Ctrl+C

def _ble_close(ble):
    # Stop scanning if in progress
    try: ble.stop_scan()
    except Exception: pass
    # Disconnect if connected
    try:
        conn = getattr(ble, "uart_connection", None)
        if conn and getattr(conn, "connected", False):
            conn.disconnect()
    except Exception: pass
    # Clear flag for your code’s single source of truth
    try:
        ble.sdata['connection'] = False
    except Exception:
        pass

# ---------------- THREAD: MAVLink ----------------
def mav_thread(stop_evt, q_winch, q_ble, q_mav, wincfg, winst,blest):
    # initialize failed flag on startup
    if "failed" not in sensor_state:
        sensor_state["failed"] = load_buffer(BUFFER_PATH)
    
    print(f"MAVLINK: starting (bind {CONN_STR_RX})")
    last_armed = None  # put near other state vars
    try:
        if CONN_STR_RX.startswith(('udp', 'tcp')):
            m_rx = mavutil.mavlink_connection(CONN_STR_RX)
        else:
            m_rx = mavutil.mavlink_connection(CONN_STR_RX, baud=BAUD)
        m_tx = mavutil.mavlink_connection(CONN_STR_TX)

        print("MAVLINK: waiting for HEARTBEAT...")
        hb = m_rx.recv_match(type='HEARTBEAT', blocking=True, timeout=10)
        if not hb:
            print("MAVLINK: no HEARTBEAT in 10s (check simulator IP/port)")
        else:
            print("MAVLINK: connected: sys=", hb.get_srcSystem(), "comp=",hb.get_srcComponent())
        # clean buffer on reconnect
        sensor_state["failed"] = resend_buffer(m_tx, sensor_state.get("failed", {}))

        last_lat = last_lon = None
        last_ping = time.time()

        while not stop_evt.is_set():
            msg = m_rx.recv_match(blocking=True, timeout=0.5)
            now = time.time()
            if (now - last_ping) >= PING_SEC:
                last_ping = now
            if not msg:
                continue

            t = msg.get_type()
            if t == 'GLOBAL_POSITION_INT':
                last_lat = msg.lat / 1e7
                last_lon = msg.lon / 1e7

            elif t == 'HEARTBEAT':
                process_heartbeat(msg, mv_state)

            elif t == 'SERVO_OUTPUT_RAW':
                handle_servo_output_raw(msg, fsm_st)
                flags = fsm_consume_flags(fsm_st)
                if flags["cycle_activated"]:
                    pass
                
                if flags["deploy_needed"]:
                    #Start BLE Sampling
                    print("EVENT: start BLE sampling")
                    q_ble.put({"action": "START"})
                    #engage winch                    
                    q_winch.put({"action": "RELEASE"})
                    # schedule retract without blocking MAVLink loop
                    FREEFALL_SEC=wincfg["FREEFALL_SEC"] # time expected for payload to reach the expected depth
                    RETRACT_SEC=wincfg["RETRACT_SEC"]   # time for the payload to be fully retracted
                    def _enqueue_retract():
                        q_winch.put({"action": "RETRACT", "duration": RETRACT_SEC})
                    _pending_retract_timer = threading.Timer(FREEFALL_SEC, _enqueue_retract)
                    _pending_retract_timer.daemon = True
                    _pending_retract_timer.start()

                if flags["send_to_gcs"]:
                    sensor_state["failed"] = resend_buffer(m_tx, sensor_state.get("failed", {}))
                    # Upload simulator data
                    if data_sim_flag == True:
                        cols=prep_sim_data(csv_path) # create simulation data
                        send_payload(m_tx, cols,sensor_state)  # upload data
                    else:
                        print("MAV->BL: To fetch data from BL sensor")
                        q_ble.put({"action": "FETCH"})
                if flags["cycle_deactivated"]:
                    print("EVENT: cycle_deactivated")
                    q_winch.put({"action": "NEUTRAL"}) # set winch to neutral at the end of a cycle
            
            # process any queued action (mainly from BLE thread
            try:
                cmd = q_mav.get(timeout=0.1)
                print(f"cmd:{cmd}")
            except queue.Empty:
                continue
            action = (cmd.get("action") or "").upper()
            print(f"mav action:{action}")
            if action == 'SENDPAYLOAD': 
                cols = blest["last_cols"]
                print(f"Uploading fetched BL data: cols:{cols}")
                send_payload(m_tx, cols,sensor_state)  # upload data
            
    except Exception as e:
        print(f"MAVLINK thread crashed: {e}")
        print(traceback.format_exc())

# =========================
def main():
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")

    stop_evt = threading.Event()
    q_winch  = queue.Queue()  # WINCH commands
    q_ble    = queue.Queue()  # BLE commands
    q_mav    = queue.Queue()  # MAV commands
    
    t_mav = threading.Thread(target=mav_thread, name="MAVLINK", args=(stop_evt, q_winch, q_ble, q_mav, wParms, winch_st,ble_st))
    t_win = threading.Thread(target=winch_thread, name="WINCH", args=(stop_evt, q_winch, wParms, winch_st))
    t_ble = threading.Thread(target=ble_thread, name="BLE", args=(stop_evt, q_ble, q_mav, ble_st))

    def cleanup(*_):
        print("Stopping…")
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
    # main loop
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Ctrl+C → stopping...")
        stop_evt.set()
    finally:
        cleanup()
    # Wait for clean shutdown
if __name__ == "__main__":
    main()
