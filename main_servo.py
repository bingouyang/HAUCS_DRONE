from pymavlink import mavutil
import threading, queue, time, logging, sys
import traceback
from builtins import range
import math
from dataclasses import dataclass
import signal

import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero import Servo
from winch_helper   import *
from encoder_helper import *
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
#CONN_STR_TX = 'udpout:10.113.32.16:14555'
CONN_STR_TX = 'udpout:192.168.1.160:14555'
# wired to Pixhawk TELEM2:
# CONN_STR = '/dev/serial0'
#BAUD = 115200

############ Mavlink helper #####################################
# =========================
# Mavlink constants
NAV_IN_AIR       = mavutil.mavlink.MAV_LANDED_STATE_IN_AIR
NAV_ON_GROUND    = mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND
NAV_LANDING      = mavutil.mavlink.MAV_LANDED_STATE_LANDING
NAV_SAFETY_ARMED = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED

PING_SEC = 2          # interval to check heartbeat
TOUCH_CONFIRM_SEC = 2    # interval to be sure drone is touch down

# Winch constants
winchParms = {
    "SERVO_PIN": 17,
    "ADC_PIN": 0,                 # ADS1115 A0
    "HALL_MIN": 1035,
    "HALL_MAX": 12285,
    "HALL_TARGET": 1035,          # retracted target
    "RELEASE_PWR": -0.4,          # release power
    "RETRACT_PWR": 0.30,          # retract power
    "PWR_LIMIT":   0.30,          # cap retract power
    "NEUTRAL_POS": -0.04,         # adjust for servo creeping
    "ROTATION_DIRECTION": -1,     # -1 or +1 depending on wiring
    "SAFETY_TIMEOUT": 0.7,
    "RETRACT_SETTLE": 50,         # tolerance near target (raw units)
    "RETRACT_TH": 8000,
    "PWD_ADP_TH": 10000,          # adjust power when below this threshold(?)
    "LOG_PREFIX": "[WINCH] ",
    "FREEFALL_SEC":  20,           # how long to wait after release the winch
    "RETRACT_SEC":   35,           # how long to wait for the winch to fully retracted. 
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

# =========================
# Winch status
# ---- Winch FSM (flags only; no actions) ----

import time

winch_st = {
    # state
    "RETRACTED": 1,          # 1=retracted, 0=extended (update elsewhere if you want)
}

########################## FSM triggerd by servo ##############################
fsm_st = {
    "cycle_active": False,        # persistent: True after rising, False after falling

    # one-shot flags (you consume & clear)
    "deploy_needed": False,       # set on rising edge
    "send_to_gcs_needed": False,  # set on falling edge
    "cycle_activated": False,     # set when cycle_active becomes True
    "cycle_deactivated": False,   # set when cycle_active becomes False

    # internals (private)
    "_last_pwm": None,
    "_last_rel_t": 0.0,
    "_last_ret_t": 0.0,
}

# Trigger detection (RC or mission)
WINCH_CH = 9          # output channel for the "winch"
PWM_RELEASE = 1800    # rising through => Release ON
PWM_RETRACT = 1200    # falling through => Release OFF
DEBOUNCE_S = 0.1
    
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
    """Read & clear one-shot flags."""
    keys = ("deploy_needed", "send_to_gcs_needed", "cycle_activated", "cycle_deactivated")
    out = {k: fsm.get(k, False) for k in keys}
    for k in keys:
        fsm[k] = False
    return out

def on_rising_edge(fsm, dwell_s=None, now=None):
    """
    Rising edge servo signal:
      - mark cycle active
      - raise deploy_needed
    """
    fsm["cycle_active"] = True
    fsm["deploy_needed"] = True

def on_airborne_true(fsm):
    """
    First transition to airborne during an active cycle:
      - raise ble_upload_needed once per cycle
    """
    if fsm["cycle_active"] and not fsm["ble_started"]:
        fsm["ble_started"] = True
        fsm["ble_upload_needed"] = True

def on_falling_edge(fsm):
    """
    Falling edge of servo signal:
      - raise send_to_gcs_needed
      - reset persistent cycle state
    """
    fsm["send_to_gcs_needed"] = True
    fsm_reset_cycle(fsm)

def fsm_reset_cycle(fsm):
    #Reset persistent state
    fsm["cycle_active"] = False
    fsm["ble_started"] = False
    fsm["retract_deadline"] = None

def handle_servo_output_raw(msg, fsm,
                            ch=WINCH_CH,
                            pwm_release=PWM_RELEASE,
                            pwm_retract=PWM_RETRACT,
                            debounce_s=DEBOUNCE_S):
    """Edge detector: sets deploy/send flags and cycle activate/deactivate flags."""
    # lazy init
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
            fsm["send_to_gcs_needed"] = True
            print("falling edge detected")

    fsm["_last_pwm"] = pwm

############################################################################
def process_heartbeat(msg, st):
    """Track ARMED/DISARMED edges and reset init-takeoff on DISARM."""
    base_mode = getattr(msg, 'base_mode', 0)
    custom_mode_id = msg.custom_mode
    custom_mode_name = COPTER_MODES.get(custom_mode_id, f"UNKNOWN({custom_mode_id})")
    #print(f"custom_mode_id:{custom_mode_id} custom_mode: {custom_mode_name}")
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
        #print(f"Current mode: {custom_mode_name} - (Manual), don't deploy winch")
        st['auto_mode']=False
    else: # this should be auto mode
        st['auto_mode']=True
        #print(f"Current mode: {custom_mode_name}")

def process_statustext(txt, st):
    """expecting a final touchdown."""
    if not txt:
        return
    s = txt.lower()
    if "mission" in s and "complete" in s:
        st['awaiting_final_td'] = True

def handle_extsys_event(landed_state, st, need_confirm=2):
    """
    Returns one of: INIT_TAKEOFF, TAKEOFF, LANDING_START, TOUCHDOWN, TOUCHDOWN_FINAL, or None.
    - LANDING_START - descent when state enters LANDING.
    - TOUCHDOWN - ground contact (rate-proof via consecutive ON_GROUND).
    """
    last = st['last_landed']
    evt = None

    # --- LANDING start (edge) ---
    if landed_state == NAV_LANDING and last != NAV_LANDING:
        # Only treat as sampling if we are NOT expecting the final touchdown
        if not st['awaiting_final_td'] and not st['landing_fired']:
            evt = "LANDING_START"          # <-- use this to start sampling
            st['landing_fired'] = True

    # --- ON_GROUND counting for touchdown ---
    if landed_state == NAV_ON_GROUND:
        st['og_count'] = st['og_count'] + 1 if last == NAV_ON_GROUND else 1
        st['ground_ready'] = True
        if not st['td_fired'] and st['og_count'] >= need_confirm:
            if st['awaiting_final_td'] or st['auto_mode']==False: # if we are not 
                evt = "TOUCHDOWN_FINAL"
                print("final landed or not in AUTO mode, don't deploy winch")
            elif st['auto_mode']==True: # only do this if we are in auto mode.
                evt = "TOUCHDOWN"
                print("sampling landed in AUTO mode, deploy winch")
            st['td_fired'] = True
            # if this was final TD, prep next mission to start fresh
            if st['awaiting_final_td']:
                st['seen_init'] = False
                st['awaiting_final_td'] = False
                st['ground_ready'] = True
    else:
        # left ground → allow next touchdown to fire again
        st['og_count'] = 0
        if st['td_fired']:
            st['td_fired'] = False

    # Reset LANDING gate when we leave LANDING (so it can fire next time)
    if last == NAV_LANDING and landed_state != NAV_LANDING:
        st['landing_fired'] = False

    # --- liftoff  ---
    if landed_state == NAV_IN_AIR and last != NAV_IN_AIR:
        if not st['seen_init'] and st['ground_ready']:
            evt = "INIT_TAKEOFF"
            st['seen_init']   = True
            st['ground_ready'] = False
        else:
            evt = "TAKEOFF"

    st['last_landed'] = landed_state
    return evt

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
    #print("Retract")

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
    #
    #print(f"hall_raw(adc): {hall_raw(adc)}, dist: {dist}, servo power:{pwr}")

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
    elif (dist < 10_000) and ((cfg["ROTATION_DIRECTION"] * servo.value) > 0.0): # target is far and winch is being retracted
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

# use mavlink simulator
mav_sim_flag = 1
# using ADC simulator
adc_sim_flag = 1
# =========================
def winch_thread(stop_evt, q_winch, cfg, st):
    try:
        # setup servo
        # if sim_flag == 1: 
        #     servo = ServoSim()  # to use servo simulator
        # else:
        #servo = Servo(cfg["SERVO_PIN"],
        #    MIN_PW = 0.0009,   # 900 µs
        #    MAX_PW = 0.0021,   # 2100 µs
        #    pin_factory=PiGPIOFactory()
        #)        
        servo=Servo(cfg["SERVO_PIN"],
            min_pulse_width=0.0009,
            max_pulse_width=0.0021,
            frame_width=0.02,
            pin_factory=PiGPIOFactory(),
            initial_value=cfg["NEUTRAL_POS"])  # try neutral
        #servo.value =  cfg["NEUTRAL_POS"] # correct servo creep...
        print(f"initialize servo power to neutral:{cfg['NEUTRAL_POS']}")
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
        time.sleep(0.2)

# ---------------- THREAD: MAVLink ----------------
def mavlink_thread(stop_evt, q_winch, wincfg, winst):
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

            elif t == 'EXTENDED_SYS_STATE':
                evt = handle_extsys_event(msg.landed_state, mv_state, need_confirm=2)
                '''if evt == "TOUCHDOWN":
                    print("EVENT: TOUCHDOWN (intermediate)")
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
                '''
                if evt == "TOUCHDOWN_FINAL":
                    print("EVENT: TOUCHDOWN (final)")
                    # set winch to neutral
                    q_winch.put({"action": "NEUTRAL"})                    
                elif evt == "INIT_TAKEOFF":
                    print("EVENT: INIT_TAKEOFF")
            
            elif t == 'SERVO_OUTPUT_RAW':
                handle_servo_output_raw(msg, fsm_st)
                # Consume exactly once per loop; this clears the one-shot flags
                flags = fsm_consume_flags(fsm_st)
                if flags["cycle_activated"]:
                    pass
                    # infor BT
                if flags["deploy_needed"]:
                    print("EVENT: start BT sampling")
                    # lock in lat/lng
                    
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

                if flags["send_to_gcs_needed"]:
                    sensor_state["failed"] = resend_buffer(m_tx, sensor_state.get("failed", {}))
                    # Request data from BL sensor
                    if mav_sim_flag == True:
                        cols=prep_sim_data(csv_path) # create simulation data                    
                    send_payload(m_tx, cols,sensor_state)  # main step
                if flags["cycle_deactivated"]:
                    print("EVENT: cycle_deactivated")

    except Exception as e:
        print(f"MAVLINK thread crashed: {e}")
        print(traceback.format_exc())

# ---------------- THREAD: Bluetooth WIP -------------
def bluetooth_thread(stop_evt, q_bt_out):
    print("BT: starting")
    while not stop_evt.is_set():
        try:
            print("dummy:connect/read real sensor")
        except Exception as e:
            print(f"BT thread crashed: {e}")
            print(traceback.format_exc())
# =========================
# Main
# =========================
def main():
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")

    stop_evt = threading.Event()
    q_winch  = queue.Queue()

    t_win = threading.Thread(target=winch_thread,  name="WINCH",   args=(stop_evt, q_winch, winchParms, winch_st))
    t_mav = threading.Thread(target=mavlink_thread, name="MAVLINK", args=(stop_evt, q_winch, winchParms, winch_st))

    def cleanup(*_):
        print("Stopping…")
        stop_evt.set()
        t_mav.join(timeout=5)
        t_win.join(timeout=5)
        sys.exit(0)

    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    t_win.start()
    t_mav.start()
    
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
