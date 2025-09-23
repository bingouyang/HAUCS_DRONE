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

# ---------------- CONFIG ----------------
# testing against simulator event stream:
CONN_STR = 'udp:0.0.0.0:14551'
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

##### LOGGING #####
logging.basicConfig(
    format="%(asctime)s %(levelname)s: %(message)s",
    filename="companion_computer.log",
    encoding="utf-8",
    level=logging.INFO,
)
logger = logging.getLogger(__name__)
logger.info("Starting")

def process_heartbeat(msg, st):
    """Track ARMED/DISARMED edges and reset init-takeoff on DISARM."""
    base_mode = getattr(msg, 'base_mode', 0)
    armed = bool(base_mode & NAV_SAFETY_ARMED)
    if st['armed'] is None:
        st['armed'] = armed
    elif st['armed'] and not armed:
        # ARMED -> DISARMED: reset for next mission
        st['seen_init'] = False
        st['og_count'] = 0
        st['last_landed'] = NAV_ON_GROUND
        st['awaiting_final_td'] = False
        st['ground_ready'] = True       # require ground before counting next INIT_TAKEOFF
    elif (not st['armed']) and armed:
        # DISARMED -> ARMED
        st['ground_ready'] = True       # still require ground->air edge
    st['armed'] = armed

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
            evt = "TOUCHDOWN_FINAL" if st['awaiting_final_td'] else "TOUCHDOWN"
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
    if last == NAV_LANDINGG and landed_state != NAV_LANDING:
        st['landing_fired'] = False

    # --- liftoff edges ---
    if landed_state == NAV_IN_AIR and last != NAV_IN_AIR:
        if not st['seen_init'] and st['ground_ready']:
            evt = "INIT_TAKEOFF"
            st['seen_init']   = True
            st['ground_ready'] = False
        else:
            evt = "TAKEOFF"

    st['last_landed'] = landed_state
    return evt

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
}

# =========================
# Winch status
winch_st = {
    "RETRACTED": 1,             # 1 = retracted, 0 = extended
}

# =========================
# Winch helpers
# =========================
def hall_raw(c) -> int:
    if sim_flag == 1:
        print (f"adc value in hall_raw: {c.read()}")
        return int(c.read())
    else:
        return int(c.value)  # ADS1115

def release_win(servo, adc, cfg, st, stop_evt):
    # set the servo position to "open", so that the tether will be released.
    servo.value = -cfg["ROTATION_DIRECTION"] * abs(cfg["RELEASE_PWR"])+cfg["NEUTRAL_POS"]
    t0 = time.time()
    print(f"Release, servo open: {servo.value}")
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
        print(f"hall_raw(adc): {hall_raw(adc)}, dist: {dist}")
        if dist > cfg["RETRACT_TH"]:
            st["RETRACTED"] = 0     #set retracted flag to false.
            break
        time.sleep(0.25)
    neutral(servo, cfg) #move servo back to neutral after let the payload fall

def retract_adpative(servo, adc, cfg, st):
    """
    servo retract toward HALL_TARGET, reducing power when getting close to HALL_TARGET.
    """  
    print(f"Retract")

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
        pwr = math.pow(pwr, 1.0/3.0)
    if (pwr > cfg["PWR_LIMIT"]):
        pwr = cfg["PWR_LIMIT"]
    else:
        pwr = 0
    #
    print(f"hall_raw(adc): {hall_raw(adc)}, dist: {dist}, servo power:{pwr}")

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

# using ADC simulator
sim_flag = 1

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
        if sim_flag == 1: # using simulated ADC           
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
    print("MAVLINK: starting (bind %s)", CONN_STR)
    last_armed = None  # put near other state vars
    try:
        if CONN_STR.startswith(('udp:', 'tcp:')):
            m = mavutil.mavlink_connection(CONN_STR)
        else:
            m = mavutil.mavlink_connection(CONN_STR, baud=BAUD)

        print("MAVLINK: waiting for HEARTBEAT...")
        hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=10)
        if not hb:
            print("MAVLINK: no HEARTBEAT in 10s (check simulator IP/port)")
        else:
            print("MAVLINK: connected: sys=", hb.get_srcSystem(), "comp=",hb.get_srcComponent())

        last_lat = last_lon = None
        last_ping = time.time()

        while not stop_evt.is_set():
            msg = m.recv_match(blocking=True, timeout=0.5)
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

            elif t == 'STATUSTEXT':
                txt = getattr(msg, 'text', '') or getattr(msg, 'message', b'').decode('utf-8','ignore')
                process_statustext(txt, mv_state)
                print("STATUSTEXT:", txt)
            
            elif t == 'EXTENDED_SYS_STATE':
                evt = handle_extsys_event(msg.landed_state, mv_state, need_confirm=2)

                if evt == "LANDING_START":
                    print("EVENT: LANDING_START (sampling) -> start BT sampling")
                    # Notify the BL to begin sampling

                elif evt == "TOUCHDOWN":
                    print("EVENT: TOUCHDOWN (intermediate)")
                    #lock in the GPS and look up the Pond ID.
                    
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

                elif evt == "TOUCHDOWN_FINAL":
                    print("EVENT: TOUCHDOWN (final)")
                    # at end of mission - check to stop all ...

                    # set winch to neutral
                    q_winch.put({"action": "NEUTRAL"})
                    

                elif evt == "INIT_TAKEOFF":
                    print("EVENT: INIT_TAKEOFF")

                elif evt == "TAKEOFF":
                    print("EVENT: SAMPLING TAKEOFF")
                    # Request data from BL sensor
                    
    except Exception as e:
        print("MAVLINK thread crashed: %s", e)
        print(traceback.format_exc())

# ---------------- THREAD: Bluetooth WIP -------------
def bluetooth_thread(stop_evt, q_bt_out):
    print("BT: starting")
    while not stop_evt.is_set():
        try:
            print("dummy:connect/read real sensor")
        except Exception:
            print("BT thread crashed: %s", e)
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
