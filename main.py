from builtins import range
from pymavlink import mavutil
import threading, queue, time, logging, sys
import ADS1x15
from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero import Servo
import math
from dataclasses import dataclass

from winch_helper   import *
from encoder_helper import *

#SERIAL_DEV = /dev/serial0'   # TELEM2 (adjust if needed)
#SERIAL_BAUD = 115200
# create a mavlink serial instance
#master = mavutil.mavlink_connection(CONN_STR, baud=BAUD)

lock = Threading.lock()

# Threads: MAVLink (TELEM2 or UDP SITL), Winch, Bluetooth sensor
# - Uses only EXTENDED_SYS_STATE for takeoff/landing
# - Sends STATUSTEXT so GCS can see events
# - Simple dicts in queues (no classes, no generics)

# ---------------- CONFIG ----------------
# testing against SITL stream:
CONN_STR = 'udp:0.0.0.0:14551'
# wired to Pixhawk TELEM2:
# CONN_STR = '/dev/serial0'
BAUD = 115200

# Initialize Servo
servo = Servo(17, pin_factory=PiGPIOFactory())
servo.value = 0
# Initialize ADC
adc = ADS1x15.ADS1115(1)
adc.setGain(1)
time.sleep(0.05)
# hall effect settings (calibrate for every system)
HALL_MIN = 1035
HALL_MAX = 12285

# global variables
CYCLE_COUNT = 0
CYCLE_LIMIT = 0
RETRACTED = 1
AUTO_STATE = "idle"
AUTO_DROP = 15  # drop time for auto cycling
AUTO_PWR = 1.0  # retrieve power
ROTATION = -1  # CW, -1 CCW

# ---------------- SHARED QUEUES ----------------
q_winch = queue.Queue()   # holds dicts like {'action': 'RELEASE', 'duration': 5.0}
q_bt_out = queue.Queue()  # holds sensor samples (bytes, dicts, etc.)
stop_evt = threading.Event()

# ---------------- HELPERS ----------------
def handle_takeoff_landing_only(landed_state, st):
    """Detect INIT_TAKEOFF, TAKEOFF, TOUCHDOWN from EXTENDED_SYS_STATE only."""
    now = time.time()
    last = st.get('last')
    seen_init = st.get('seen_init', False)
    evt = None

    if landed_state == mavutil.mavlink.MAV_LANDED_STATE_IN_AIR \
       and last != mavutil.mavlink.MAV_LANDED_STATE_IN_AIR:
        evt = "INIT_TAKEOFF" if not seen_init else "TAKEOFF"
        st['seen_init'] = True
        st['touch_t0'] = None
        st['touch_confirmed'] = False

    if landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND:
        if last != mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND:
            st['touch_t0'] = now
            st['touch_confirmed'] = False
        else:
            if (not st['touch_confirmed']
                and st.get('touch_t0')
                and (now - st['touch_t0']) >= 2.0):
                evt = "TOUCHDOWN"
                st['touch_confirmed'] = True

    st['last'] = landed_state
    return evt

def send_statustext(link, text, sev=mavutil.mavlink.MAV_SEVERITY_INFO):
    """Send STATUSTEXT so it shows in GCS."""
    try:
        link.mav.statustext_send(sev, text.encode('utf-8')[:50])
    except Exception:
        logging.exception("STATUSTEXT send failed")

# ---------------- THREAD: MAVLink ----------------
def mavlink_thread(stop_evt, q_winch):
    logging.info("MAVLINK: connecting %s", CONN_STR)
    if CONN_STR.startswith(('udp:', 'tcp:')):
        m = mavutil.mavlink_connection(CONN_STR)
    else:
        m = mavutil.mavlink_connection(CONN_STR, baud=BAUD)

    hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
    if not hb:
        logging.error("MAVLINK: no HEARTBEAT; check connection")
        return
    m.target_system = hb.get_srcSystem()
    m.target_component = hb.get_srcComponent()
    logging.info("MAVLINK: connected (sys=%s comp=%s)", m.target_system, m.target_component)

    state = {'last': None, 'seen_init': False, 'touch_t0': None, 'touch_confirmed': False}
    last_lat = last_lon = None

    while not stop_evt.is_set():
        msg = m.recv_match(blocking=True, timeout=0.5)
        if not msg:
            continue
        t = msg.get_type()

        if t == 'GLOBAL_POSITION_INT':
            last_lat = msg.lat / 1e7
            last_lon = msg.lon / 1e7

        elif t == 'EXTENDED_SYS_STATE':
            evt = handle_takeoff_landing_only(msg.landed_state, state)
            if evt == "INIT_TAKEOFF":
                logging.info("INIT_TAKEOFF")
                send_statustext(m, "[PI] INIT_TAKEOFF")
            elif evt == "TAKEOFF":
                logging.info("TAKEOFF")
                send_statustext(m, "[PI] TAKEOFF")
            elif evt == "TOUCHDOWN":
                logging.info("TOUCHDOWN")
                if last_lat is not None and last_lon is not None:
                    logging.info("Touchdown GPS: %.7f,%.7f", last_lat, last_lon)
                    send_statustext(m, f"[PI] TD {last_lat:.5f},{last_lon:.5f}")
                # Example: queue winch commands
                q_winch.put({'action': 'RELEASE', 'duration': 5.0})
                q_winch.put({'action': 'RETRACT', 'duration': 5.0})

    logging.info("MAVLINK: stopping")
    try: m.close()
    except: pass

# ---------------- THREAD: Winch ----------------
def winch_thread(stop_evt, q_winch):
    logging.info("WINCH: init hardware")
    while not stop_evt.is_set():
        try:
            cmd = q_winch.get(timeout=0.2)
        except queue.Empty:
            continue

        action = cmd.get('action')
        duration = float(cmd.get('duration', 0.0))
        logging.info("WINCH: %s (%.1fs)", action, duration)

        try:
            if action == 'RELEASE':
                # TODO: set motor release
                time.sleep(max(0.0, duration))
                # TODO: stop motor
            elif action == 'RETRACT':
                # TODO: set motor retract
                time.sleep(max(0.0, duration))
                # TODO: stop motor
            elif action == 'STOP':
                # TODO: immediate stop
                pass
        except Exception:
            logging.exception("WINCH: error")
        finally:
            q_winch.task_done()

    logging.info("WINCH: shutdown")
    # TODO: safe motor off

# ---------------- THREAD: Bluetooth -------------
def bluetooth_thread(stop_evt, q_bt_out):
    logging.info("BT: starting")
    while not stop_evt.is_set():
        try:
            # TODO: connect/read real sensor
            sample = b"sensor_payload"
            q_bt_out.put(sample)
            time.sleep(1.0)
        except Exception:
            logging.exception("BT: error; retry soon")
            time.sleep(2.0)
    logging.info("BT: stopping")

# ---------------- MAIN ----------------
def main():
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(threadName)s: %(message)s",
        stream=sys.stdout,
    )

    t_mav = threading.Thread(target=mavlink_thread,   name="MAVLINK",   args=(stop_evt, q_winch), daemon=True)
    t_win = threading.Thread(target=winch_thread,     name="WINCH",     args=(stop_evt, q_winch), daemon=True)
    t_bt  = threading.Thread(target=bluetooth_thread, name="BT",        args=(stop_evt, q_bt_out), daemon=True)

    t_mav.start(); t_win.start(); t_bt.start()
    logging.info("Threads started. Ctrl+C to exit.")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        logging.info("Shutting down...")
        stop_evt.set()
        t_mav.join(timeout=3); t_win.join(timeout=3); t_bt.join(timeout=3)
        logging.info("Bye.")

if __name__ == "__main__":
    main()

'''
set stream rate on an APM
'''
'''
def wait_heartbeat(m):
    '''wait for a heartbeat so we know the target system IDs'''
    print("Waiting for APM heartbeat")
    m.wait_heartbeat()
    print("Heartbeat from APM (system %u component %u)" % (m.target_system, m.target_system))

def show_messages(m):
    '''show incoming mavlink messages'''
    while True:
        msg = m.recv_match(blocking=True)
        if not msg:
            return
        elif msg.get_type() == "BAD_DATA":
            pass
        elif msg.get_Type() == "":
            pass
        else:
            print(msg)

# wait for the heartbeat msg to find the system ID
wait_heartbeat(master)

##### LOGGING #####
logging.basicConfig(
    format="%(asctime)s %(levelname)s: %(message)s",
    filename="winchlog.log",
    encoding="utf-8",
    level=logging.INFO,
)
logger = logging.getLogger(__name__)
logger.info("Starting")

print("starting")
last_time = time.time()
test_data = [i for i in range(96)]
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

    #################################################################
    ############################  data send #########################
    if (time.time() - last_time) > 1:
        last_time = time.time()
        master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_WINCH, mavutil.mavlink.MAV_AUTOPILOT_INVALID, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 0, mavutil.mavlink.MAV_STATE_ACTIVE)
        #def: winch_status_send(self, time_usec: int, line_length: float, speed: float, tension: float, voltage: float, current: float, temperature: int, status: int, force_mavlink1: bool = False) -> None:
        
        master.mav.winch_status_send(0,1,2,3,4,5,6,7)
        master.mav.data96_send(mavutil.mavlink.MAV_TYPE_WINCH, 3, test_data)

# stream_rate = 1
# print("Sending all stream request for rate %u" % stream_rate)
# for i in range(0, 3):
#     master.mav.request_data_stream_send(master.target_system, master.target_component,
#                                         mavutil.mavlink.MAV_DATA_STREAM_ALL, stream_rate, 1)
# show_messages(master)
'''
# ---------- messaging ----------

