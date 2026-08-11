import threading
import time
import logging
import math
from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero import Servo
import ADS1x15
from pymavlink import mavutil

lock = threading.Lock()

# Initialize Servo
servo = Servo(17, pin_factory=PiGPIOFactory())
with lock:
    servo.value = 0

# Initialize ADC
adc = ADS1x15.ADS1115(1)
adc.setGain(1)
time.sleep(0.05)

# Hall effect settings
HALL_MIN = 1035
HALL_MAX = 12285

# Shared state
RETRACTED = 1
AUTO_STATE = "idle"
CYCLE_COUNT = 0
CYCLE_LIMIT = 0
AUTO_DROP = 15
ROTATION = -1
AUTO_PWR = 1.0

# Logging
logging.basicConfig(format="%(asctime)s %(levelname)s: %(message)s", filename="winchlog.log", encoding="utf-8", level=logging.INFO)
logger = logging.getLogger(__name__)
logger.info("Starting")

# MAVLink setup
master = mavutil.mavlink_connection('/dev/serial0', baud=115200)
print("Waiting for APM heartbeat")
master.wait_heartbeat()
print(f"Heartbeat from APM (system {master.target_system})")

def release():
    with lock:
        servo.value = -ROTATION * 0.4
    safety_timer = time.time()
    while True:
        with lock:
            if RETRACTED == 1 and (time.time() - safety_timer) > 0.7:
                servo.value = 0
                print("safety timer triggered")
                logger.warning("safety timer on release")
                break
        time.sleep(0.05)
    with lock:
        servo.value = 0

def winch_control():
    global RETRACTED
    target_dist = HALL_MIN
    pwr_limit = 0.3
    while True:
        try:
            with lock:
                dist = adc.readADC(0) - target_dist
        except:
            with lock:
                servo.value = 0
            logger.warning("failed to read hall effect")
            time.sleep(5)
            continue

        pwr = dist / (HALL_MAX - HALL_MIN)
        pwr = pwr * pwr_limit
        if pwr > 0:
            pwr = math.pow(pwr, 1 / 3)

        pwr = max(0, min(pwr, pwr_limit))

        with lock:
            if dist < (target_dist + 50):
                RETRACTED = 1
                if (ROTATION * servo.value) > 0.0:
                    servo.value = 0
            elif dist < 10000 and (ROTATION * servo.value) > 0.0:
                servo.value = ROTATION * pwr
            elif dist > 8000:
                if RETRACTED != 0:
                    RETRACTED = 0

def state_machine():
    global RETRACTED, CYCLE_COUNT, CYCLE_LIMIT, AUTO_STATE
    #global AUTO_PWR
    drop_timer = time.time()
    retrieve_timer = time.time()
    while True:
        with lock:
            state = AUTO_STATE

        if state == "idle":
            pass
        elif state == "released":
            if AUTO_DROP < (time.time() - drop_timer):
                with lock:
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
        elif state == "retrieving":
            with lock:
                if RETRACTED:
                    AUTO_STATE = "retracted"
                    CYCLE_COUNT += 1
                    print(f"finished cycle {CYCLE_COUNT} of {CYCLE_LIMIT}")
                    logger.info("cycle finished: %s", CYCLE_COUNT)
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
        elif state == "retracted":
            release()
            drop_timer = time.time()
            with lock:
                AUTO_STATE = "released"

print("starting")
last_time = time.time()
test_data = [i for i in range(96)]

wcontrol = threading.Thread(target=winch_control)
smachine = threading.Thread(target=state_machine)
wcontrol.start()
smachine.start()

while True:
    cmd = input().split(",")
    if len(cmd) < 1:
        continue
    elif len(cmd) == 2 and cmd[0] == "s":
        val = float(cmd[1])
        print("setting servo to:", val)
        with lock:
            servo.value = val
    elif cmd[0] == "q":
        print("stopping")
        with lock:
            AUTO_STATE = "idle"
            servo.value = 0
    elif cmd[0] == "r":
        print("releasing")
        release()
    elif cmd[0] == "p":
        with lock:
            print(f"servo pwr: {servo.value}")
            print(f"hall efct: {adc.readADC(0)}")
            print(f"    state: {AUTO_STATE}")
            print(f"retracted: {'yes' if RETRACTED == 1 else 'no'}")
    elif cmd[0] == "start":
        with lock:
            CYCLE_COUNT = 0
            CYCLE_LIMIT = int(cmd[1]) if len(cmd) == 2 else 1
            AUTO_STATE = "retracted"
        print(f"running test for {CYCLE_LIMIT} cycles")

    if (time.time() - last_time) > 1:
        last_time = time.time()
        with lock:
            master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_WINCH, mavutil.mavlink.MAV_AUTOPILOT_INVALID, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 0, mavutil.mavlink.MAV_STATE_ACTIVE)
            master.mav.winch_status_send(0, 1, 2, 3, 4, 5, 6, 7)
            master.mav.data96_send(mavutil.mavlink.MAV_TYPE_WINCH, 3, test_data)