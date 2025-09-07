import threading
import adafruit_ads1x15
from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero import Servo
import time
import logging
import math

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

##### LOGGING #####
logging.basicConfig(
    format="%(asctime)s %(levelname)s: %(message)s",
    filename="winchlog.log",
    encoding="utf-8",
    level=logging.INFO,
)
logger = logging.getLogger(__name__)
logger.info("Starting")

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

def winch_control():
    """
    Low-level winch control. Sets motor power and retract state.
    """
    global RETRACTED
    dist = adc.readADC(0)
    target_dist = HALL_MIN
    pwr_limit = 0.3
    while True:
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

def state_machine():
    """
    High-level winch state machine.
    """
    global RETRACTED, CYCLE_COUNT, CYCLE_LIMIT
    global AUTO_STATE, AUTO_PWR
    drop_timer = time.time()
    retrieve_timer = time.time()
    while True:
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