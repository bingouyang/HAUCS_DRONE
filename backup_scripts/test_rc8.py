from pymavlink import mavutil
import time

PORT = "/dev/serial0"
BAUD = 115200

RC_CH = 8
THRESH_HIGH = 1800
THRESH_LOW = 1300

def main():
    print("Connecting to MAVLink on %s at %d..." % (PORT, BAUD))

    m = mavutil.mavlink_connection(PORT, baud=BAUD)
    m.wait_heartbeat()

    print("Connected.")
    print("Monitoring RC%d..." % RC_CH)

    last_state = None
    last_pwm = None

    while True:
        msg = m.recv_match(type="RC_CHANNELS", blocking=True, timeout=2)

        if msg is None:
            print("No RC_CHANNELS message received...")
            continue

        pwm = getattr(msg, "chan%d_raw" % RC_CH)

        if pwm == 0 or pwm == 65535:
            continue

        if pwm > THRESH_HIGH:
            state = "HIGH"
        elif pwm < THRESH_LOW:
            state = "LOW"
        else:
            state = "MID"

        if state != last_state:
            print("%s RC%d changed: pwm=%d state=%s" %
                  (time.strftime("%Y-%m-%d %H:%M:%S"), RC_CH, pwm, state))

            if state == "HIGH" and last_state != "HIGH":
                print("WINCH EVENT: RC8 pressed / triggered")

        last_state = state
        last_pwm = pwm

if __name__ == "__main__":
    main()
