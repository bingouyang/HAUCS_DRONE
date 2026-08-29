#!/usr/bin/env python3
"""
test_winch.py
-------------
Combined servo + Hall effect sensor diagnostic for HAUCS winch.

Drive the servo manually and watch the Hall sensor respond in real time.
Use this to verify:
  - Hall sensor wiring and range
  - Motor direction (extend should INCREASE Hall value)
  - Polarity (if Hall goes wrong direction, swap Hall wires or flip ROTATION_DIRECTION)
  - Servo neutral position

Controls:
  r  = release (extend) at RELEASE_PWR
  t  = retract at RETRACT_PWR
  z  = stop pulses (servo off, draws nothing)
  a  = nudge more negative (finer control)
  d  = nudge more positive (finer control)
  q  = quit

Press Ctrl+C or q to exit.
"""

import time, sys, termios, tty, select

# ── CONFIG (must match wParms in main_rc8_uart_parm.py) ──────────────────────
SERVO_PIN        = 17
MIN_PW           = 0.0009
MAX_PW           = 0.0021
FRAME            = 0.02
NEUTRAL_POS      = 0.0
ROTATION_DIR     = -1      # -1 or 1; flip if motor runs backwards
# 082526: were RELEASE_PWR 0.10 / RETRACT_PWR 0.30, which with the extra sign
# flip on retract below made r and t drive the winch OPPOSITE to the flight
# code: r sent -0.10 (the flight code's retract) and t sent +0.30 (its
# release). Any direction or polarity conclusion from this script was backwards.
# Now matched to wParms in main_rc8_uart_parm.py.
RELEASE_PWR      = -0.30   # extend (release); wParms RELEASE_PWR
RETRACT_PWR      = 0.10    # retract;          wParms RETRACT_PWR

ADC_CHANNEL      = 0
HALL_TARGET      = 2500    # expected when fully retracted
HALL_MAX         = 12285   # expected when fully extended
RETRACT_TH       = 8000    # considered "extended"
RETRACT_SETTLE   = 50      # tolerance for "fully retracted"

SAMPLE_HZ        = 10      # Hall readings per second
NUDGE_STEP       = 0.01    # step size for a/d keys
# ─────────────────────────────────────────────────────────────────────────────

def init_servo():
    from gpiozero import Servo
    from gpiozero.pins.pigpio import PiGPIOFactory
    factory = PiGPIOFactory()
    srv = Servo(SERVO_PIN,
                min_pulse_width=MIN_PW,
                max_pulse_width=MAX_PW,
                frame_width=FRAME,
                pin_factory=factory,
                initial_value=NEUTRAL_POS)
    return srv

def init_adc():
    import board, busio
    import adafruit_ads1x15.ads1115 as ADS
    from adafruit_ads1x15.analog_in import AnalogIn
    i2c = busio.I2C(board.SCL, board.SDA)
    ads = ADS.ADS1115(i2c)
    return AnalogIn(ads, ADC_CHANNEL)

def hall_read(adc):
    return int(adc.value)

def bar(val, lo, hi, width=24):
    frac = max(0.0, min(1.0, (val - lo) / float(hi - lo)))
    filled = int(frac * width)
    return "[" + "#" * filled + "-" * (width - filled) + "]"

def hall_status(val):
    if val <= HALL_TARGET + RETRACT_SETTLE:
        return "RETRACTED"
    elif val >= RETRACT_TH:
        return "EXTENDED "
    else:
        return "MID      "

def servo_usec(v):
    pw = MIN_PW + ((v + 1.0) / 2.0) * (MAX_PW - MIN_PW)
    return int(round(pw * 1e6))

def main():
    print()
    print("=" * 60)
    print("  HAUCS Winch Test: Servo and Hall Sensor")
    print("=" * 60)
    print("  ROTATION_DIR : %d" % ROTATION_DIR)
    print("  RELEASE_PWR  : %.2f  (servo value %.3f)" % (
        RELEASE_PWR, ROTATION_DIR * RELEASE_PWR + NEUTRAL_POS))
    print("  RETRACT_PWR  : %.2f  (servo value %.3f)" % (
        RETRACT_PWR, ROTATION_DIR * RETRACT_PWR + NEUTRAL_POS))   # 082526
    print("  HALL_TARGET  : %d  (retracted)" % HALL_TARGET)
    print("  RETRACT_TH   : %d  (extended)" % RETRACT_TH)
    print()
    print("  Controls:")
    print("    r = RELEASE (extend) at RELEASE_PWR")
    print("    t = RETRACT at RETRACT_PWR")
    print("    z = STOP PULSES (servo off - the flight code's neutral())")
    print("    a = nudge negative   d = nudge positive")
    print("    q = quit")
    print()
    print("  Expected when extending : Hall value INCREASES")
    print("  Expected when retracting: Hall value DECREASES")
    print("  If REVERSED: flip ROTATION_DIR in config or swap Hall wires")
    print("=" * 60)
    print()

    print("Initialising servo...", end=" ")
    try:
        srv = init_servo()
        print("OK  (GPIO %d, neutral)" % SERVO_PIN)
    except Exception as e:
        print("FAILED: %s" % e)
        sys.exit(1)

    print("Initialising ADS1115...", end=" ")
    try:
        adc = init_adc()
        first = hall_read(adc)
        print("OK  (first reading: %d)" % first)
        if first < 100:
            print("WARNING: Hall reading very low — check sensor power/wiring")
        elif first > 15000:
            print("WARNING: Hall reading very high — sensor may be saturated")
    except Exception as e:
        print("FAILED: %s" % e)
        srv.detach()
        sys.exit(1)

    print()
    print("  %-6s  %-8s %-7s  %-6s  %-26s  %-10s  %s" % (
        "SERVO", "ACTION", "PWM(us)", "HALL", "POSITION", "STATE", "DIRECTION"))
    print("  " + "-" * 80)

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setcbreak(fd)

    v = NEUTRAL_POS
    srv.value = v
    prev_hall = None
    interval = 1.0 / SAMPLE_HZ

    try:
        while True:
            # non-blocking key check
            dr, _, _ = select.select([sys.stdin], [], [], interval)
            if dr:
                ch = sys.stdin.read(1).lower()
                if ch == 'q':
                    break
                elif ch == 'r':
                    v = ROTATION_DIR * RELEASE_PWR + NEUTRAL_POS
                elif ch == 't':
                    # 082526: was -ROTATION_DIR, an extra flip the flight
                    # code does not have.
                    v = ROTATION_DIR * RETRACT_PWR + NEUTRAL_POS
                elif ch == 'z':
                    # 082526: neutral() in the flight code stops the pulse
                    # train rather than holding 1500us, because a servo told to
                    # hold still still draws current and heats. Same here.
                    v = None
                elif ch == 'a':
                    # 082526: v is None after z, so nudge from neutral.
                    v = max(-1.0, (NEUTRAL_POS if v is None else v) - NUDGE_STEP)
                elif ch == 'd':
                    v = min(+1.0, (NEUTRAL_POS if v is None else v) + NUDGE_STEP)
                srv.value = v

            # Hall sensor reading
            try:
                hall = hall_read(adc)
            except Exception as e:
                print("\nHall read error: %s" % e)
                continue

            direction = ""
            if prev_hall is not None:
                diff = hall - prev_hall
                if diff > 80:
                    direction = ">> EXTENDING"
                elif diff < -80:
                    direction = "<< RETRACTING"

            b   = bar(hall, HALL_TARGET, HALL_MAX)
            st  = hall_status(hall)
            usec = -1 if v is None else servo_usec(v)   # 082526: -1 = no pulses

            # show what the servo value implies for the winch
            release_val = ROTATION_DIR * RELEASE_PWR + NEUTRAL_POS
            retract_val = ROTATION_DIR * RETRACT_PWR + NEUTRAL_POS   # 082526
            if v is None:
                action = "OFF     "   # 082526: pulses stopped
            elif abs(v - NEUTRAL_POS) < 0.001:
                action = "NEUTRAL "
            elif abs(v - release_val) < 0.001:
                action = "RELEASE "
            elif abs(v - retract_val) < 0.001:
                action = "RETRACT "
            elif (ROTATION_DIR == 1 and v > NEUTRAL_POS) or (ROTATION_DIR == -1 and v < NEUTRAL_POS):
                action = "~release"
            else:
                action = "~retract"

            print("  %-6s %-8s %-7d  %-6d  %s  %-10s  %s" % (
                ("None" if v is None else "%+.3f" % v),
                action, usec, hall, b, st, direction), end="\r", flush=True)

            prev_hall = hall

    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        srv.value = None   # 082526: stop pulses; no need to hold neutral first
        time.sleep(0.2)
        srv.detach()
        print()
        print()

    # final summary
    try:
        final_hall = hall_read(adc)
    except Exception:
        final_hall = None

    print("=" * 60)
    print("  Test complete.")
    if final_hall is not None:
        print("  Final Hall reading: %d  (%s)" % (final_hall, hall_status(final_hall)))
    print()
    print("  CHECKLIST:")
    print("  [ ] Hall INCREASES when you press r (extend)")
    print("  [ ] Hall DECREASES when you press t (retract)")
    print("  [ ] Hall near %d when fully retracted" % HALL_TARGET)
    print("  [ ] Hall near %d when fully extended" % HALL_MAX)
    print("  [ ] Servo stops cleanly on z, and STAYS COOL with pulses off")
    print()
    print("  If Hall direction is REVERSED:")
    print("    Option 1: flip ROTATION_DIR from %d to %d in this file and main script" % (
        ROTATION_DIR, -ROTATION_DIR))
    print("    Option 2: swap the Hall sensor signal wires")
    print("=" * 60)
    print()

if __name__ == "__main__":
    main()
