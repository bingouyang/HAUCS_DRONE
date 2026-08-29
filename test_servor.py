from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory
import time, sys, termios, tty, select

# ===== USER SETTINGS =====
PIN = 17  # your GPIO pin
MIN_PW = 0.0009   # 900 µs
MAX_PW = 0.0021   # 2100 µs
FRAME = 0.02      # 20 ms (50 Hz)
STEP  = 0.005     # step size for speed adjustments
# =========================

factory = PiGPIOFactory()
srv = Servo(PIN,
            min_pulse_width=MIN_PW,
            max_pulse_width=MAX_PW,
            frame_width=FRAME,
            pin_factory=factory,
            initial_value=0.0)  # try neutral

def value_to_usec(v):
    pw = MIN_PW + ((v + 1.0) / 2.0) * (MAX_PW - MIN_PW)
    return int(round(pw * 1e6))

print("\nContinuous Servo Creep Test (PiGPIOFactory)")
print("Controls: [a]= more negative, [d]= more positive, [z]= zero (1500us),")
print("          [x]= STOP PULSES (servo off), [r]= 0.4, [v]= -0.12, [q]= quit\n")

# 082526: this used to start at 0.0, i.e. a continuous 1500us pulse train, and
# sit there until a key was pressed. A servo commanded to hold still still
# draws current: bench measurement showed a damaged unit dissipating ~12 W in
# exactly this state. Start with the pulses stopped so the servo is only driven
# when you actually ask for it, and press z if you want to command neutral.
v = None
srv.value = v
time.sleep(0.5)

fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)
tty.setcbreak(fd)

try:
    while True:
        # check if a key was pressed
        dr, _, _ = select.select([sys.stdin], [], [], 0.05)
        if dr:
            ch = sys.stdin.read(1)
            if ch.lower() == 'q':
                break
            elif ch.lower() == 'a':
                # 082526: v is None at startup and after x.
                v = max(-1.0, (0.0 if v is None else v) - STEP)
            elif ch.lower() == 'd':
                v = min(+1.0, (0.0 if v is None else v) + STEP)
            elif ch.lower() == 'r':
                v=0.4
            elif ch.lower() == 'v':
                v=-0.12               
            elif ch.lower() == 'z':
                v = 0.0
            elif ch.lower() == 'x':
                v = None      # 082526: stop pulses, servo draws nothing

            srv.value = v
            if v is None:
                print("value=None    pulses STOPPED (servo off)   ",
                      end='\r', flush=True)
            else:
                print(f"value={v:+.3f}  ~ {value_to_usec(v)} us          ",
                      end='\r', flush=True)
except KeyboardInterrupt:
    pass
finally:
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    srv.detach()  # stop driving pulses
    print("\n\nDone.")
    if v is None:
        print("Exited with pulses stopped.")
    else:
        print(f"Suggested NEUTRAL approx value={v:+.3f}  (~{value_to_usec(v)} us)")