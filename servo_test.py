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
print("Controls: [a]= more negative, [d]= more positive, [s]= step toward 0, [z]= zero, [q]= quit\n")

v = 0.0
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
                v = max(-1.0, v - STEP)
            elif ch.lower() == 'd':
                v = min(+1.0, v + STEP)
            elif ch.lower() == 's':
                if v > 0: 
                    v = max(0.0, v - STEP)
                elif v < 0: 
                    v = min(0.0, v + STEP)
            elif ch.lower() == 'z':
                v = 0.0

            srv.value = v
            print(f"value={v:+.3f}  ~ {value_to_usec(v)} µs", end='\r', flush=True)

except KeyboardInterrupt:
    pass
finally:
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    srv.detach()  # stop driving pulses
    print("\n\nDone.")
    print(f"Suggested NEUTRAL ≈ value={v:+.3f}  (~{value_to_usec(v)} µs)")
