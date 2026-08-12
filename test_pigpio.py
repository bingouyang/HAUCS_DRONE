from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero import Servo
factory = PiGPIOFactory()
s = Servo(17, pin_factory=factory)
s.value = 0.0
import time; time.sleep(1)
import pigpio
pi = pigpio.pi()
print('servo pw:', pi.get_servo_pulsewidth(17))
print('pwm dc:', pi.get_PWM_dutycycle(17))
print('pwm freq:', pi.get_PWM_frequency(17))
pi.stop()
