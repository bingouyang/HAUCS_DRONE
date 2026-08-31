# HAUCS Pi companion computer -- 083026
#
# Generated from the imports actually present in the flight stack, not from
# pip freeze (which would capture every unrelated package on the Pi).
#
# Install on Raspberry Pi OS Bookworm or later:
#     pip install -r requirements.txt --break-system-packages
#
# The --break-system-packages flag is required because Bookworm marks the
# system Python as externally managed (PEP 668). Use a venv instead if you
# prefer, but note that gpiozero/pigpio and Blinka need system I2C and GPIO
# access, so a venv must be created with --system-site-packages.

# ---- flight critical: main_rc8_uart_parm.py / main_rc8_uart_latch.py -------
pymavlink                          # MAVLink to the Cube over /dev/serial0
gpiozero                           # servo PWM
pigpio                             # gpiozero's pin factory (PiGPIOFactory)
Adafruit-Blinka                    # provides the `board` and `busio` modules
adafruit-circuitpython-ads1x15     # ADS1115, Hall sensor on A0
smbus2                             # 083026: INA226 rail monitor (ina_helper.py)

# ---- BLE sensor: bt_helper.py ---------------------------------------------
adafruit-circuitpython-ble         # Nordic UART Service to the DO probe
numpy
pandas
scipy                              # curve_fit, used for the calibration fits

# ---- utilities, not needed for flight -------------------------------------
# upload_cached_samples.py only. Omit on the aircraft if you want a smaller
# install; the flight scripts never import these.
firebase-admin
pytz

# ---- optional -------------------------------------------------------------
# bt_helper.py imports PyQt5 for QThread/QMutex but falls back to threading
# stubs when it is absent, so this is genuinely optional on the Pi. Leave it
# out unless something else needs it.
# PyQt5


# ---- NOT pip installable --------------------------------------------------
# pigpio needs its DAEMON running, which is a system package, not the Python
# library above:
#     sudo apt install pigpio
#     sudo systemctl enable --now pigpiod
# Without pigpiod, PiGPIOFactory() raises at servo init and winch_thread
# returns immediately.
#
# I2C must also be enabled:
#     sudo raspi-config -> Interface Options -> I2C -> Enable
# Verify both chips are present before flying:
#     i2cdetect -y 1        # expect 0x44 (INA226) and 0x48 (ADS1115)
