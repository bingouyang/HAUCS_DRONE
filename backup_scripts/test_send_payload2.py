"""
Quick end-to-end path test for the RC8 -> Pi -> FC -> base station -> Firebase chain,
run directly on the companion Pi.

IMPORTANT -- this only tests the send side. It does NOT (and can't) fake RC8/servo
state for the base station: SERVO_OUTPUT_RAW/RC_CHANNELS come from the flight
controller itself, not the Pi, so the base station's servo_mon won't arm unless RC8
is actually held above threshold (>=1800us) on the real transmitter/receiver while
this script sends. If you run this with RC8 low, the base station's [haucs] debug
log will show "DATA96 received but DROPPED: servo_mon not armed" -- that's expected,
not a bug, in that case.

What this script does:
  1. Connects to the FC exactly like the main script does.
  2. Prints the live RC8 (chan8_raw) PWM value for a few seconds so you can confirm,
     from the Pi's own point of view, that the switch is actually above threshold
     before anything gets sent -- if the Pi doesn't see it either, the problem is
     RC/receiver wiring, not software.
  3. Sends a few recognizable dummy payloads via send_payload() (using the correct
     "DO"/"pressure" keys so they actually reach encoder_helper's var mapping).

While this runs, watch the base station's MAVProxy console for:
  - "Servo Rising Edge" / state=1 (confirms the base station itself saw RC8 go high)
  - "[haucs][status] ... DATA96: seen=N processed=N dropped=N" (confirms packets
    arrived and were accepted, not just arrived-and-dropped)
  - the Firebase write / "db upload data: ..." print in _commit_current()

Usage: python3 test_send_payload2.py
"""

import time
import traceback

from pymavlink import mavutil

# Mirror the main script's imports exactly -- send_payload/resend_buffer/load_buffer
# live in encoder_helper, not winch_helper, but import all three the same way the
# main script does so whichever module defines what resolves correctly.
from winch_helper import *
from encoder_helper import *
from bt_helper import *

FC_CONN_STR = "/dev/serial0"
FC_BAUD = 115200
BUFFER_PATH = "outbox.json"
TRIGGER_RC_CH = 8
PWM_RELEASE = 1800  # must match main script's PWM_RELEASE

RC_WATCH_SECONDS = 5
NUM_TEST_SENDS = 3
SEND_INTERVAL_S = 3

# Recognizable dummy values -- easy to spot in Firebase / the base station's printed
# "db upload data: {...}" line so you know it's this test, not a real drop.
DUMMY_DO = 77.7
DUMMY_TEMP = 21.2
DUMMY_PRESS = 1055.5
DUMMY_LAT = 37.4275
DUMMY_LON = -122.1697


def make_dummy_cols(n=5):
    return {
        "time": list(range(n)),
        "DO": [round(DUMMY_DO + i, 2) for i in range(n)],
        "temp": [round(DUMMY_TEMP + 0.1 * i, 2) for i in range(n)],
        "pressure": [round(DUMMY_PRESS + i, 2) for i in range(n)],
        "init_DO": [100.0] * n,
        "init_pressure": [1013.0] * n,
        "batt_v": [4.1] * n,
    }


def watch_rc8(m_fc, seconds):
    print("\nWatching RC8 (chan8_raw) for %ds -- flip/hold your switch now." % seconds)
    print("(This is the Pi's own view -- if it doesn't move, check RC/receiver wiring")
    print(" before worrying about anything downstream.)\n")
    t0 = time.time()
    last_print = 0
    while time.time() - t0 < seconds:
        msg = m_fc.recv_match(type=["RC_CHANNELS", "SERVO_OUTPUT_RAW"], blocking=True, timeout=1)
        if msg is None:
            continue
        pwm = getattr(msg, "chan%d_raw" % TRIGGER_RC_CH, None) if msg.get_type() == "RC_CHANNELS" \
            else getattr(msg, "servo%d_raw" % TRIGGER_RC_CH, None)
        if pwm is None:
            continue
        now = time.time()
        if now - last_print > 0.3:
            last_print = now
            armed = "ABOVE threshold (armed)" if pwm >= PWM_RELEASE else "below threshold"
            print("  [%s] RC%d = %s us -- %s" % (msg.get_type(), TRIGGER_RC_CH, pwm, armed))


def main():
    print("=" * 60)
    print("RC8 -> Pi -> FC -> base station path test")
    print("=" * 60)

    for name in ("send_payload", "resend_buffer", "load_buffer"):
        if name not in globals():
            print(
                "FATAL: %s is not defined by any of winch_helper/encoder_helper/"
                "bt_helper. Check that module still defines/exports it." % name
            )
            return

    sensor_state = {"seq": 0}
    try:
        sensor_state["failed"] = load_buffer(BUFFER_PATH)
    except Exception as e:
        print("load_buffer() FAILED: %s" % e)
        sensor_state["failed"] = {}

    print("Connecting to FC on %s @ %s ..." % (FC_CONN_STR, FC_BAUD))
    m_fc = mavutil.mavlink_connection(FC_CONN_STR, baud=FC_BAUD)

    print("Waiting up to 10s for HEARTBEAT ...")
    hb = m_fc.wait_heartbeat(timeout=10)
    if not hb:
        print("NO HEARTBEAT in 10s -- check UART wiring/baud/serial port. Aborting.")
        return
    print("Got HEARTBEAT: sys=%s comp=%s" % (hb.get_srcSystem(), hb.get_srcComponent()))

    payload_link = m_fc

    watch_rc8(m_fc, RC_WATCH_SECONDS)

    print("\nFlushing any previously buffered/failed sends via resend_buffer() ...")
    try:
        sensor_state["failed"] = resend_buffer(payload_link, sensor_state.get("failed", {}))
    except Exception as e:
        print("resend_buffer() FAILED: %s" % e)
        print(traceback.format_exc())

    print(
        "\nSending %d dummy payloads (DO~%.1f, pressure~%.1f), %ds apart."
        % (NUM_TEST_SENDS, DUMMY_DO, DUMMY_PRESS, SEND_INTERVAL_S)
    )
    print("Keep RC8 held above %dus while these go out.\n" % PWM_RELEASE)

    for i in range(NUM_TEST_SENDS):
        cols = make_dummy_cols()
        seq_before = sensor_state.get("seq")
        t0 = time.monotonic()
        try:
            send_payload(payload_link, cols, sensor_state)
            dt = time.monotonic() - t0
            print(
                "[%d/%d] send_payload() OK in %.3fs (seq %s -> %s)"
                % (i + 1, NUM_TEST_SENDS, dt, seq_before, sensor_state.get("seq"))
            )
        except Exception as e:
            dt = time.monotonic() - t0
            print("[%d/%d] send_payload() RAISED after %.3fs: %s" % (i + 1, NUM_TEST_SENDS, dt, e))
            print(traceback.format_exc())

        if i < NUM_TEST_SENDS - 1:
            time.sleep(SEND_INTERVAL_S)

    print("\nDone sending. Now check the base station MAVProxy console:")
    print("  - Did 'Servo Rising Edge' / state=1 print while you held RC8?")
    print("  - Does '[haucs][status] ... DATA96: seen=N processed=N' show processed > 0?")
    print("  - Did '_commit_current' print 'db upload data: {...}' with DO~%.1f?" % DUMMY_DO)
    print(
        "\nIf processed stays 0 while seen increases: servo_mon never armed (check RC8 "
        "wiring/channel/threshold on the base station side)."
    )
    print(
        "If seen never increases at all: packets aren't reaching the base station "
        "(radio link / FC forwarding), not a base-station logic issue."
    )


if __name__ == "__main__":
    main()
