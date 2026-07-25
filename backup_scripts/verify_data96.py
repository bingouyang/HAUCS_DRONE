import os
os.environ["MAVLINK_DIALECT"] = "ardupilotmega"

import time, struct, random
from datetime import datetime
from pymavlink import mavutil

# ---- Fixed settings (no args) ----
#TARGET_IP   = "10.113.32.16"
TARGET_IP   = "192.168.1.160"

TARGET_PORT = 14551
SYSID       = 1
COMPID      = 1
HZ          = 2.0
VERBOSE     = True

# ---- Connect ----
mav = mavutil.mavlink_connection(
    f"udpout:{TARGET_IP}:{TARGET_PORT}",
    source_system=SYSID,
    source_component=COMPID,
    autoreconnect=True
)

# Optional: maximize compatibility with older stacks
try:
    mav.mav.WIRE_PROTOCOL_VERSION = "1.0"
except Exception:
    pass

def say(msg, sev=mavutil.mavlink.MAV_SEVERITY_INFO):
    # bytes required on your build
    try:
        mav.mav.statustext_send(sev, msg[:50].encode("ascii","replace"))
    except Exception as e:
        if VERBOSE:
            print("[STATUSTEXT] FAIL:", e)

def hb():
    mav.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0, 0,
        mavutil.mavlink.MAV_STATE_ACTIVE
    )
    print("hb")

def sys_status(volts_mv, current_cA, battery_remaining_pct, load_0_1000):
    present = enabled = health = 0
    drop_rate_comm = errors_comm = 0
    e1 = e2 = e3 = e4 = 0
    mav.mav.sys_status_send(
        present, enabled, health,
        int(load_0_1000),
        int(volts_mv) & 0xFFFF,
        int(current_cA) if -32768 <= int(current_cA) <= 32767 else -1,
        int(battery_remaining_pct) if -1 <= int(battery_remaining_pct) <= 100 else -1,
        drop_rate_comm, errors_comm, e1, e2, e3, e4
    )

def named(k, v, tms=0):
    mav.mav.named_value_float_send(int(tms), k[:10].encode("ascii","replace"), float(v))

def send_data96(seq, temp_c, press_kpa, do_mgL):
    # Pack small struct; pad to 96 bytes as required
    payload = struct.pack("<IIfff", int(seq), int(time.time()), float(temp_c), float(press_kpa), float(do_mgL))
    if len(payload) < 96:
        payload = payload + b"\x00" * (96 - len(payload))
    mav.mav.data96_send(1, 20, payload)  # dtype=1, len=20 for our struct

def main():
    # Confirm dialect + DATA96 availability
    has_d96 = hasattr(mav.mav, "data96_send")
    say(f"dialect=ardupilotmega DATA96={'yes' if has_d96 else 'no'}")

    # Warm-up heartbeats
    for _ in range(3):
        hb(); time.sleep(0.3)

    t0 = time.time()
    period = 1.0 / HZ
    last_fast = 0.0
    last_slow = 0.0
    seq = 0

    temp_c = 22.5
    press_kpa = 101.3
    do_mgL  = 6.75

    say("test_data96_ardupilot started")

    while True:
        now = time.time()

        # 2 Hz block: HB + DATA96 (with fallback to NAMED if needed)
        if now - last_fast >= period:
            last_fast = now
            hb()
            try:
                if has_d96:
                    send_data96(seq, temp_c, press_kpa, do_mgL)
                    if VERBOSE:
                        print(f"[{datetime.now().strftime('%H:%M:%S')}] DATA96 seq={seq}")
                else:
                    # Fallback if somehow missing
                    named("tempC", temp_c); named("pressKPa", press_kpa); named("DOmgL", do_mgL)
                    say(f"T={temp_c:.1f}C P={press_kpa:.1f}kPa DO={do_mgL:.2f}")
            except Exception as e:
                # If DATA96 send fails at runtime, fall back and report
                print("[DATA96] FAIL:", e)
                named("tempC", temp_c); named("pressKPa", press_kpa); named("DOmgL", do_mgL)
                say(f"T={temp_c:.1f}C P={press_kpa:.1f}kPa DO={do_mgL:.2f}")

            seq += 1

        # ~2 s block: SYS_STATUS + a human-readable line
        if now - last_slow >= 2.0:
            last_slow = now
            batt_v_mv = 12000 + int(200*random.uniform(-1, 1))
            batt_i_cA = 1500 + int(50*random.uniform(-1, 1))
            batt_pct  = 95
            load      = 120
            try:
                sys_status(batt_v_mv, batt_i_cA, batt_pct, load)
                print(f"[SYS_STATUS]")

            except Exception as e:
                print("[SYS_STATUS] FAIL:", e)
                say(f"v={batt_v_mv/1000:.2f}V i={batt_i_cA/100:.2f}A")

        time.sleep(2)

if __name__ == "__main__":
    main()
