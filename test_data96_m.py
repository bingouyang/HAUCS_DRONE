#!/usr/bin/env python3
from datetime import datetime
import sys, time, threading, struct, random
from pymavlink import mavutil
# --- Fixed settings ---
TARGET_IP   = "10.120.239.168"
#TARGET_IP   = "192.168.1.160"

TARGET_PORT = 14555
SYSID       = 200
COMPID      = 200
HZ          = 2.0
VERBOSE     = True

# --- Setup connection ---
url = f"udpout:{TARGET_IP}:{TARGET_PORT}"
mav = mavutil.mavlink_connection(
    url,
    source_system=SYSID,
    source_component=COMPID,
    autoreconnect=True
)

def heartbeat_loop(link):
    """
    Send 1 Hz HEARTBEAT so MAVProxy recognizes this system.
    MAV_TYPE_GCS(6) + MAV_AUTOPILOT_INVALID(8) is fine for a companion sender.
    """
    while True:
        try:
            link.mav.heartbeat_send(
                type=6,           # MAV_TYPE_GCS
                autopilot=8,      # MAV_AUTOPILOT_INVALID
                base_mode=0,
                custom_mode=0,
                system_status=0
            )
            print("[HB] sent")
        except Exception as e:
            print(f"[HB] error: {e}")
        time.sleep(1.0)
        
def send_heartbeat():
    mav.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GENERIC,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0, 0,
        mavutil.mavlink.MAV_STATE_ACTIVE
    )

def send_sys_status(volts_mv=12000, current_cA=1500, battery_remaining=95, load=120):
    present = 0
    enabled = 0
    health  = 0
    drop_rate_comm = 0
    errors_comm = 0
    e1 = e2 = e3 = e4 = 0

    mav.mav.sys_status_send(
        present, enabled, health,
        int(load),
        int(volts_mv) & 0xFFFF,
        int(current_cA) if -32768 <= int(current_cA) <= 32767 else -1,
        int(battery_remaining) if -1 <= int(battery_remaining) <= 100 else -1,
        drop_rate_comm,
        errors_comm,
        e1, e2, e3, e4
    )

def send_named(name, value, time_boot_ms):
    mav.mav.named_value_float_send(time_boot_ms, name[:10].encode("ascii", "replace"), float(value))

def send_statustext(text, severity=mavutil.mavlink.MAV_SEVERITY_INFO):
    mav.mav.statustext_send(severity, text[:50].encode("ascii", "replace"))

def pack_data96(payload_bytes: bytes):
    if len(payload_bytes) > 96:
        payload_bytes = payload_bytes[:96]
    return payload_bytes + b"\x00" * (96 - len(payload_bytes))

def send_data96(dtype, payload_bytes):
    b = pack_data96(payload_bytes)
    length = min(len(payload_bytes), 96)
    try:
        mav.mav.data96_send(dtype, length, b)
        return True, None
    except Exception as e:
        return False, str(e)

def main():
    t0 = time.time()
    hb_period = 1.0 / HZ
    last_hb = 0.0
    last_sys = 0.0
    seq = 0

    temp_c = 22.5
    press_kpa = 101.3
    do_mgL = 6.75

    send_statustext("TEST sender started")
    send_statustext("DATA96 enabled")
    # Start heartbeat thread
    threading.Thread(target=heartbeat_loop, args=(mav,), daemon=True).start()

    while True:
        now = time.time()
        ms = int((now - t0) * 1000)

        # Heartbeat + fast numbers @ 2 Hz
        if now - last_hb >= hb_period:
            last_hb = now
            #send_heartbeat()
            #send_named("tempC",    temp_c    + 0.1*random.uniform(-1, 1), ms)
            #send_named("pressKPa", press_kpa + 0.05*random.uniform(-1, 1), ms)
            #send_named("DOmgL",    do_mgL    + 0.05*random.uniform(-1, 1), ms)

            payload = struct.pack("<IIfff", seq, int(time.time()), temp_c, press_kpa, do_mgL)
            ok, err = send_data96(1, payload)
            if VERBOSE:
                ts = datetime.now().strftime("%H:%M:%S")
                print(f"[{ts}] HB + {payload} {'- DATA96 seq='+str(seq) if ok else 'DATA96 FAIL: '+str(err)}")
            seq += 1

        # SYS_STATUS + STATUSTEXT every ~2 s
        if now - last_sys >= 2.0:
            last_sys = now
            batt_v_mv = 12000 + int(200*random.uniform(-1, 1))
            batt_i_cA = 1500 + int(50*random.uniform(-1, 1))
            batt_pct  = max(0, min(100, 96 - int((now - t0) / 60)))
            load      = 120 + int(10*random.uniform(-1, 1))
            send_sys_status(batt_v_mv, batt_i_cA, batt_pct, load)
            #send_statustext(f"OK v={batt_v_mv/1000:.2f}V i={batt_i_cA/100:.2f}A soc={batt_pct}%")

        time.sleep(0.05)

if __name__ == "__main__":
    main()
