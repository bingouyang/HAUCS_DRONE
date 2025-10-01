import time, random
from pymavlink import mavutil

# ------------ CONFIG ------------
GCS_TARGET = 'udpout:192.168.1.160:14551'    # GCS on same PC
PI_TARGET  = 'udpout:10.113.51.25:14551'      # <-- replace with your Pi’s IP
SOURCE_SYS  = 1
SOURCE_COMP = 1
N_CYCLES    = 2

# Use your location
HOME_LAT = 27.535369
HOME_LON = -80.360334

# (label, landed_state, duration_sec, lat, lon, rel_alt_m)
PHASES = [
    ("INIT_TAKEOFF",   mavutil.mavlink.MAV_LANDED_STATE_IN_AIR,     5, HOME_LAT,        HOME_LON,        20),
    ("IN_AIR",         mavutil.mavlink.MAV_LANDED_STATE_IN_AIR,     5, HOME_LAT+0.0001, HOME_LON+0.0001, 30),
    ("LANDING",        mavutil.mavlink.MAV_LANDED_STATE_LANDING,    3, HOME_LAT+0.0002, HOME_LON+0.0002, 10),
    ("LANDED",        mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,    60, HOME_LAT+0.0004, HOME_LON+0.0002, 10),    
    ("SECOND_TAKEOFF", mavutil.mavlink.MAV_LANDED_STATE_IN_AIR,     5, HOME_LAT+0.0003, HOME_LON+0.0002, 25),
    ("LANDING",        mavutil.mavlink.MAV_LANDED_STATE_LANDING,    3, HOME_LAT+0.0004, HOME_LON+0.0002, 10),
    ("LANDED",        mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,    60, HOME_LAT+0.0004, HOME_LON+0.0002, 10),
    ("THIRD_TAKEOFF", mavutil.mavlink.MAV_LANDED_STATE_IN_AIR,     5, HOME_LAT+0.0005, HOME_LON+0.0002, 25),
    ("FINAL_LANDING",  mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,  6, HOME_LAT,        HOME_LON,        0),
]

# Copter mode mapping (ArduPilot custom_mode values)
# https://ardupilot.org/copter/docs/parameters.html#flightmode
COPTER_MODES = {
    0: "STABILIZE",
    1: "ACRO",
    2: "ALT_HOLD",
    3: "AUTO",
    4: "GUIDED",
    5: "LOITER",
    6: "RTL",
    7: "CIRCLE",
    9: "LAND",
    11: "DRIFT",
    13: "SPORT",
    14: "FLIP",
    15: "AUTOTUNE",
    16: "POSHOLD",
    17: "BRAKE",
    18: "THROW",
    19: "AVOID_ADSB",
    20: "GUIDED_NOGPS",
    21: "SMART_RTL",
    22: "FLOWHOLD",
    23: "FOLLOW",
    24: "ZIGZAG",
    25: "SYSTEMID",
    26: "AUTOROTATE",
    27: "AUTO_RTL",
}

ID_TO_MODE = {v: k for k, v in COPTER_MODES.items()}

GPS_HZ    = 5
HB_HZ     = 1
EXTSYS_HZ = 2
MD_MIN = 3             #mission duration min
MD_MAX = 5             #mission duration max

def endpt_connect(target):
    return mavutil.mavlink_connection(target, source_system=SOURCE_SYS, source_component=SOURCE_COMP)

endpts = [endpt_connect(GCS_TARGET), endpt_connect(PI_TARGET)]
def update_all_endpts(fn):
    for endpt in endpts:
        try: fn(endpt)
        except: pass

def send_heartbeat(endpt, armed, mode_name):
    mode = ID_TO_MODE[mode_name.upper()]
    #print(f"mode_name:{mode_name} mode:{mode}")
    base_mode = 0
    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1 << 5
    if armed:
        MAV_MODE_FLAG_SAFETY_ARMED = 1 << 7
        base_mode |= MAV_MODE_FLAG_SAFETY_ARMED
    base_mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
    endpt.mav.heartbeat_send(
        type=mavutil.mavlink.MAV_TYPE_QUADROTOR,
        autopilot=mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode=base_mode,
        custom_mode=mode,
        system_status=mavutil.mavlink.MAV_STATE_ACTIVE if armed else mavutil.mavlink.MAV_STATE_STANDBY,
    )

def send_extsys(c, landed_state):
    c.mav.extended_sys_state_send(
        mavutil.mavlink.MAV_VTOL_STATE_UNDEFINED,
        landed_state
    )

def send_gps(c, tboot_ms, lat_deg, lon_deg, rel_alt_m, vx=0, vy=0, vz=0, hdg_deg=0):
    lat = int(lat_deg * 1e7)
    lon = int(lon_deg * 1e7)
    alt_mm = int((50 + rel_alt_m) * 1000)
    rel_mm = int(rel_alt_m * 1000)
    hdg = int((hdg_deg % 360) * 100) if hdg_deg is not None else 65535
    c.mav.global_position_int_send(tboot_ms, lat, lon, alt_mm, rel_mm, vx, vy, vz, hdg)

def send_statustext(c, text, sev=mavutil.mavlink.MAV_SEVERITY_INFO):
    c.mav.statustext_send(sev, text.encode("utf-8")[:50])


def send_data96(c, seq, temp_c, press_kpa, do_mgL):
    # Pack small struct; pad to 96 bytes as required
    payload = struct.pack("<IIfff", int(seq), int(time.time()), float(temp_c), float(press_kpa), float(do_mgL))
    if len(payload) < 96:
        payload = payload + b"\x00" * (96 - len(payload))
    c.mav.data96_send(1, 20, payload)  # dtype=1, len=20 for our struct

def main():
    print(f"Streaming to:\n  GCS={GCS_TARGET}\n  PI ={PI_TARGET}")
    t0 = time.time()
    last_hb = 0.0
    hb_dt   = 1.0 / HB_HZ
    armed   = True

    seq = 0

    temp_c = 22.5
    press_kpa = 101.3
    do_mgL  = 6.75

    for cycle in range(1, N_CYCLES + 1):
        print(f"\n=== Starting cycle {cycle}/{N_CYCLES} ===")
        did_disarm_announce = False

        for label, landed, dur, lat, lon, rel_alt in PHASES:
            if label == "LANDING":
                print(">>> STATUSTEXT: Sampling landing")
                update_all_endpts(lambda endpt: send_statustext(endpt, "Sampling landing"))
            if label == "FINAL_LANDING":
                print(">>> STATUSTEXT: Mission complete")
                update_all_endpts(lambda endpt: send_statustext(endpt, "Mission complete"))

            print(f"---> {label}")
            start   = time.time()
            gps_dt  = 1.0 / GPS_HZ
            ext_dt  = 1.0 / EXTSYS_HZ
            next_gps = 0.0
            next_ext = 0.0

            while (time.time() - start) < dur:
                now = time.time()

                if (now - last_hb) >= hb_dt:
                    with open('mode.txt', 'r') as file:
                        mode = file.read()
                        #print(mode)
                    update_all_endpts(lambda endpt: send_heartbeat(endpt, armed, mode))
                    last_hb = now
                    
                    tboot_ms = int((now - t0) * 1000)
                    #update_all_endpts(lambda endpt: send_gps(endpt, tboot_ms, lat, lon, rel_alt))
                    

                    next_gps += gps_dt

                if (now - start) >= next_ext:
                    #update_all_endpts(lambda endpt: send_extsys(endpt, landed))
                    update_all_endpts(lambda endpt: send_data96(endpt, seq, temp_c, press_kpa, do_mgL))
                    print("send data96")
                    next_ext += ext_dt

                time.sleep(0.01)

            if label == "FINAL_LANDING" and not did_disarm_announce:
                armed = False
                print(">>> STATUSTEXT: Auto disarm")
                update_all_endpts(lambda endpt: send_statustext(endpt, "Auto disarm"))
                did_disarm_announce = True

            pause = random.uniform(MD_MIN, MD_MAX)
            print(f"--- dwell {pause:.1f}s ---")
            time.sleep(pause)

        print(f"=== Finished cycle {cycle}/{N_CYCLES} ===")
        armed = True  # re-arm for next cycle

    print("All cycles complete.")

if __name__ == "__main__":
    main()
