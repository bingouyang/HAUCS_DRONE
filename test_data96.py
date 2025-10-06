#!/usr/bin/env python3
"""
Pi → PC test:
- sends 1 Hz HEARTBEAT so MAVProxy recognizes this system
- sends a few DATA96 frames (var_id=1 → DO) for your module to decode/print

Run:
  python3 test_data96_with_heartbeat.py <PC_IP> [14551]
"""

import sys, time, threading, struct, random
from pymavlink import mavutil

PC_IP   = sys.argv[1] if len(sys.argv) > 1 else "10.113.32.16"  # <-- set your PC IP (or pass arg)
#PC_IP   = sys.argv[1] if len(sys.argv) > 1 else "192.168.1.160"  # <-- set your PC IP (or pass arg)

PC_PORT = int(sys.argv[2]) if len(sys.argv) > 2 else 14555      # MAVProxy master listening port

# Wire-format (must match your MAVProxy module)
SCALE      = 32
DATA_BYTES = 96
HDR_LEN    = 8        # u32 seq | u8 var_type | i16 base | u8 len
MAX_RES    = DATA_BYTES - HDR_LEN

# This system id/component id for the Pi companion (arbitrary but consistent)
SYSID = 1
COMPID = 1

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

def make_payload(seq, var_id, base, values):
    """
    Build DATA96 payload like the encoder:
      [seq:u32][var_type:u8][base:i16][len:u8][residues:int8...](zero-padded to 96B)
    """
    residues = []
    for v in values:
        r = int(round((v - base) * SCALE))
        residues.append(max(-128, min(127, r)))
    if len(residues) > MAX_RES:
        residues = residues[:MAX_RES]
    var_len  = len(residues)
    var_byte = var_id & 0x7F  # top bit reserved for resend
    header = struct.pack("!IBhB", seq & 0xFFFFFFFF, var_byte & 0xFF, int(base), var_len & 0xFF)
    payload = bytearray(header + struct.pack("!" + "b"*var_len, *residues))
    if len(payload) < DATA_BYTES:
        payload.extend(b"\x00" * (DATA_BYTES - len(payload)))
    return bytes(payload)

def main():
    print(f"[SETUP] TX → udpout:{PC_IP}:{PC_PORT}  (SYSID={SYSID}, COMPID={COMPID})")
    m = mavutil.mavlink_connection(
        f"udpout:{PC_IP}:{PC_PORT}",
        source_system=SYSID,
        source_component=COMPID
    )

    # Start heartbeat thread
    threading.Thread(target=heartbeat_loop, args=(m,), daemon=True).start()

    # Send a few DATA96 frames (var_id=1 == DO in your module)
    var_id = 1
    seq = 100
    for i in range(5000):
        base = 7
        # make ~10 DO samples around 7.00 ±0.02
        vals = [7.00 + 0.02*(j-5)/5.0 + random.uniform(-0.002, 0.002) for j in range(10)]
        payload = make_payload(seq, var_id, base, vals)
        try:
            m.mav.data96_send(var_id, len(payload), payload)
            print(f"[TX] DATA96 sent: seq={seq} id={var_id} base={base} samples={len(vals)} len={len(payload)}")
        except Exception as e:
            print(f"[TX] ERROR: {e}")
        seq = (seq + 1) & 0xFFFFFFFF
        time.sleep(0.5)

    print("[DONE] Sent 5 DATA96 frames; heartbeat continues at 1 Hz. Press Ctrl+C to stop.")
    # Keep process alive to keep heartbeats going
    while True:
        time.sleep(1)

if __name__ == "__main__":
    main()
