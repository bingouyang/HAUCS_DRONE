from pymavlink import mavutil

m = mavutil.mavlink_connection("/dev/serial0", baud=115200)
m.wait_heartbeat()
print("Connected")

while True:
    msg = m.recv_match(blocking=True, timeout=5)
    if msg is None:
        print("No MAVLink messages...")
    else:
        print(msg.get_type())
