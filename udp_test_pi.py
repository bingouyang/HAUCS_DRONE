# udp_test_pi.py
import socket, time

PC_IP = "10.113.32.16"   # <-- change to your PC's IP
PC_PORT = 14551          # the port where MAVProxy is listening

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

for i in range(100):
    msg = f"TEST_PACKET {i}".encode()
    sock.sendto(msg, (PC_IP, PC_PORT))
    print(f"Sent: {msg} to {PC_IP}:{PC_PORT}")
    time.sleep(1)

sock.close()
