#!/usr/bin/env python3
"""
test_gcs_msg.py - verify the Pi -> FC -> telemetry -> Mission Planner path.

Sends the same STATUSTEXT that gcs_status() in main_rc8_uart_parm.py sends,
using the same connection string, baud, severity, prefix and 49-char cap, so
a pass here means the real thing will work.

Stop main_rc8_uart_parm.py first - two processes cannot share /dev/serial0.
    sudo systemctl stop haucs      (or however it is started)

Usage:
    python3 test_gcs_msg.py                 # 5 messages, 2s apart
    python3 test_gcs_msg.py -n 20 -i 0.5    # rate/dedup test
    python3 test_gcs_msg.py --text "winch bench test"
    python3 test_gcs_msg.py --listen        # also print STATUSTEXT from the FC

Watch Mission Planner's Messages tab. Each line should appear as
    HAUCS: gcs test 1/5 ...
"""

import argparse
import sys
import time

try:
    from pymavlink import mavutil
except ImportError:
    print("pymavlink not installed:  pip3 install pymavlink")
    sys.exit(1)

# Must match main_rc8_uart_parm.py
FC_CONN_STR = "/dev/serial0"
FC_BAUD = 115200
PREFIX = "HAUCS: "
MAX_LEN = 49          # gcs_status truncates to 49 chars incl. prefix


def send_status(m, text):
    """Byte-for-byte the same call gcs_status() makes."""
    payload = (PREFIX + text)[:MAX_LEN].encode("ascii", "replace")
    m.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, payload)
    return payload.decode("ascii")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-n", "--count", type=int, default=5)
    ap.add_argument("-i", "--interval", type=float, default=2.0)
    ap.add_argument("--text", default="gcs test")
    ap.add_argument("--port", default=FC_CONN_STR)
    ap.add_argument("--baud", type=int, default=FC_BAUD)
    ap.add_argument("--listen", action="store_true",
                    help="also print STATUSTEXT coming back from the FC")
    args = ap.parse_args()

    print("opening %s @ %d" % (args.port, args.baud))
    try:
        m = mavutil.mavlink_connection(args.port, baud=args.baud)
    except Exception as e:
        print("FAILED to open the serial port: %s" % e)
        print("  - is main_rc8_uart_parm.py still running? stop it first")
        print("  - check the port exists:  ls -l /dev/serial0")
        print("  - check permissions:      groups | grep dialout")
        return 1

    print("waiting for HEARTBEAT from the Cube (10s)...")
    hb = m.wait_heartbeat(timeout=10)
    if not hb:
        print("NO HEARTBEAT.")
        print("  The Pi cannot talk to the flight controller at all, so no")
        print("  STATUSTEXT can reach the GCS. Check UART wiring, that the FC")
        print("  serial port is SERIALn_PROTOCOL=2 (MAVLink2), and that its")
        print("  BAUD matches %d." % args.baud)
        return 1

    print("connected: sys=%s comp=%s  (FC type %s)"
          % (hb.get_srcSystem(), hb.get_srcComponent(), hb.type))
    print("sending %d message(s), %.1fs apart\n" % (args.count, args.interval))

    sent = 0
    for i in range(1, args.count + 1):
        text = "%s %d/%d %s" % (args.text, i, args.count,
                                time.strftime("%H:%M:%S"))
        try:
            shown = send_status(m, text)
            sent += 1
            print("  sent: %s" % shown)
        except Exception as e:
            print("  send FAILED: %s" % e)

        # Drain the link so the buffer does not back up, and optionally show
        # anything the FC sends back.
        t_end = time.time() + args.interval
        while time.time() < t_end:
            msg = m.recv_match(blocking=False)
            if msg is None:
                time.sleep(0.05)
                continue
            if args.listen and msg.get_type() == "STATUSTEXT":
                try:
                    txt = msg.text
                    if isinstance(txt, bytes):
                        txt = txt.decode("ascii", "replace")
                except Exception:
                    txt = "<undecodable>"
                print("    <- FC: %s" % txt.rstrip("\x00"))

    print("\n%d/%d sent without error." % (sent, args.count))
    print("Now check Mission Planner's Messages tab (Ctrl-F -> Messages, or")
    print("the message line under the HUD).")
    print("")
    print("If they appear      -> the path works; gcs_status() will too.")
    print("If they do NOT      -> the Pi->FC link is fine (heartbeat proved it),")
    print("                       so the break is FC->GCS. Check that the")
    print("                       telemetry radio's SERIALn_PROTOCOL is 2, that")
    print("                       Mission Planner is connected on that link, and")
    print("                       that SRn_EXTRA3 / the STATUSTEXT stream is not")
    print("                       set to 0.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
