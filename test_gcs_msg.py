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
    ap.add_argument("--sysid", type=int, default=1,
                    help="our MAVLink system id (default 1 = same as vehicle)")
    ap.add_argument("--compid", type=int, default=191,
                    help="our component id (default 191 = ONBOARD_COMPUTER)")
    ap.add_argument("--heartbeat", action="store_true",
                    help="also emit HEARTBEAT. MAVProxy prints 'online system N' "
                         "the first time it sees a new system id, so pairing this "
                         "with --sysid 42 proves whether the FC relays Pi traffic "
                         "to the telemetry link at all.")
    args = ap.parse_args()

    print("opening %s @ %d as sys=%d comp=%d"
          % (args.port, args.baud, args.sysid, args.compid))
    try:
        # Identify as the onboard computer, NOT as 255. Mission Planner is 255,
        # and ArduPilot will not forward a packet out to a link where it already
        # believes that system id lives - the STATUSTEXT gets dropped as a loop.
        m = mavutil.mavlink_connection(args.port, baud=args.baud,
                                       source_system=args.sysid,
                                       source_component=args.compid)
    except Exception as e:
        print("FAILED to open the serial port: %s" % e)
        print("  - is main_rc8_uart_parm.py still running? stop it first")
        print("  - check the port exists:  ls -l /dev/serial0")
        print("  - check permissions:      groups | grep dialout")
        return 1

    # A plain wait_heartbeat() matches ANY heartbeat, including the GCS's own
    # relayed through the FC (sys 255, MAV_TYPE_GCS=6). That looks like success
    # while proving nothing about the Cube, so filter for a real autopilot.
    print("waiting for an AUTOPILOT heartbeat (10s)...")
    seen = {}
    fc = None
    t_end = time.time() + 10
    while time.time() < t_end:
        hb = m.recv_match(type="HEARTBEAT", blocking=False)
        if hb is None:
            time.sleep(0.02)
            continue
        key = (hb.get_srcSystem(), hb.get_srcComponent())
        if key not in seen:
            seen[key] = (hb.type, hb.autopilot)
        is_ap = (hb.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID
                 and hb.type != mavutil.mavlink.MAV_TYPE_GCS)
        if is_ap and fc is None:
            fc = hb
            break

    if seen:
        print("heartbeat sources seen:")
        for (sysid, compid), (typ, ap_) in sorted(seen.items()):
            tag = "AUTOPILOT" if (ap_ != mavutil.mavlink.MAV_AUTOPILOT_INVALID
                                  and typ != mavutil.mavlink.MAV_TYPE_GCS) else "not an autopilot"
            print("   sys=%-4d comp=%-4d type=%-3d %s" % (sysid, compid, typ, tag))

    if fc is None:
        print("\nNO AUTOPILOT HEARTBEAT.")
        if seen:
            print("  Traffic IS arriving, but none of it is from the Cube -")
            print("  what you see above is the GCS relayed through the FC.")
            print("  The Pi->FC leg is unproven. Check UART wiring and that the")
            print("  FC port is SERIALn_PROTOCOL=2 at %d baud." % args.baud)
        else:
            print("  No MAVLink at all on this port.")
        return 1

    print("\nAUTOPILOT found: sys=%s comp=%s type=%s"
          % (fc.get_srcSystem(), fc.get_srcComponent(), fc.type))
    print("sending %d message(s), %.1fs apart\n" % (args.count, args.interval))

    sent = 0
    for i in range(1, args.count + 1):
        text = "%s %d/%d %s" % (args.text, i, args.count,
                                time.strftime("%H:%M:%S"))
        try:
            if args.heartbeat:
                # Announce ourselves as an onboard controller. This is the packet
                # MAVProxy reacts to visibly, unlike STATUSTEXT which it may drop
                # silently if it filters on source system.
                m.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            shown = send_status(m, text)
            sent += 1
            print("  sent: %s%s" % (shown, "  (+heartbeat)" if args.heartbeat else ""))
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
    print("If they appear -> the path works. Then apply the same source_system")
    print("   /source_component to main_rc8_uart_parm.py line ~1060, which today")
    print("   opens the link with no ids and so transmits as 255 like this")
    print("   script used to:")
    print("       m_fc = mavutil.mavlink_connection(FC_CONN_STR, baud=FC_BAUD,")
    print("                                         source_system=1,")
    print("                                         source_component=191)")
    print("")
    print("If they do NOT -> try a different id pair (--sysid 1 --compid 25, or")
    print("   --sysid 42), then check the telemetry port's SERIALn_PROTOCOL=2 and")
    print("   that SRn_EXTRA3 is not 0. Run with --listen to confirm traffic is")
    print("   flowing both ways on this port.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
