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
import struct
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


# ---------------------------------------------------------------------------
# DATA96 wire format. Mirrors sampling_helper.py exactly - that file is the
# contract. NOTE: encoder_helper.py on the Pi was not available when this was
# written, so this is an independent implementation of the documented format.
# If this test passes and real casts do not, encoder_helper.py is the delta.
# ---------------------------------------------------------------------------
DATA_BYTES = 96
HDR_LEN = 9                      # seq(4) varbyte(1) base(int16,2) len(1) chunk(1)
SCALE = 32
SCALE_MAP = {0: 1, 1: 100, 2: 32, 3: 100, 4: 32, 5: 100, 6: 100}
WIDTH_MAP = {3: 2}               # pressure is int16
VAR_MAP = {"time": 0, "DO": 1, "temp": 2, "pressure": 3,
           "init_DO": 4, "init_pressure": 5, "batt_v": 6}
VAR_ID_FRAME_END = 127           # must match _var_id_frame_end on the ground


def _width(var_id):
    return WIDTH_MAP.get(int(var_id), 1)


def max_samples(var_id):
    return (DATA_BYTES - HDR_LEN) // _width(var_id)


def encode_frame(seq_id, var_id, values, chunk_idx, is_resend=0):
    """Build one DATA96 payload. Inverse of sampling_helper.msg_decoder()."""
    scale = SCALE_MAP.get(var_id, SCALE)
    width = _width(var_id)
    fmt = "h" if width == 2 else "b"
    lo, hi = (-32768, 32767) if width == 2 else (-128, 127)

    base = int(round(sum(values) / float(len(values))))
    base = max(-32768, min(32767, base))

    residues = []
    for v in values:
        r = int(round((v - base) * scale))
        if r < lo or r > hi:
            raise ValueError("residue %d out of range for var_id %d "
                             "(value %r, base %d, scale %d)"
                             % (r, var_id, v, base, scale))
        residues.append(r)

    var_byte = (var_id & 0x7F) | (0x80 if is_resend else 0)
    hdr = (struct.pack("!I", seq_id)
           + bytes([var_byte])
           + struct.pack("!h", base)
           + bytes([len(values) & 0xFF])
           + bytes([chunk_idx & 0xFF]))
    return hdr + struct.pack("!" + fmt * len(values), *residues)


def decode_frame(buf):
    """Local copy of sampling_helper.msg_decoder, for round-trip checking."""
    seq_id = struct.unpack_from("!I", buf, 0)[0]
    var_byte = buf[4]
    var_base = struct.unpack_from("!h", buf, 5)[0]
    var_len = buf[7] & 0xFF
    chunk_idx = buf[8]
    var_id = var_byte & 0x7F
    fmt = "h" if _width(var_id) == 2 else "b"
    residues = list(struct.unpack_from("!" + fmt * var_len, buf, HDR_LEN))
    scale = SCALE_MAP.get(var_id, SCALE)
    return (seq_id, (var_byte & 0x80) and 1 or 0, var_id, var_len,
            [var_base + r / scale for r in residues], 0, chunk_idx)


def send_frame(m, payload):
    buf = payload + b"\x00" * (DATA_BYTES - len(payload))
    m.mav.data96_send(0, len(payload), list(buf))


def synth_cast(n):
    """A plausible cast: pressure ramps ~1 m, DO falls, temp falls slightly."""
    return {
        "time":     [float(i) for i in range(n)],
        "DO":       [1.05 - 0.25 * (i / float(max(n - 1, 1))) for i in range(n)],
        "temp":     [25.4 - 0.8 * (i / float(max(n - 1, 1))) for i in range(n)],
        "pressure": [1013.0 + 98.0 * (i / float(max(n - 1, 1))) for i in range(n)],
    }


def run_data96(m, seq_id, n, drop_chunk):
    cast = synth_cast(n)
    frames = []
    for name in ("time", "DO", "temp", "pressure"):
        var_id = VAR_MAP[name]
        width = max_samples(var_id)
        vals = cast[name]
        for c in range((len(vals) + width - 1) // width):
            block = vals[c * width:(c + 1) * width]
            if name == "DO" and drop_chunk is not None and c == drop_chunk:
                print("  DROPPING %s chunk %d on purpose (%d samples)"
                      % (name, c, len(block)))
                continue
            frames.append((name, c, encode_frame(seq_id, var_id, block, c)))

    # scalars: one value each, single chunk
    for name, val in (("init_DO", 1.0), ("init_pressure", 1013.0), ("batt_v", 12.4)):
        frames.append((name, 0, encode_frame(seq_id, VAR_MAP[name], [val], 0)))

    # round-trip every frame before anything goes on the wire
    print("  verifying %d frames locally..." % len(frames))
    for name, c, f in frames:
        d = decode_frame(f)
        assert d[0] == seq_id and d[6] == c, "header mismatch on %s chunk %d" % (name, c)
        if len(f) > DATA_BYTES:
            raise ValueError("%s chunk %d is %d bytes, over the 96 limit"
                             % (name, c, len(f)))
    print("  round-trip OK, all frames <= %d bytes" % DATA_BYTES)

    for name, c, f in frames:
        send_frame(m, f)
        print("    sent %-14s chunk %-2d  %d bytes" % (name, c, len(f)))
        time.sleep(0.05)

    send_frame(m, encode_frame(seq_id, VAR_ID_FRAME_END, [0], 0))
    print("    sent frame_end (var_id %d)" % VAR_ID_FRAME_END)

    print("\n  expected on the ground, seq %d, %d samples:" % (seq_id, n))
    for name in ("DO", "temp", "pressure"):
        v = cast[name]
        print("    %-9s first %.2f  last %.2f" % (name, v[0], v[-1]))
    if drop_chunk is not None:
        w = max_samples(VAR_MAP["DO"])
        print("    DO should show %d None entries at index %d..%d"
              % (w, drop_chunk * w, drop_chunk * w + w - 1))


# MAVLink severities. Mission Planner styles the low numbers as errors; which
# ones reach the HUD versus the Messages tab is what this flag is for.
SEVERITIES = {
    "emergency": 0, "alert": 1, "critical": 2, "error": 3,
    "warning": 4, "notice": 5, "info": 6, "debug": 7,
}


def send_status(m, text, sev=6):
    """Byte-for-byte the same call gcs_status() makes, with severity exposed."""
    payload = (PREFIX + text)[:MAX_LEN].encode("ascii", "replace")
    m.mav.statustext_send(sev, payload)
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
    ap.add_argument("--dwell", type=int, metavar="N",
                    help="repaint one message N times, 'gap' seconds apart, with "
                         "a countdown suffix. Mission Planner repaints the high "
                         "priority line per message rather than holding it, so "
                         "this is how long an alert stays readable.")
    ap.add_argument("--gap", type=float, default=1.0,
                    help="seconds between repaints for --dwell (default 1.0)")
    ap.add_argument("--sev", default="info",
                    help="severity: name (emergency alert critical error warning "
                         "notice info debug) or 0-7. Default info (6), matching "
                         "gcs_status(). Use 'sweep' to send one of every level.")
    ap.add_argument("--data96", type=int, metavar="N",
                    help="instead of status text, send a synthetic N-sample cast "
                         "as DATA96 frames plus frame_end")
    ap.add_argument("--seq", type=int, default=None,
                    help="seq_id for the synthetic cast (default: from the clock)")
    ap.add_argument("--drop-chunk", type=int, default=None, metavar="K",
                    help="omit DO chunk K to exercise the 081326 gap handling")
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
    if args.data96 is not None:
        seq = args.seq if args.seq is not None else int(time.time()) % 100000
        print("sending a synthetic %d-sample cast as seq %d\n"
              % (args.data96, seq))
        try:
            run_data96(m, seq, args.data96, args.drop_chunk)
        except Exception as e:
            print("  FAILED: %s" % e)
            return 1
        print("")
        print("On the ground, watch the MAVProxy console:")
        print("  '[haucs] RX cast started (seq %d)'  -> frames arriving and gate OPEN" % seq)
        print("  '[haucs] frame end received'        -> commit ran")
        print("  '[haucs] DATA96 received but DROPPED: gate closed'")
        print("        -> frames ARRIVED but were gated. Transport works. The gate")
        print("           needs servo_mon state 1: RC8 >= 1800, or within 20s of")
        print("           the falling edge. Hold the winch switch high and retry.")
        print("  nothing at all                      -> frames are not reaching the GCS")
        return 0

    # --dwell: emulate the ground module's repeat=N so the dwell time can be
    # tuned here without redeploying __init__.py.
    if args.dwell:
        try:
            dsev = SEVERITIES[str(args.sev).lower()]
        except KeyError:
            dsev = int(args.sev)
        print("repainting %d times at severity %d, %.1fs apart\n"
              % (args.dwell, dsev, args.gap))
        for i in range(args.dwell):
            shown = send_status(m, "%s (%d)" % (args.text, args.dwell - i), dsev)
            print("  %s" % shown)
            if i < args.dwell - 1:
                time.sleep(args.gap)
        print("\nTotal on-screen time was about %.1fs. If that is still too")
        print("short, raise --dwell; if the repaints collapsed into one, MP is")
        print("de-duplicating and the countdown suffix is not enough.")
        return 0

    # --sev sweep: one message per severity, so a single run shows which levels
    # Mission Planner puts on the HUD and which only reach the Messages tab.
    if str(args.sev).lower() == "sweep":
        print("sweeping all 8 severities as sys=%d comp=%d\n"
              % (args.sysid, args.compid))
        for name in ("emergency", "alert", "critical", "error",
                     "warning", "notice", "info", "debug"):
            lvl = SEVERITIES[name]
            try:
                shown = send_status(m, "sev %d %s" % (lvl, name), lvl)
                print("  sent sev=%d %-9s : %s" % (lvl, name, shown))
            except Exception as e:
                print("  sev=%d FAILED: %s" % (lvl, e))
            time.sleep(max(args.interval, 1.0))
        print("\nCheck which of the 8 reached the HUD and which only the")
        print("Messages tab. That is the severity threshold for this MP build.")
        return 0

    try:
        sev = SEVERITIES[str(args.sev).lower()]
    except KeyError:
        try:
            sev = int(args.sev)
            if not 0 <= sev <= 7:
                raise ValueError
        except ValueError:
            print("bad --sev %r; use a name, 0-7, or 'sweep'" % args.sev)
            return 1

    print("sending %d message(s) at severity %d, %.1fs apart\n"
          % (args.count, sev, args.interval))

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
            shown = send_status(m, text, sev)
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
