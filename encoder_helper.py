from pymavlink import mavutil
import time, csv, struct, os, json, random
from collections import deque
import sys

# network
HOST_STR = "tcpin:0.0.0.0:5770"
PI_SYSID = 200
PI_COMP  = 190
HEARTBEAT_RATE=1.0

# payload and header for encoding
DATA_BYTES = 96
HDR_LEN = 8   # seq_id 32bit(4)  varbyte (variable type uint8)  base (int16)  len (uint8)
# MAX_SAMPLES is per-variable now; see max_samples(var_id) below.
# ---- Residue coding, MUST match the other side exactly -------------------
#
# Each frame carries: var_base (int16) + N residues, where
#     residue = round((value - var_base) * SCALE_MAP[var_id])
# and var_base = round(mean of the chunk).
#
# Two things bound the scale:
#   1. the residue must fit its integer width
#   2. var_base is rounded to an integer, which alone costs up to 0.5
#
# Worst-case |value - var_base| measured over 8 real casts (BP2, Aug 2026):
#     DO        0.41 ratio      temp   2.07 C
#     pressure  64.1 hPa        batt   0.22 V
# Scales below keep at least 2x margin on those figures.
#
# Pressure is the exception: 64 hPa of spread against an int8 limit of 127
# caps it at scale 1, i.e. 1 hPa = 0.40 in of depth. That is adequate at a
# fast ascent but becomes coarser than the sample spacing once the winch is
# slowed to resolve near-bottom structure. It therefore uses int16 residues,
# which costs one extra frame per cast and lifts the safe scale to ~255.

SCALE = 32          # fallback for any var_id not listed

SCALE_MAP = {
    0: 1,      # time           sample index, integer
    1: 100,    # DO             0.01 ratio, matches the sensor's own 2 dp
    2: 32,     # temp           0.031 C; 64 would clip on observed 2.07 C spread
    3: 100,    # pressure       0.01 hPa = 0.004 in   (int16, see WIDTH_MAP)
    4: 32,     # init_DO        constant per cast, base captures it
    5: 100,    # init_pressure  0.01 hPa; scale 1 was discarding the decimals
    6: 100,    # batt_v         0.01 V
}

# Residue width in bytes. Anything not listed is 1 (int8).
WIDTH_MAP = {
    3: 2,      # pressure needs int16
}

def residue_width(var_id):
    return WIDTH_MAP.get(int(var_id), 1)

def residue_fmt(var_id):
    return "h" if residue_width(var_id) == 2 else "b"

def residue_limits(var_id):
    return (-32768, 32767) if residue_width(var_id) == 2 else (-128, 127)

def max_samples(var_id):
    return (DATA_BYTES - HDR_LEN) // residue_width(var_id)

# set or read the two high bits in var_len (payload[7])
FLAG_NONE = 0
FLAG_EOF  = 1  # end of frame
FLAG_SOF  = 2  # optional start
FLAG_SOLO = 3  # optional solo

# buffer json on disk
BUFFER_PATH = "outbox.json"
failed = {}  # { "seq_id": [ { "var_type": int, "payload": [ints] }, ... ] }


# will pad init_DO and init_pressure to same length as the actual data.
VAR_MAP = {"time": [], "DO": [], "temp": [], "pressure": [], "init_DO":[],"init_pressure":[],"batt_v":[]}
SEND_ORDER = [k for k in VAR_MAP]
# ---------------- helpers ----------------

# prep simulation data
def load_csv(path):
    cols = {name: [] for name in SEND_ORDER}
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            for name in SEND_ORDER:
                val = row[name]
                if val != "":
                    cols[name].append(float(val))
    return cols

########### set the flag to indicate this is new data or resend
# usign the top bit of the variable_type for this
def set_var_byte_resend(payload_bytes, is_resend):
    vb = payload_bytes[4]        # offset for uint32 seq header
    vb = vb & 0x7F               # clear top bit
    if is_resend:
        vb = vb | 0x80           # set top bit
    payload_bytes[4] = vb

# return input stream in frame of length n
def chunker(values, n):
    for i in range(0, len(values), n):
        yield values[i:i+n]
        
# This function take input (data_cols) and generate a queue of 
# packet payloads ready to be sent
# the variables is now sent in sequential way:
# time_packet1, time_packet2... do_packet1, do_packet2,... 
# temp_packetN... pressure_packtN

def build_frames(values, var_id, start_seq, is_resend=False):
    seq = start_seq
    scale = SCALE_MAP.get(int(var_id), SCALE)
    fmt = residue_fmt(var_id)
    lo, hi = residue_limits(var_id)
    n_max = max_samples(var_id)

    if not values:
        return

    for chunk in chunker(values, n_max):
        if not chunk:
            continue

        var_base = int(round(sum(chunk) / len(chunk)))
        residues = []
        clipped = 0

        for v in chunk:
            r = int(round((v - var_base) * scale))
            if r < lo:
                r = lo
                clipped += 1
            if r > hi:
                r = hi
                clipped += 1
            residues.append(r)

        if clipped:
            # Silent clipping is how the old flat SCALE=32 destroyed pressure
            # casts, so make it visible rather than letting it pass.
            print("WARN encoder: var %d clipped %d of %d residues at scale %d"
                  % (var_id, clipped, len(residues), scale))

        var_len = len(residues)
        var_byte = (int(var_id) & 0x7F) | (0x80 if is_resend else 0)

        header = struct.pack(
            "!IBhB",
            seq & 0xFFFFFFFF,
            var_byte & 0xFF,
            var_base,
            var_len & 0xFF,
        )

        payload = bytearray(header + struct.pack("!" + fmt * var_len, *residues))

        if len(payload) < DATA_BYTES:
            payload.extend(b"\x00" * (DATA_BYTES - len(payload)))

        yield seq, payload
        seq = (seq + 1) & 0xFFFFFFFF

'''
def build_frames(values, var_id, start_seq):
    seq = start_seq
    if not values:  # control/marker frame
        var_byte = (int(var_id) & 0x7F) | (0x80 if is_resend else 0)
        var_base = 0 if base is None else int(base)
        vlen = 0
        len_with_flags = ((flag & 0x03) << 6) | vlen
        header = struct.pack("!IBhB", seq_id & 0xFFFFFFFF, var_byte, var_base, len_with_flags)
        payload = header + b"\x00" * (DATA_BYTES - len(header))
        return payload    
    
    for chunk in chunker(values, MAX_SAMPLES):
        if not chunk:
            continue
        # using the average as the base. 
        # hower, this can also be the minimum value in the frame
        # if using minimum value, residue should be be uint8.
        var_base = int(round(sum(chunk) / len(chunk)))
        residues = []
        for v in chunk:            
            r = int(round((v - var_base) * SCALE))
            # these two lines will make sure the values are int8
            # the actual convertion to int8 was done in struck.pack
            if r < -128: r = -128
            if r >  127: r =  127
            residues.append(r)
        var_len = len(residues)
        var_byte = var_id & 0x7F   # turn off resend bit for new packet
        
        # "!IBhB" convert to Big Endian format
        # Code  Bytes   Type    variable
        # I 4   unsigned 32bit  seq_id
        # B 1   unsigned char   var_type
        # h 2   signed short    var_base
        # B 1   unsigned char   val_len        
        header = struct.pack("!IBhB", seq & 0xFFFFFFFF, var_byte & 0xFF, var_base, var_len & 0xFF)
        payload = bytearray(header + struct.pack("!" + "b"*var_len, *residues))
        # padding zeros so that each packet will have the same length
        if len(payload) < DATA_BYTES:
            payload.extend(b"\x00" * (DATA_BYTES - len(payload)))
        
        # yield will continue generate the packet (one at a time) for the caller
        yield seq, payload
        
        #reset sequence id to 0 when reach max(uint16)
        seq = (seq + 1) & 0xFFFFFFFF
'''
def _assert_list_like(values, name):
    if not isinstance(values, list):
        raise TypeError(f"'{name}' must be a list, got {type(values).__name__}")

def prepare_per_var_queues(data_cols, start_seq=0):
    per_var = {}
    seq = start_seq

    for name in SEND_ORDER:
        if name not in data_cols:
            continue
        var_id = SEND_ORDER.index(name)

        q = deque()
        # all scalar will be converted to list (init_DO, init_pressure, batt_v...) first
        # so no need to worry about differet data types here.
        for seq_id, payload in build_frames(data_cols[name], var_id, seq):
            q.append((var_id, seq_id, payload))
            seq = (seq_id + 1) & 0xFFFFFFFF
        per_var[name] = q

    return per_var, seq

# ---------- json buffer ----------
def save_buffer(failed, path="outbox.json"):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(failed, f)

def load_buffer(path="outbox.json"):
    """Read buffer from disk and return a dict."""
    import json, os
    if os.path.exists(path):
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)
    return {}

def add_failed(failed, seq_id, var_type, payload, path="outbox.json"):
    """Add one failed frame to buffer and return updated dict."""
    sid = str(int(seq_id))
    rec = {"var_type": int(var_type), "payload": list(payload)}
    failed.setdefault(sid, []).append(rec)
    save_buffer(failed, path)
    return failed

def remove_one_success(failed, seq_id, path="outbox.json"):
    """Remove first success record from buffer and return updated dict."""
    sid = str(int(seq_id))
    if sid in failed and failed[sid]:
        failed[sid].pop(0)
        if not failed[sid]:
            del failed[sid]
        save_buffer(failed, path)
    return failed

def buffer_queue(q, failed, path="outbox.json"):
    while q:
        var_type, seq_id, payload = q.popleft()
        failed = add_failed(failed, seq_id, var_type, payload, path)
    return failed

def buffer_all_remaining(per_var, failed, path="outbox.json"):
    """Buffer all remaining frames from per_var queues into failed."""
    for name, q in per_var.items():
        if q:
            while q:
                var_type, seq_id, payload = q.popleft()
                failed = add_failed(failed, seq_id, var_type, payload, path)
    return failed

def send_packet(m, var_type, payload_bytes, seq_id):
    try:
        set_var_byte_resend(payload_bytes, False)  # live send flag off
        m.mav.data96_send(var_type, len(payload_bytes), bytes(payload_bytes))
        return True
    except Exception:
        print(f"Pi send error seq {seq_id} buffering all remaining")
        return False

def resend_buffer(m, failed, path="outbox.json"):
    if not failed:
        print("no buffered records")
        return failed
    total = sum(len(v) for v in failed.values())
    print(f"buffered count {total}")
    for sid in sorted(failed.keys(), key=lambda x: int(x)):
        for rec in list(failed[sid]):
            var_type = rec["var_type"]
            payload  = bytearray(rec["payload"])
            set_var_byte_resend(payload, True)  # resends only here
            seq_id = int(sid)
            try:
                m.mav.data96_send(var_type, len(payload), bytes(payload))
                failed = remove_one_success(failed, seq_id, path)
            except Exception:
                print(f"resend failed seq {seq_id} keep buffered")
                return failed
    return failed

def send_or_buffer_all(m, per_var, send_order, failed, path="outbox.json"):
    while True:
        any_sent = False
        for name in send_order:
            q = per_var.get(name)
            if not q:
                continue
            if q:
                var_type, seq_id, payload = q[0]
                ok = send_packet(m, var_type, payload, seq_id)  # <-- uses your name
                if not ok:
                    failed = add_failed(failed, seq_id, var_type, payload, path)
                    q.popleft()
                    failed = buffer_all_remaining(per_var, failed, path)
                    return failed, False
                q.popleft()
                any_sent = True
        if not any_sent:
            return failed, True

# ---------- entry points ----------
def prep_sim_data(csv_path="input.csv"):
    print("Pi loading csv")
    data_cols = load_csv(csv_path)
    return data_cols

##########################
VAR_ID_FRAME_END = 127  # indicating frame end
FRAME_END_RESEND = 3    # send frame end 3 times when live transmission succeeds

def send_payload(m, data_cols, state, path="outbox.json"):
    if "failed" not in state:
        state["failed"] = load_buffer(path)

    # Build all sensor packets. state["seq"] becomes the next free sequence.
    per_var, state["seq"] = prepare_per_var_queues(data_cols, state["seq"])

    # Send sensor packets, buffering any unsent remainder on failure.
    state["failed"], done = send_or_buffer_all(
        m, per_var, SEND_ORDER, state["failed"], path
    )

    if done:
        print("Pi done sending batch")
    else:
        print("Pi buffered remaining after failure-network link is down")

    # FRAME_END gets its own sequence AFTER every data packet in this sample.
    # This makes buffered replay deterministic: resend_buffer() sorts by seq,
    # therefore all missing data is resent before FRAME_END is resent.
    end_seq = state["seq"]
    ctrl_vals = list(b"CONTROL")

    gen = build_frames(ctrl_vals, VAR_ID_FRAME_END, end_seq)
    try:
        _, end_pkt = next(gen)
    except StopIteration:
        return

    # Reserve the control packet sequence before returning from this call.
    state["seq"] = (end_seq + 1) & 0xFFFFFFFF

    if not done:
        # Some data is already in outbox.json, so FRAME_END must be buffered too.
        # Otherwise the base station could commit a partial sampling event.
        state["failed"] = add_failed(
            state["failed"], end_seq, VAR_ID_FRAME_END, end_pkt, path
        )
        print(f"Pi buffered FRAME_END seq {end_seq}")
        return

    # All data packets were accepted for live transmission. Send the required
    # FRAME_END once; if that send itself fails, preserve it in the outbox.
    if not send_packet(m, VAR_ID_FRAME_END, end_pkt, end_seq):
        state["failed"] = add_failed(
            state["failed"], end_seq, VAR_ID_FRAME_END, end_pkt, path
        )
        print(f"Pi buffered FRAME_END seq {end_seq}")
        return

    # Additional copies are redundancy only. The receiver must ignore duplicate
    # FRAME_END packets with the same seq.
    for _ in range(FRAME_END_RESEND - 1):
        send_packet(m, VAR_ID_FRAME_END, end_pkt, end_seq)
        time.sleep(0.1)
