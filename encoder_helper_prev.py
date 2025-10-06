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
MAX_SAMPLES = (DATA_BYTES - HDR_LEN) // 1  # int8 residues
SCALE = 32    # tradeoff between accuracy (higher) vs dynamic range (lower). 
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
VAR_ID_FRAME_END = 250 # indicating frame end.
FRAME_END_RESEND = 3   # send frame end 3 times.
def send_payload(m, data_cols, state, path="outbox.json"):
    """
    Create packets and send them. Uses state["failed"] to avoid a global.
    Keeps the original function name and call pattern.
    """
    # Ensure we have a buffer in state (no global)
    if "failed" not in state:
        state["failed"] = load_buffer(path)
        
    first_seq = state["seq"]  # capture BEFORE creating the packets
    # create packets
    per_var, state["seq"] = prepare_per_var_queues(data_cols, state["seq"])
    # try to send; pass and update the buffer kept in state
    state["failed"], done = send_or_buffer_all(m, per_var, SEND_ORDER, state["failed"], path)

    if done:
        print("Pi done sending batch")
        
    else:
        print("Pi buffered remaining after failure-network link is down")

    # --- end-of-frame marker (values=["control"] as bytes) ---
    ctrl_vals = list(b"CONTROL")      # int8 list; length must be <= 63
    pkt = build_frame(first_seq, VAR_ID_FRAME_END, ctrl_vals, flag=FLAG_SOLO)
    for _ in range(FRAME_END_RESEND): #send frame end multple times times.
        send_packet(m, VAR_ID_FRAME_END, pkt, first_seq)
        time.sleep(0.1)