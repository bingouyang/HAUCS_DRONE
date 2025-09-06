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
DATA_BYTES = 64
HDR_LEN = 8   # seq_id 32bit(4)  varbyte (variable type uint8)  base (int16)  len (uint8)
MAX_SAMPLES = (DATA_BYTES - HDR_LEN) // 1  # int8 residues
SCALE = 32    # tradeoff between accuracy (higher) vs dynamic range (lower). 

# set or read the two high bits in var_len (payload[7])
FLAG_NONE = 0
FLAG_EOF  = 1  # end of frame
FLAG_SOF  = 2  # optional start
FLAG_SOLO = 3  # optional solo

# variables
SEND_ORDER = ["time","do", "temp", "press"] 
VAR_MAP = {"time": 0, "do": 1, "temp": 2, "press": 3}

# buffer json on disk
BUFFER_PATH = "outbox.json"
failed = {}  # { "seq_id": [ { "var_type": int, "payload": [ints] }, ... ] }

# ---------------- helpers ----------------

# prep simulation data
def load_csv(path):
    cols = {"do": [], "press": [], "temp": [], "time": []}
    with open(path, newline="") as f:
        r = csv.DictReader(f)
        headers = [h.strip().lower() for h in r.fieldnames]
        for row in r:
            if "time" in headers:
                cols["time"].append(float(row["time"]))
            cols["do"].append(float(row["do"]))
            cols["press"].append(float(row["press"]))
            cols["temp"].append(float(row["temp"]))
    return cols

# this is to allow the pi to reconnect if PC goes down and come back up.
def wait_for_pc(m):
    print("Pi waiting for PC heartbeat")
    while True:
        msg = m.recv_match(blocking=True, timeout=1.0)
        if msg and msg.get_type() == "HEARTBEAT" and msg.get_srcSystem() != PI_SYSID:
            print("Pi got PC heartbeat")
            return

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

def build_frames(values, var_id, start_seq):
    seq = start_seq
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
        # Code	Bytes	Type	        variable
        # I	4	unsigned 32bit	seq_id
        # B	1	unsigned char	var_type
        # h	2	signed short	var_base
        # B	1	unsigned char   val_len        
        header = struct.pack("!IBhB", seq & 0xFFFFFFFF, var_byte & 0xFF, var_base, var_len & 0xFF)
        payload = bytearray(header + struct.pack("!" + "b"*var_len, *residues))
        # padding zeros so that each packet will have the same length
        if len(payload) < DATA_BYTES:
            payload.extend(b"\x00" * (DATA_BYTES - len(payload)))
        
        # yield will continue generate the packet (one at a time) for the caller
        yield seq, payload
        
        #reset sequence id to 0 when reach max(uint16)
        seq = (seq + 1) & 0xFFFFFFFF

# This function take input (data_cols) and generate a queue of 
# packet payloads ready to be sent
# the variables is now sent in sequential way:
# time_packet1, time_packet2... do_packet1, do_packet2,... 
# temp_packetN... pressure_packtN
def prepare_per_var_queues(data_cols, start_seq=0):
    per_var = {}
    seq = start_seq
    for name in data_cols:
        if name not in VAR_MAP:
            continue
        q = deque()
        for seq_id, payload in build_frames(data_cols[name], VAR_MAP[name], seq):
            q.append((VAR_MAP[name], seq_id, payload))
            seq = (seq_id + 1) & 0xFFFFFFFF
        per_var[name] = q
    return per_var, seq

# ---------- json buffer ----------
def save_buffer():
    with open(BUFFER_PATH, "w") as f:
        json.dump(failed, f)

def load_buffer():
    global failed
    if os.path.exists(BUFFER_PATH):
        with open(BUFFER_PATH, "r") as f:
            failed = json.load(f)
    else:
        failed = {}

def add_failed(seq_id, var_type, payload_bytes):
    sid = str(int(seq_id))
    rec = {"var_type": int(var_type), "payload": list(payload_bytes)}
    failed.setdefault(sid, []).append(rec)
    save_buffer()

def remove_one_success(seq_id):
    sid = str(int(seq_id))
    if sid in failed and failed[sid]:
        failed[sid].pop(0)
        if not failed[sid]:
            del failed[sid]
        save_buffer()

def buffer_queue(q):
    while q:
        var_type, seq_id, payload = q.popleft()
        add_failed(seq_id, var_type, payload)

def buffer_all_remaining(per_var):
    for name, q in per_var.items():
        if q:
            buffer_queue(q)

# ---------- send and resend ----------
######WIP!!! TCP/IP should assure delivery unless the link goes down...
# But when at the MAVLINK application layer, the packets still may not be consumed
# even if the link is up. So we do need active ack.

def send_packet_tcp(m, var_type, payload_bytes, seq_id):
    try:
        set_var_byte_resend(payload_bytes, False)  # live send flag off
        m.mav.data64_send(var_type, len(payload_bytes), bytes(payload_bytes))
        return True
    except Exception:
        print(f"Pi send error seq {seq_id} buffering all remaining")
        return False

def resend_buffer(m):
    if not failed:
        print("Pi no buffered records")
        return
    total = sum(len(v) for v in failed.values())
    print(f"Pi buffered count {total}")
    for sid in sorted(failed.keys(), key=lambda x: int(x)):
        for rec in list(failed[sid]):
            var_type = rec["var_type"]
            payload  = bytearray(rec["payload"])
            set_var_byte_resend(payload, True)  # resends only here
            seq_id = int(sid)
            try:
                m.mav.data64_send(var_type, len(payload), bytes(payload))
                remove_one_success(seq_id)
            except Exception:
                print(f"Pi resend failed seq {seq_id} keep buffered")
                return

def send_or_buffer_all(m, per_var, send_order):
    while True:
        any_sent = False
        for name in send_order:
            q = per_var.get(name)
            if not q:
                continue
            if q:
                var_type, seq_id, payload = q[0]
                ok = send_packet_tcp(m, var_type, payload, seq_id)
                if not ok:
                    add_failed(seq_id, var_type, payload)
                    q.popleft()
                    buffer_all_remaining(per_var)
                    return False
                q.popleft()
                any_sent = True
        if not any_sent:
            return True

# ---------- entry points ----------
def prep_sim_data(csv_path="input.csv"):
    print("Pi loading csv")
    data_cols = load_csv(csv_path)
    return data_cols


def init_network():
    print("Pi opening mavlink tcp")
    m = mavutil.mavlink_connection(HOST_STR, source_system=PI_SYSID, source_component=PI_COMP)

    # Wait for a real MAVLink heartbeat from the PC
    print("Pi waiting heartbeat from pc")
    last_hb_out = 0.0
    while True:
        # send our own heartbeat at 1 Hz while waiting
        now = time.time()
        if now - last_hb_out >= HEARTBEAT_RATE:
            m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                 mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0,
                                 mavutil.mavlink.MAV_STATE_ACTIVE)
            last_hb_out = now
        try:
            hb = m.wait_heartbeat(timeout=1)
            if hb:
                break
        except Exception:
            pass
    print("Pi client connected heartbeat received")

    # resend buffer packets (due to network link down) if there is any.
    load_buffer()
    resend_buffer(m)
    return m

def send_payload(m, data_cols, state):
    # create packets 
    per_var, state["seq"] = prepare_per_var_queues(data_cols, state["seq"])

    done = send_or_buffer_all(m, per_var, SEND_ORDER)
    if done:
        print("Pi done sending batch")
    else:
        print("Pi buffered remaining after failure-network link is down")

#######################################################
def run_server():
    while True:
        # open listener
        m = mavutil.mavlink_connection(f"tcpin:0.0.0.0:{PORT}",
                                       source_system=PI_SYSID,
                                       source_component=PI_COMP)
        try:
            wait_for_pc(m)
            next_hb = time.time() + 0.3
            while True:
                # periodic heartbeat
                now = time.time()
                if now >= next_hb:
                    m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                         mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0,
                                         mavutil.mavlink.MAV_STATE_ACTIVE)
                    next_hb = now + HEARTBEAT_RATE

                resend_buffer(mav)
                time.sleep(1)
    
                send_payload(mav, cols,state)  # main step
    
                ########  This is for simulation ######################
                # wait for ~5-10 seconds and repeat
                wait_time = random.uniform(5, 10)
                print(f"Pi waiting {wait_time:.1f} seconds before next send")
                time.sleep(wait_time)
                now = time.time()                
                
        except (ConnectionResetError, OSError) as e:
            print(f"Pi connection lost {e} waiting for new client")
            # loop restarts and re-listens
        except KeyboardInterrupt:
            print("Pi exiting")
            break
            
################### main function #####################
state = {"seq": 0} #initialize sequence counter

mav = init_network() # create the tcp/ip link
csv_path = sys.argv[1] if len(sys.argv) > 1 else "input.csv"
cols=prep_sim_data(csv_path) # create simulation data
last_hb = 0.0

run_server()