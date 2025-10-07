
# ble_thread.py
# Simple BLE worker thread for Raspberry Pi (no GUI).
# Signature matches other threads in your codebase:
#   def ble_thread(stop_evt, q_ble, cfg, st):
#
# Behavior per your use case:
#   1) On startup: connect BLE, request init_do & init_pressure.
#   2) When you tell it to start sampling (via q_ble 'START'), it starts sampling.
#   3) It keeps trying to reconnect if underwater/out of range.
#      Once reconnected and you request STOP_AND_FETCH:
#         - it stops sampling, downloads data, and prepares columns for GCS.
#         - it writes results into st['last_cols'] and flips st['gcs_ready']=True.
#
# Commands (put dicts into q_ble):
#   {'cmd':'START'}                 -> start sampling
#   {'cmd':'STOP_AND_FETCH'}        -> stop sampling + fetch + stage 'last_cols'
#   {'cmd':'GET_BATT'}              -> update battery fields in st
#   {'cmd':'RECONNECT'}             -> force a reconnect attempt
#
# Shared state 'st' fields (updated by this thread):
#   st['ble']           : BluetoothReader instance
#   st['connected']     : bool
#   st['sampling']      : bool
#   st['init_done']     : bool (inits successfully fetched once)
#   st['last_cols']     : dict ready for GCS encoder (see _package_sdata)
#   st['gcs_ready']     : bool (True when 'last_cols' is freshly updated)
#   st['last_status']   : str (human-readable last status)
#
# Minimal dependencies: PyQt5 for QMutex because your BluetoothReader uses it.

import time, queue, logging
from typing import Dict, Any
from PyQt5.QtCore import QMutex
from bt_helper import BluetoothReader

log = logging.getLogger(__name__)

# ---- helpers ----
def _start_sampling(reader: BluetoothReader):
    # Try common names; fall back to raw command string
    if hasattr(reader, 'start_sampling'):
        reader.start_sampling()
    elif hasattr(reader, 'set_sample_start'):
        reader.set_sample_start()
    else:
        reader.send_receive_command('sstart')

def _stop_sampling(reader: BluetoothReader):
    if hasattr(reader, 'stop_sampling'):
        reader.stop_sampling()
    elif hasattr(reader, 'set_sample_stop'):
        reader.set_sample_stop()
    else:
        reader.send_receive_command('sstop')

def _package_sdata(sdata: Dict[str, Any]) -> Dict[str, Any]:
    """Pack BluetoothReader.sdata -> compact columns for GCS (adjust names as needed)."""
    do_vals = sdata.get('do_vals') or []
    t_vals  = sdata.get('temp_vals') or sdata.get('temperature') or []
    p_vals  = sdata.get('pressure_vals') or sdata.get('pressure') or []
    ts_vals = sdata.get('ts_vals') or sdata.get('timestamps') or list(range(len(do_vals)))
    return {
        'time': ts_vals,
        'do': do_vals,
        'temp': t_vals,
        'press': p_vals,
        'init_DO': sdata.get('init_do'),
        'init_pressure': sdata.get('init_pressure'),
        'batt_v': sdata.get('battv'),
    }

# ---- main worker ----
def ble_thread(stop_evt, q_ble, cfg, st):
    """Run BLE loop with a single command queue and shared state dict."""
    # Config with safe defaults
    poll_sec   = float(cfg.get('POLL_SEC', 1.0)) if isinstance(cfg, dict) else 1.0
    backoff_lo = float(cfg.get('RETRY_MIN', 0.5)) if isinstance(cfg, dict) else 0.5
    backoff_hi = float(cfg.get('RETRY_MAX', 5.0)) if isinstance(cfg, dict) else 5.0

    # State init
    st.setdefault('mutex', QMutex())
    st.setdefault('ble', None)
    st.setdefault('connected', False)
    st.setdefault('sampling', False)
    st.setdefault('init_done', False)
    st.setdefault('last_cols', None)
    st.setdefault('gcs_ready', False)
    st.setdefault('last_status', 'init')

    reader = st['ble'] = BluetoothReader(st['mutex'])
    backoff = backoff_lo

    # --- initial connect + init pulls ---
    if reader.connect():
        st['connected'] = True
        st['last_status'] = 'connected'
        try:
            reader.get_init_do()
            reader.get_init_pressure()
            st['init_done'] = True
        except Exception:
            # don't crash if inits fail; we'll retry later
            st['last_status'] = 'connected_no_inits'
    else:
        st['connected'] = False
        st['last_status'] = 'disconnected'

    last_poll = 0.0
    pending_after_reconnect = None  # None | 'START' | 'STOP_AND_FETCH'

    while not stop_evt.is_set():
        now = time.time()
        # Periodic connection maintenance
        if now - last_poll >= poll_sec:
            last_poll = now
            # Check connection; if lost, attempt reconnect with backoff
            try:
                still_ok = bool(reader.check_connection_status())
            except Exception:
                still_ok = False

            if not still_ok:
                st['connected'] = False
                st['last_status'] = 'disconnected'
                time.sleep(backoff)
                try:
                    if reader.reconnect():
                        st['connected'] = True
                        st['last_status'] = 'reconnected'
                        backoff = backoff_lo
                        # On reconnect, handle any pending action
                        if pending_after_reconnect == 'STOP_AND_FETCH':
                            try:
                                _stop_sampling(reader)
                                ok = bool(reader.get_sample_data())
                                st['sampling'] = False
                                if ok:
                                    st['last_cols'] = _package_sdata(reader.sdata)
                                    st['gcs_ready'] = True
                                    st['last_status'] = 'fetched_after_reconnect'
                            except Exception:
                                st['last_status'] = 'fetch_failed_after_reconnect'
                            pending_after_reconnect = None
                        elif pending_after_reconnect == 'START':
                            try:
                                _start_sampling(reader)
                                st['sampling'] = True
                                st['last_status'] = 'sampling_started_after_reconnect'
                            except Exception:
                                st['last_status'] = 'start_failed_after_reconnect'
                            pending_after_reconnect = None
                    else:
                        # exponential backoff up to hi
                        backoff = min(backoff_hi, max(backoff_lo, backoff * 1.5))
                except Exception:
                    # keep backing off if reconnect throws
                    backoff = min(backoff_hi, max(backoff_lo, backoff * 1.5))
            else:
                st['connected'] = True
                st['last_status'] = 'connected'
                # cheap housekeeping
                try:
                    reader.get_sample_size()
                except Exception:
                    pass

        # Handle queued commands
        try:
            cmd = q_ble.get(timeout=0.1)
        except queue.Empty:
            continue

        action = (cmd.get('cmd') or '').upper()
        if action == 'START':
            if st['connected']:
                try:
                    _start_sampling(reader)
                    st['sampling'] = True
                    st['last_status'] = 'sampling_started'
                except Exception:
                    st['last_status'] = 'start_failed'
            else:
                pending_after_reconnect = 'START'
                st['last_status'] = 'queued_start_until_reconnect'

        elif action == 'STOP_AND_FETCH':
            if st['connected']:
                try:
                    _stop_sampling(reader)
                    ok = bool(reader.get_sample_data())
                    st['sampling'] = False
                    if ok:
                        st['last_cols'] = _package_sdata(reader.sdata)
                        st['gcs_ready'] = True
                        st['last_status'] = 'fetched'
                    else:
                        st['last_status'] = 'fetch_empty'
                except Exception:
                    st['last_status'] = 'fetch_failed'
            else:
                pending_after_reconnect = 'STOP_AND_FETCH'
                st['last_status'] = 'queued_fetch_until_reconnect'

        elif action == 'GET_BATT':
            if st['connected']:
                try:
                    v, bstat = reader.get_battery()
                    # reader.sdata already updated; nothing else needed
                    st['last_status'] = f'batt:{v:.2f}V,{bstat}'
                except Exception:
                    st['last_status'] = 'batt_failed'
            else:
                st['last_status'] = 'batt_skipped_disconnected'

        elif action == 'RECONNECT':
            # Force a reconnect attempt immediately
            try:
                if reader.reconnect():
                    st['connected'] = True
                    st['last_status'] = 'manual_reconnect_ok'
                    backoff = backoff_lo
                else:
                    st['connected'] = False
                    st['last_status'] = 'manual_reconnect_failed'
            except Exception:
                st['connected'] = False
                st['last_status'] = 'manual_reconnect_exception'

        # Unknown commands are ignored silently for simplicity
