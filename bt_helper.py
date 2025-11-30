from adafruit_ble import BLERadio
from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
from adafruit_ble.services.nordic import UARTService

import random
import numpy as np
from scipy.optimize import curve_fit
import pandas as pd
import json, logging, time, os, sys
from time import sleep
from subprocess import call

# PyQt kept crashing, maybe just this system... this helps to bypassing it.
try:
    from PyQt5.QtCore import QObject, QThread, pyqtSignal, QMutex, QMutexLocker
except Exception:
    import threading
    class QObject: pass
    class QThread: pass
    def pyqtSignal(*args, **kwargs):
        class _Sig:
            def connect(self, *a, **k): pass
            def emit(self, *a, **k): pass
        return _Sig()
    class QMutex:
        def __init__(self): self._lock = threading.Lock()
        def lock(self): self._lock.acquire()
        def unlock(self): self._lock.release()
        def __enter__(self): self.lock(); return self
        def __exit__(self, *exc): self.unlock()
    class QMutexLocker:
        def __init__(self, m): self._m = m
        def __enter__(self): self._m.lock(); return self._m
        def __exit__(self, *exc): self._m.unlock()

#init logger
logger = logging.getLogger(__name__)

class BluetoothReader(QObject):
    data_updated = pyqtSignal(dict)
    uart_connection = None
    sdata = {'connection':False, 'name':'', 'sample_hz':1, 'battv':0, 'batt_status':"not charging", 'init_p':0, 'init_do':0}
    current_sample_size = 0
    prev_sample_size = 0

   # dictionary of commands (tx) and expected retunrs (rx)
    commands = {
        'init_do': {'tx':'get init_do', 'rx':'init_do', 'size':2},
        'init_ps': {'tx':'get init_p', 'rx':'init_p', 'size':2},
        'battery': {'tx':'batt', 'rx':'v', 'size':4},
        's_reset': {'tx':'sample reset', 'rx':''},
        's_size' : {'tx':'sample size', 'rx':'dsize', 'size':2},
        's_print': {'tx':'sample print', 'rx':'dstart', 'end':'dfinish'},
        'cal_do' : {'tx':'cal do', 'rx':'init do', 'size':2},
        'cal_ps' : {'tx':'cal ps', 'rx':'init p', 'size':2},
        'xmas'   : {'tx':'set light xmas', 'rx':''},
        's_rate' : {'tx':'get sample_hz', 'rx':'sample_hz', 'size':2},
        'max_size' : {'tx':'set max_sample', 'rx':''},
        'pmode' :  {'tx':'set pmode high', 'rx':''},
        's_flag' : {'tx':'get sample_flag', 'rx':'sample_flag', 'size':2},
        's_type' : {'tx':'get sample_type', 'rx':'sample_type', 'size':2},
    }

    def __init__(self, ble_mutex):
        super().__init__()
        self.transmission_timeouts = 0
        self.ble_mutex = ble_mutex
        self.ble = BLERadio()
        self._abort = False

    def connect_by_address(self, addr):
        for attempt in range(1,3):
            try:
                print(f"[{attempt}/3] Connecting to {addr} ...")
                conn = self.ble.connect(addr, timeout=15)
                print("Connected:", conn)
                return conn
            except Exception as e:
                print("Connect failed:", e)
                # small adapter nudge helps sometimes
                time.sleep(1.0)
        return None

    def connect(self):
        logger.debug('starting ble scan')
        connection_success = False
        try:
            for adv in self.ble.start_scan(ProvideServicesAdvertisement):
                print(f"current adv: {adv.complete_name}")
                if UARTService in adv.services:
                    logger.debug(f"found sensor with UART service {adv.complete_name}")
                    print(f"found sensor with UART service {adv.complete_name}")
                    break
            self.ble.stop_scan()
            print(f"found sensor with UART service {adv.complete_name}, address:{adv.address}")

            self.uart_connection = self.connect_by_address(adv.address)
            #self.uart_connection = self.ble.connect(adv)
            if self.uart_connection.connected:
                self.sensor_name = adv.complete_name
                self.sdata['name'] = adv.complete_name[9:]
                self.sdata['connection'] = True
                logger.debug(f"successfully connected to {adv}")
                connection_success = True
                
        except Exception as e:
            logger.error('failed BLE scan %s', e)
            print(f'failed BLE scan {e}')

            connection_success = False
        self.sdata['connection'] = connection_success
        try:
            self.ble.stop_scan()
        except Exception as e:
            logger.error('failed to stop BLE scan %s', e)
        return connection_success

    def check_connection_status(self):
        connected = False
        if not (self.uart_connection and self.uart_connection.connected):
            if self.sdata['connection']:
                logger.debug("uart_connection.connected triggered not connected status in check_connection_status")
        # use timeouts to predict disconnect
        elif self.transmission_timeouts > 0:
            if self.sdata['connection']:
                logger.debug("send_receive timeout triggered not connected status")
        # sensor is connected
        else:
            if self.sdata['connection'] == False:
                logger.info('sdata reported disconnected but uart_connection.connected is true')
            connected = True
        self.sdata['connection'] = connected
        return connected

    def reconnect(self):
        if not (self.uart_connection and self.uart_connection.connected):
            if self.sdata['connection'] != False:
                logger.info('sdata reported connected while uart_connection.connected is false')
                self.sdata['connection'] = False
            return self.connect()
        
        logger.debug('reconnect attempted, already connected, do nothing')
        return True

    def init_sensor_status(self):
        try:
            self.get_init_do()
            self.get_init_pressure()
            self.get_battery()
            self.get_sampling_rate()
        except Exception as e:
            logger.error('failed to init sensor %s', e)
            print(f'failed to init sensor {e}')
            
    def get_init_do(self):
        self.sdata["init_do"] = 0
        self.send_receive_command(self.commands['init_do'])
        return self.sdata['init_do']

    def get_init_pressure(self):
        self.sdata["init_pressure"] = 0
        self.send_receive_command(self.commands['init_ps'])
        return self.sdata['init_pressure']
    
    def get_sampling_rate(self):
        self.sdata["sample_hz"] = 1
        self.send_receive_command(self.commands['s_rate'])
        return self.sdata['sample_hz']
    
    def get_battery(self):
        self.send_receive_command(self.commands['battery'])
        return self.sdata['battv'], self.sdata['batt_status']
    
    def get_sample_size(self):
        return self.send_receive_command(self.commands['s_size'])
    
    def set_pmode(self, in_pmode):
        command = {'tx':f"set pmode {in_pmode}", 'rx':''}
        self.send_receive_command(command)

    def set_sample_reset(self):
        print("in bt_helpfer:set_sample_reset ")
        self.send_receive_command(self.commands['s_reset'])
        self.prev_sample_size = 0
        self.current_sample_size = 0

    def get_sample_data(self):
        msg = self.send_receive_command(self.commands['s_print'], timeout=5)
        print(f'get_sample_data rcvd msg:{msg}')
        return msg[0] == 'dfinish'
    
    def set_calibration_pressure(self):
        return self.send_receive_command(self.commands['cal_ps']) 

    def set_calibration_do(self):
        msg = self.send_receive_command(self.commands['cal_do'])
        if len(msg) > 0:
            if msg[0] == self.commands['cal_do']['rx']:
                return True
        return False

    def set_maxsize(self, in_max):
        command = {'tx':f"set max_size {int(in_max)}", 'rx':''}
        msg = self.send_receive_command(command)
           
    def set_smpl_type(self, in_type):
        command = {'tx':f"set sample_type {in_type}", 'rx':''}
        msg = self.send_receive_command(command)

    def set_sampl_flag(self, in_flag):
        command = {'tx':f"sample_flag {int(in_flag)}", 'rx':''}
        msg = self.send_receive_command(command)
    
    def get_smpl_type(self):
        return self.send_receive_command(self.commands['s_type'])

    def get_sampl_flag(self):
        return self.send_receive_command(self.commands['s_flag'])

    def set_lights(self, pattern):
        command = {'tx':f"set light {pattern}", 'rx':''}
        self.send_receive_command(command)

    def extract_message(self, msg):
        #TODO: break this into individual callbacks for each message
        key, value = msg[0], msg[1:]
        try:
            if key == "init_do":
                self.sdata["init_do"] = float(value[0].strip())

            elif key == "init_p":
                self.sdata["init_pressure"] = float(value[0].strip())

            elif key == 'v' and len(value) == 3:
                self.sdata['battv'] = float(value[0])
                self.sdata['batt_status'] = value[2].strip()

            elif key == 'sample_hz':
                self.sdata['sample_hz'] = float(value[0])

            elif key == "dsize":
                self.prev_sample_size = self.current_sample_size
                self.current_sample_size = int(value[0])         

            elif key == "dstart":
                print(f"received dstart")

                self.data_counter = 0
                self.sdata["do_vals"] = []
                self.sdata["temp_vals"] = []
                self.sdata["pressure_vals"] = []

            elif key == "ts":
                try:
                    do = float(value[2])
                    temp_val = float(value[4])
                    pressure_val = float(value[6])
                except:
                    logger.warning(f'data corrupted in ble transfer {key, value}')
                    do = 0
                    temp_val = 0
                    pressure_val = 0

                self.sdata['do_vals'].append(do)
                self.sdata["temp_vals"].append(temp_val)
                self.sdata["pressure_vals"].append(pressure_val)
                self.data_counter = self.data_counter + 1  
                #print(f"in ts: data_counter:{self.data_counter}, received tsL do:{do}, temp_val:{temp_val}, pressure_val:{pressure_val}")

            elif "dfinish" in key:
                # compare data counter to current sample size
                print(f"received dfinish")

                if self.data_counter != self.current_sample_size:
                    logger.warning(f"size mismatch between data collected on sensor and data received: {self.current_sample_size} vs. {self.data_counter}")
        except:
            logger.warning(f'failed to parse {key} message, received: {value}')

    def send_receive_command(self, command, timeout=1):
        '''
        Returns imediately if not connected
        '''
        
        if not self.uart_connection:
            return ""
        if not self.uart_connection.connected:
            return ""
        
        #handle commands with multiple responses expected
        multiple_outputs = "end" in command
        receiving_array = False
        with QMutexLocker(self.ble_mutex):
            uart_service = self.uart_connection[UARTService]
            msg = "" # return nothing if tx only
            #print(f"ble outgoing cmd: {command['tx']}")

            uart_service.write((command['tx'] + "\n").encode())
            start_time = time.time()
            while len(command['rx']) > 0:
                # check timeout
                if (time.time() - start_time > timeout):
                    logger.debug('timeout triggered')
                    self.transmission_timeouts += 1
                    return ""
                # wait till buffer has something
                if not uart_service.in_waiting:
                    continue
                msg = uart_service.readline().decode()
                msg = msg.replace("\n","")
                msg = msg.lower()
                msg = msg.split(",")
                logger.info(f"ble incoming msg: {msg}, command:{command}")
                #print(f"ble incoming msg: {msg}, command:{command}")
                # check message length if defined
                if command.get('size'):
                    if command.get('size') != len(msg):
                        logger.info("ble message corrupted %s", msg)
                if len(msg) >= 1:
                    logger.debug("message received %s", msg)
                    # handle first response
                    if msg[0] == command['rx']:
                        receiving_array = True
                        self.extract_message(msg)
                        # break if only one output
                        if not multiple_outputs:
                            break
                    # handle other responses
                    elif receiving_array:
                        self.extract_message(msg)
                        if msg[0] == command.get("end"):
                            break
            # reset transmission timeouts on successfull transmission 
            self.transmission_timeouts = 0
            logger.debug(f"sent {command}, received {msg}")
            return msg