from adafruit_ble import BLERadio
from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
from adafruit_ble.services.nordic import UARTService
import board, adafruit_gps

import random, csv
from matplotlib import pyplot as plt
from scipy.optimize import curve_fit
import numpy as np
import pandas as pd
import json, logging, time, os, sys
from time import sleep
from datetime import datetime
import subprocess
import serial
from subprocess import call
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
import firebase_admin
from firebase_admin import credentials,db
############### Running On Startup ###############
# To configure this script to run on startup for unix systems
# add a command to the cron scheduler using crontab.
#
# Run "crontab -e" to open the editor
#
# Paste the following line
#
# @reboot /home/haucs/Desktop/HAUCS/startup.sh buoy/main.py >> /home/haucs/Desktop/HAUCS/buoy/cronlog.log 2>&1
#
# This runs the program when the device is powered on and stores the output in
# the local "cronlog.log" file. Please note that the python script outputs a more detailed
# log in the local "log.log" file.
#
# Let the computer establish a network connection on reboot
# folder = "Desktop/HAUCS/"
folder = "/home/haucs/Desktop/HAUCS/truck_app/data/"

#Handle Inputs
if len(sys.argv) > 1:
    timer_only = sys.argv[1]
else:
    timer_only = "false"

ble = BLERadio()
uart_connection = None

#scheduled_msgs = {"batt":15, "single":5}
# should it be "sample print"
#scheduled_msgs = {"batt":15, "sample print":5}

#msg_timers = {}
#for i in scheduled_msgs:
#    msg_timers[i] = time.time() + random.randint(1,scheduled_msgs[i])

### INIT JSON FILE
sdata = {}

sensor_file = folder + "sensor.json"
header = ['time', 'do', 'temperature', 'pressure']

pond_history = np.array(["unknown"])

fails = {'gps':0, 'batt':0, 'internet':0, }

cred = credentials.Certificate("/home/haucs/Desktop/HAUCS/truck_app/fb_key.json")
app = firebase_admin.initialize_app(cred, {'databaseURL': 'https://haucs-monitoring-default-rtdb.firebaseio.com'})

##### LOGGING #####
logging.basicConfig(format='%(asctime)s %(levelname)s: %(message)s', filename=folder + 'log.log', encoding='utf-8',
                    level=logging.INFO)
logger = logging.getLogger(__name__)
logger.info('Starting')

################# defining functions############################
def writeCSV(ofile, data):
    with open(ofile,'a',newline='') as csvfile:
      writer = csv.writer(csvfile, delimiter=',')
      writer.writerow(data)

def init_file():
    #global folder
    global header
    filePath = "data/"
    date = datetime.now().strftime('%Y-%m-%d-%H-%M-%S')

    if not os.path.exists(filePath):
        os.mkdir(filePath)

    csvFile = filePath + date + ".csv"

    with open(csvFile,'w',newline='') as csvfile:
      writer = csv.writer(csvfile, delimiter=',')
      writer.writerow(header)
    return csvFile

def exp_func(x, a, b, c):
    return a * np.exp(-b * x) + c

def generate_graph(infile):
    df = pd.read_csv(infile)
    df['s'] = df['time'] / 1000

    x = np.arange(100) * 3 / 10

    try:
        popt, pcov = curve_fit(exp_func, df['s'], df['do'])
        y = exp_func(x, *popt)
    except:
        y = x
        logger.warning("curve fitting failed")
    
    # plt.style.use('dark_background')
    with plt.rc_context({'axes.edgecolor':'red', 'xtick.color':'red', 'xtick.labelsize':25, 'ytick.labelsize':25, 'ytick.color':'red', 'figure.facecolor':'black'}):
        plt.figure()
        plt.scatter(df['s'], df['do'], color="red", linewidth=4, alpha=1)
        plt.plot(x, y, color="red", linewidth=4, alpha=0.5)
        plt.xlabel("seconds", color="red", fontsize=25)
        plt.ylabel("% Saturation",  color="red", fontsize=25)
        plt.annotate(str(round(y[-1])) + '%', (x[-1], y[-1]), xytext=(x[70], y[15]), arrowprops={"width":1, "color":"red", "headwidth":6},color="red", fontsize=25)
        plt.savefig(infile[:-4] + ".png", bbox_inches="tight")

def save_json():
    global sdata
    with open(sensor_file, 'w') as outfile:
        json.dump(sdata, outfile)
        
def ble_connect():
    global uart_connection
    print("searching for sensor ...")
    for adv in ble.start_scan(ProvideServicesAdvertisement, timeout=30):
        print('ble:'+adv.services)
        if UARTService in adv.services:
            uart_connection = ble.connect(adv)
            print("connected to: ", adv.complete_name)
            sdata['name'] = adv.complete_name[9:]
            break
    ble.stop_scan()

def ble_uart_read():
    global uart_connection

    if uart_connection:
        uart_service = uart_connection[UARTService]
        if uart_connection.connected:
            outmsg = uart_service.readline().decode()
            return outmsg
        
    return "failed read, no connection"

def ble_uart_write(inmsg):
    global uart_connection

    if uart_connection:
        print("sending: ", inmsg)
        uart_service = uart_connection[UARTService]
        if uart_connection.connected:
            uart_service.write(inmsg.encode())
        else:
            print("failed to send")

##### FIREBASE #####
def restart_firebase(in_app):
    logging.info('Attempting to restart Firebase Connection')
    firebase_admin.delete_app(in_app)
    sleep(60)
    new_app = firebase_admin.initialize_app(cred,
                                            {'databaseURL': 'https://haucs-monitoring-default-rtdb.firebaseio.com'})
    return new_app
        
def find_subsequence_index(A, B):
    len_A = len(A)
    len_B = len(B)
    
    # Iterate through A to find if B exists as a contiguous subsequence
    for i in range(len_A - len_B + 1):
        if A[i:i+len_B].equals(B):
            return i+len_B # return the end in A.
    return -1

def get_pond_id(lat, lng):
    global pond_history
    df = pd.read_csv('sampling_points.csv')
    pond_ids = df.pop('pond')
    pond_gps = df.to_numpy()
    point = np.array([float(lng), float(lat)])
    point_y = np.tile(point, (pond_gps.shape[0], 1))
    #calculate euclidean distances
    distances = np.linalg.norm(pond_gps - point_y, axis=1)
    #calculate minimum distance in meters
    min_dist = distances.min() * 111_000
    #determine if min distance is acceptable
    if min_dist < 100:
        #find pond associated with minimum distance
        pond_id = str(pond_ids[np.argmin(distances)])
    else:
        pond_id = "unknown"
    
    ##############################################################
    # Bing note: the sequence of pond_id can also be 
    # used to detect if a pond is detected incorrectly
    ##############################################################
    ps = pd.read_csv('pond_sequence.csv')
    sequence_ids = ps.pop('sequence_id')
    pd_hist = pd.Series(pond_history)
    cid = find_subsequence_index(sequence_ids, pd_hist)
    # cid is the end of the existing sequence
    # the current pond id should be the next in the sequence
    pond_id_from_sequence = sequence_ids[cid+1]
    
    #update pond_historys - FIFO - keeping 3 entries to check 
    #against the pond sequence table.
    if pond_id == "unknown":
        pass # do not append
    elif len(pond_history) < 2:
        pond_history = np.append(pond_history, [pond_id])
    else:
        pond_history = np.append(pond_history[1:2], [pond_id])
    
    #if common_pond_id != pond_id:
    #    logger.info(f"detected in {pond_id}, most common pond {common_pond_id}, pond_history {pond_history}")
    return pond_id

##### GPS #####
def update_GPS(inp_t):
    gps_time = time.time()
    # noinspection PyBroadException    
    try:
        while time.time() - gps_time < inp_t:
            gps.update()
            sleep(0.5)
        fails['gps'] = 0
        if gps.satellites is not None:
            numsat = gps.satellites
            print(f"satellites in view: {numsat}")
        #else:
        #    print("No satellite data available.")    
    except:
        logger.warning("GPS update routine failed")
        fails['gps'] += 1
        print('GPS update failed')

def get_IP():
    terminalResponse = subprocess.run(['hostname', '-I'], capture_output=True, text=True)
    return terminalResponse.stdout

#######################################################################
######################## Initialization ###############################
#BLE
os.popen('sudo hciconfig hci0 reset')
sleep(10)
try: 
    ble_connect()
except:
    logger.warning("BLE connect failed")
    #fails['ble'] += 1
    print('BLE connect failed')
    
ble_uart_write("set light xmas")
# GPS
i2c = board.I2C()
gps = adafruit_gps.GPS_GtopI2C(i2c)
gps.send_command(b'PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
gps.send_command(b"PMTK220,8000")
update_GPS(5)

cnt = 0
scnt = 0 # this is to count the single 
#do=[] # DO
#p = [] # pressure
#t  = [] # tempuerature

init_do = -1
init_pressure = -1
prev_pond_id = -1
prev_sample_size = -1

init_connect = -1 #set a flag to indicate first time connect to the payload
check_size = -1 # check the dsize

sleep(2)
lng = gps.longitude
lat = gps.latitude
print(f"lng is {lng}, lat is {lat}")

msg = 'sample reset' 
ble_uart_write(msg)
sleep(5)

batt_cnt = 0
# locking the initial DO and initial pressure
init_DO = -1
init_pressure = -1
msg = 'batt'
ble_uart_write(msg)
sleep(5)
msg = 'get init_do'
ble_uart_write(msg)
sleep(3)
#msg = 'get init_pressure'
#ble_uart_write(msg)
#sleep(2)

# Main body
while True:
    if not (uart_connection and uart_connection.connected):
        if sdata.get('connection') != "not connected":
            sdata['connection'] = "not connected"
            save_json()
            
        print("trying to reconnect")
        try:
            ble_connect()
            time.sleep(1)
        except:
            logger.warning("BLE connect failed - maybe underwater")
            #fails['ble'] += 1
            print('BLE connect failed -maybe underwater')  
    else:
        ### update connection status
        if init_connect < 0 : # first time connected to the payload (boot up)
            init_connect = 1
            logger.info('first time connected to the payload (boot up)')
        elif sdata.get('connection') != "connected":
            sdata['connection'] = "connected"
            save_json()
            logger.info('reconnect after disconnect - finished sampling and re-emerge')
        ### send scheduled messages
        # "batt": query battery voltage
        # "calibrate DO": calibrate DO
        # "calibrate pressure": calibrate pressure
        # "save": save settings
        # "single": sampling once
    
        # variations of sample commands:
        # "sample print": send back the samples collected so far
        # "sample reset"/"sample start": reset the sample counter and start sampling
        # "sample stop": stop sampling
        # "sample size":  number of sample acquired so far
    
        #"reset": reset the controller! (not the counter)
        # "do": output current DO value
        
        # monitor the sample size - if it is >0, then samples have been acquired
        if check_size == 1:
            #msg = 'calibrate do'
            #ble_uart_write(msg)
            #sleep(5)
            #msg = 'calibrate pressure'
            #ble_uart_write(msg)
            #sleep(5)                                   
            msg = 'sample size' 
            ble_uart_write(msg)
            sleep(1)
            # check battery status every minute
            if batt_cnt<60:
                batt_cnt = batt_cnt +1
            else:
                msg = 'batt'
                ble_uart_write(msg)
                sleep(1)
                batt_cnt = 0
            #update_GPS(1)
            #sleep(1)
            #print(f"2D Fix: {gps.has_fix}")
            #lng = gps.longitude
            #lat = gps.latitude
        
        ### read incoming messages
        msg = ble_uart_read()
        if len(msg) > 0:
            print(msg)
            msg = msg.split(",")
            
        # This logic is used to monitor the sampling activity since auto sening is being engaged
        # the sample size will increase automatically 
        #print("sample: {msg[0]}")
        print(f'msglen:{len(msg)}: {msg}')
        if (len(msg)==2) and (msg[0] == "b'dsize"):
            cstr = msg[1]
            slen = len(cstr)
            print(f'strlen: {slen}')
            cstrnum = cstr[0:slen-3]
            current_sample_size = float(cstrnum)
            
            print(f"current sample size {current_sample_size}")
            print(f"prev_sample_size: {prev_sample_size}")

            if prev_sample_size<=0 and current_sample_size>0: # sampling started, let's lock the current time
                prev_sample_size = current_sample_size
                # we locking lat/lon and pond id now
                # noinspection PyBroadException
                try:
                    update_GPS(1)
                    sleep(1)
                    lng = gps.longitude
                    lat = gps.latitude
                    if not lng: lng = 0
                    if not lat: lat = 0
                    msg = 'batt'
                    ble_uart_write(msg)
                    sleep(1)

                except:
                    logger.warning("getting gps lat/lng failed")
                    
                # noinspection PyBroadException
                # locking the pond id
                pond_id = get_pond_id(lat, lng)
                # locking the time as well.
                message_time = time.strftime('%Y%m%d_%H:%M:%S', time.gmtime()) #GMT time

                
            elif prev_sample_size > 0:
                if current_sample_size > prev_sample_size: # sampling continuing
                    prev_sample_size = current_sample_size # contiue incrementing
                elif current_sample_size ==  prev_sample_size: # sampling finished
                    # trigger data upload
                    msg = 'sample print'                   
                    ble_uart_write(msg)
                    check_size = -1 # disable check size till the data upload has finished
                    sleep(1)
                    # reset the prev_sample_size counter to zero - prepare for next run.
                    prev_sample_size = 0
        elif (len(msg) == 6) and (msg[0] == 'd'): #return from "single"
            # ''d',DO, 't', temp, 'p', press
            # msg = f"d,{do[i]},t,{temperature[i]},p,{pressure[i]}\n"
            sdata['do'] = float(msg[1])
            sdata['temperature'] = round(9/5 * float(msg[3]) + 32, 1)
            sdata['pressure'] = float(msg[5])
            save_json()
            writeCSV(csv_file, [cnt, msg[1], msg[3], msg[5]])

            #do[scnt] =  float(msg[1]) * 100  # DO
            #t [scnt] = round(9/5 * float(msg[3]) + 32, 1)   # tempuerature
            #p [scnt] = float(msg[5]) # pressure
            #scnt = scnt + 1
        elif (len(msg) == 4) and (msg[0] == 'v'):
            sdata['battv'] = float(msg[1])
            sdata['batt_status'] = msg[3][:-1]
            save_json()
            batt_v = float(msg[1]) #db variable
            print(f'batt_v:{batt_v}')
        elif (len(msg) == 8) and (msg[0] == 'ts'): # sample print
            print("appending: ", msg)
            writeCSV(csv_file, [float(msg[1]), float(msg[3]), float(msg[5]), float(msg[7])])
            print(f'cnt: {cnt}')
            do[cnt] =  float(msg[3]) # DO
            t [cnt] = round(9/5 * float(msg[5]) + 32, 1)   # tempuerature
            p [cnt] = float(msg[7]) # pressure
            print(f'p[cnt]:{p[cnt]}')
            cnt = cnt + 1   
        elif (len(msg)==2) and (msg[0] == 'dstart'):
            print("starting sample")
            csv_file = init_file()
            cnt = 0
            
            size = int(msg[1])
            p = np.zeros(size)
            t = np.zeros(size)
            do = np.zeros(size)
            
        elif len(msg)==1:
            
            if msg[0] == "dfinish\n":
                print("sample done")
                sdata['sample_loc'] = csv_file
                generate_graph(csv_file)
                save_json()
                               
                #########################################################
                ################### Upload to database ####################
                try:                
                    # how to get initial DO and pressure?
                    #init_do = -1
                    #init_pressure = -1
                    # hard code these values
                    truck_id ='00' #hard code it for now
                    
                    # already divided by init_do on the payload side
                    # convert to percentage
                    avg_do_perc = 100*do[do > 0].mean()
                    print(f'avg_do_perc:{avg_do_perc}')
                    print(f'pond_id:{pond_id}')
                    print(f'lat:{lat}')
                    print(f'lng:{lng}')
                    print(f'init_do:{init_do}')
                    print(f'init_pressure:{init_pressure}')
                    data = {'do': avg_do_perc, 'init_do': init_do, 'init_pressure': init_pressure,
                            'lat': lat, 'lng': lng, 'pid': pond_id, 'pressure': [float(p.mean())], 'sid': truck_id,
                            'temp': [float(t.mean())],
                            'batt_v': batt_v, 'type': 'truck'}
                    db.reference('LH_Farm/pond_' + pond_id + '/' + message_time + '/').set(data)
                    fails['internet'] = 0
                except:
                    logger.warning("uploading data to firebase failed")
                    print("uploading data to firebase failed")
                    fails['internet'] += 1
                    # app = restart_firebase(app)
                ###########################################################################             
                # nowe reset the values and counters            
                cnt = 0
                #init_DO = -1
                #init_pressure = -1
                check_size = 1 # reset the check size flag to resume checking for next pond
                prev_sample_size = -1
                msg = 'sample reset' 
                ble_uart_write(msg)
                sleep(5)
            # this is return from the 'get' call            
            elif init_DO == -1:
                init_DO=float(msg[0])
                print(f'init_DO:{init_DO}')
                msg = 'get init_p'
                ble_uart_write(msg)
                sleep(1)
            else: # if initial DO has been updated, then get pressure
                init_pressure=float(msg[0])
                print(f'init_pressure:{init_pressure}')
                check_size = 1

