from adafruit_ble import BLERadio
from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
from adafruit_ble.services.nordic import UARTService
import board, adafruit_gps

import random, csv
import matplotlib
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

import tkinter as tk
from PIL import Image, ImageTk, ImageDraw
import time
import threading
import hashlib

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

if os.environ.get('DISPLAY','') == '':
    print('no display found. Using :0.0')
    os.environ.__setitem__('DISPLAY', ':0.0')

# Globals
image_lock = threading.Lock()
stop_event = threading.Event()
latest_hash = ""  # Shared hash value
thread = None
image_path = "/home/haucs/Desktop/HAUCS/truck_app/data/generated_image.png"

#Handle Inputs
if len(sys.argv) > 1:
    timer_only = sys.argv[1]
else:
    timer_only = "false"

ble = BLERadio()
uart_connection = None

### INIT JSON FILE
sdata = {}

sensor_file = folder + "sensor.json"
header = ['time', 'do', 'temperature', 'pressure']

pond_history = np.array(["unknown"])

fails = {'gps':0, 'batt':0, 'internet':0, }

cred = credentials.Certificate("/home/haucs/Desktop/HAUCS/fb_key.json")
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
    try:
        global header
        filePath = "/home/haucs/Desktop/HAUCS/truck_app/data/"
        #filePath = "data/"
        date = datetime.now().strftime('%Y-%m-%d-%H-%M-%S')

        if not os.path.exists(filePath):
            os.mkdir(filePath)

        csvFile = filePath + date + ".csv"

        with open(csvFile,'w',newline='') as csvfile:
          writer = csv.writer(csvfile, delimiter=',')
          writer.writerow(header)
    except e:
        print("Failed to create csv file {csvfile}", str(e))

    return csvFile

def exp_func(x, a, b, c):
    return a * np.exp(-b * x) + c

# def generate_graph(infile,gui_image):
#     df = pd.read_csv(infile)
#     df['s'] = df['time'] / 1000
# 
#     x = np.arange(100) * 3 / 10
# 
#     try:
#         popt, pcov = curve_fit(exp_func, df['s'], df['do'])
#         y = exp_func(x, *popt)
#     except:
#         y = x
#         logger.warning("curve fitting failed")
#     
#     # plt.style.use('dark_background')
#     with plt.rc_context({'axes.edgecolor':'red', 'xtick.color':'red', 'xtick.labelsize':25, 'ytick.labelsize':25, 'ytick.color':'red', 'figure.facecolor':'black'}):
#         plt.figure()
#         plt.scatter(df['s'], df['do'], color="red", linewidth=4, alpha=1)
#         plt.plot(x, y, color="red", linewidth=4, alpha=0.5)
#         plt.xlabel("seconds", color="red", fontsize=25)
#         plt.ylabel("% Saturation",  color="red", fontsize=25)
#         plt.annotate(str(round(y[-1])) + '%', (x[-1], y[-1]), xytext=(x[70], y[15]), arrowprops={"width":1, "color":"red", "headwidth":6},color="red", fontsize=25)
#         plt.savefig(infile[:-4] + ".png", bbox_inches="tight")
#         # also save to an image used for GUI
#         plt.savefig(gui_image, bbox_inches="tight")

# Thread safe version... ##############
matplotlib.use('agg')
def generate_graph(infile, gui_image):
    try:
        df = pd.read_csv(infile)
        df['s'] = df['time'] / 1000
        x = np.arange(100) * 3 / 10  # 0 to 30 sec

        try:
            popt, pcov = curve_fit(exp_func, df['s'], df['do'])
            y = exp_func(x, *popt)
        except e:
            logger.warning("Curve fitting failed: %s", str(e))
            y = x  # fallback: linear

        # Create the figure
        with plt.rc_context({
            'axes.edgecolor': 'red',
            'xtick.color': 'red',
            'ytick.color': 'red',
            'xtick.labelsize': 25,
            'ytick.labelsize': 25,
            'figure.facecolor': 'black'
        }):
            plt.figure()
            plt.scatter(df['s'], df['do'], color="red", linewidth=4, alpha=1)
            plt.plot(x, y, color="red", linewidth=4, alpha=0.5)
            plt.xlabel("seconds", color="red", fontsize=25)
            plt.ylabel("% Saturation", color="red", fontsize=25)
            plt.annotate(
                str(round(y[-1])) + '%',
                (x[-1], y[-1]),
                xytext=(x[70], y[15]),
                arrowprops={"width": 1, "color": "red", "headwidth": 6},
                color="red", fontsize=25
            )
            # Save to two files
            plt.savefig(infile[:-4] + ".png", bbox_inches="tight")
            plt.savefig(gui_image, bbox_inches="tight")
            plt.close()

        logger.info("Graph generated and saved to %s", gui_image)
        
    except e:
        logger.error("Failed to generate graph: %s", str(e))

def save_json():
    global sdata
    with open(sensor_file, 'w') as outfile:
        json.dump(sdata, outfile)
        
def ble_connect():
    global uart_connection
    print("searching for sensor ...")
    log.info("searching for sensor ...")

    for adv in ble.start_scan(ProvideServicesAdvertisement):
        print('ble:'+adv.services)
        log.info('ble'+adv.services) 
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
    global sdata
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
    # hypothesis: the sequence of pond_id can also be 
    # used to detect if a pond is detected incorrectly
    ##############################################################
    #ps = pd.read_csv('pond_sequence.csv')
    #sequence_ids = ps.pop('sequence_id')
    #pd_hist = pd.Series(pond_history)
    #cid = find_subsequence_index(sequence_ids, pd_hist)
    # cid is the end of the existing sequence
    # the current pond id should be the next in the sequence
    #pond_id_from_sequence = sequence_ids[cid+1]
    
    #update pond_historys - FIFO - keeping 3 entries to check 
    #against the pond sequence table.
    if pond_id == "unknown":
        pass # do not append
    elif len(pond_history) < 2:
        pond_history = np.append(pond_history, [pond_id])
    else:
        pond_history = np.append(pond_history[1:2], [pond_id])
    sdata['pond_id'] = pond_id
    save_json()
    #if common_pond_id != pond_id:
    #logger.info(f"detected in {pond_id}, most common pond {common_pond_id}, pond_history {pond_history}")
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

def has_image_changed(image_path, prev_hash):
    """Returns True if file hash has changed since last check."""
    try:
        with open(image_path, "rb") as f:
            new_hash = hashlib.md5(f.read()).hexdigest()
        return new_hash != prev_hash, new_hash
    except FileNotFoundError:
        return True, prev_hash

def setup_gui():
    global latest_hash
    global sdata
#    cinit_DO=-1
    """Sets up the GUI with the dynamically updated image."""
    root = tk.Tk()
    root.title("DO Truck GUI")
    root.geometry("1280x720")

    # Layout grid config
    root.grid_rowconfigure(1, weight=1)
    root.grid_columnconfigure((0, 1, 2), weight=1)

#     # --- Top Labels ---
#     for col, title in enumerate(["Title 1", "Title 2", "Title 3"]):
#         label = tk.Label(root, text=title, font=("Arial", 18), bg="white")
#         label.grid(row=0, column=col, pady=10)

    # --- Image Display ---
    img = Image.open(image_path)
    img = img.resize((800,600))
    tk_img = ImageTk.PhotoImage(img)
    image_label = tk.Label(root, image=tk_img, bg="white")
    image_label.image = tk_img
    image_label.grid(row=0, column=0, columnspan=3, pady=10)

    prev_gui_hash = latest_hash  # GUI-side hash
    def refresh_image():
        nonlocal prev_gui_hash
        with image_lock:
            changed, new_hash = has_image_changed(image_path, prev_gui_hash)
            if changed:
                try:
                    new_img = Image.open(image_path).resize((800,600))
                    new_tk_img = ImageTk.PhotoImage(new_img)
                    image_label.config(image=new_tk_img)
                    image_label.image = new_tk_img
                    prev_gui_hash = new_hash
#                     cinit_DO=sdata['init_DO']
                    print("[GUI] Image updated.")
                except Exception as e:
                    print("[Error] Failed to update image:", e)
        root.after(1000, refresh_image)
    # --- Bottom Controls (Buttons) ---
    #button_frame = tk.Frame(root, bg="white")
    #button_frame.grid(row=1, column=0, columnspan=3, pady=(10, 5))

    #cal_do_button = tk.Button(button_frame, text="Calibrate DO", font=("Arial", 16), command=do_calibration, width=20)
    #cal_press_button = tk.Button(button_frame, text="Calibrate pressure", font=("Arial", 16), command=stop_image_generation, width=20)
    #cal_do_button.pack(side=tk.LEFT, padx=10)

    #cal_label = tk.Label(button_frame, text=f'{cinit_DO}', font=("Arial", 18), bg="white")
    #cal_label.pack(side=tk.LEFT, padx=20)

    #submit_button = tk.Button(button_frame, text="Update Pond ID", font=("Arial", 14),
    #                      command=lambda: handle_text_entry(text_entry))
    #submit_button.pack(side=tk.LEFT)

    #text_entry = tk.Entry(button_frame, font=("Arial", 14), width=10)
    #text_entry.pack(side=tk.LEFT, padx=10)

    refresh_image()
    root.mainloop()

# 
# def do_calibration():
#     msg = 'calibrate do'
#     ble_uart_write(msg)
#     sleep(2)
#     msg = 'get init_do'
#     ble_uart_write(msg)
#     sleep(2)
#     #print(f"[Input] Text field: {value}")
    
def generate_image():
    global sdata
    """Continuously generates and updates the image if it has new content."""
    counter = 0
    global i2c, gps
    global latest_hash
    ######################## Initialization ###############################

    cnt = 0
    global sdata
    init_DO = -1
    init_pressure = -1
    prev_pond_id = -1
    prev_sample_size = -1

    init_connect = -1 #set a flag to indicate first time connect to the payload
    check_size = -1 # check the dsize

    sleep(2)
    lng = gps.longitude
    lat = gps.latitude
    print(f"lng is {lng}, lat is {lat}")
    logger.info(f"lng is {lng}, lat is {lat}")
    msg = 'sample reset' 
    ble_uart_write(msg)
    sleep(5)

    batt_cnt = 0
    msg = 'batt'
    ble_uart_write(msg)
    sleep(5)
    msg = 'get init_do'
    ble_uart_write(msg)
    sleep(5)
    
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
                check_size = 1
            # monitor the sample size - if it is >0, then samples have been acquired
            if check_size == 1:                                 
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
            ### read incoming messages
            msg = ble_uart_read()
            if len(msg) > 0:
                print(msg)
                msg = msg.split(",")
                
            # This logic is used to monitor the sampling activity since auto sening is being engaged
            # the sample size will increase automatically 
            print(f'msglen:{len(msg)}: {msg}')
            if (len(msg)==2) and (msg[0].startswith("b'd")): #b'dsize
                cstr = msg[1]
                slen = len(cstr)
                print(f'strlen: {slen}')
                cstrnum = cstr[0:slen-3]
                current_sample_size = float(cstrnum)                
                print(f"current sample size {current_sample_size}")
                print(f"prev_sample_size: {prev_sample_size}")
                # sampling started, let's lock the current time
                if prev_sample_size<=0 and current_sample_size>0: 
                    prev_sample_size = current_sample_size
                    # noinspection PyBroadException
                    try:
                        # we locking lat/lon now
                        update_GPS(1)
                        sleep(1)
                        lng = gps.longitude
                        lat = gps.latitude
                        if not lng: lng = 0
                        if not lat: lat = 0
                        sdata['lng'] = lng
                        sdata['lat'] = lat                        
                        msg = 'batt'
                        ble_uart_write(msg)
                        # locking the pond id
                        pond_id = get_pond_id(lat, lng)
                        sleep(1)
                    except:
                        logger.warning("getting gps lat/lng failed")                        
                    # locking the time as well.
                    message_time = time.strftime('%Y%m%d_%H:%M:%S', time.gmtime()) #GMT time
                    sdata['message_time'] = message_time
                    save_json() 
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
                        
                        print("starting sample")
                        csv_file = init_file()
                        cnt = 0               
                        size = int(current_sample_size) #int(msg[1])
                        p = np.zeros(size)
                        t = np.zeros(size)
                        do = np.zeros(size)  
                               
            elif (len(msg) == 6) and (msg[0] == 'd'): #return from "single"
                # msg = f"d,{do[i]},t,{temperature[i]},p,{pressure[i]}\n"
                sdata['do'] = float(msg[1])
                sdata['temperature'] = round(9/5 * float(msg[3]) + 32, 1)
                sdata['pressure'] = float(msg[5])
                save_json()
                writeCSV(csv_file, [cnt, msg[1], msg[3], msg[5]])
            elif (len(msg) == 4) and (msg[0] == 'v'):
                sdata['battv'] = float(msg[1])
                sdata['batt_status'] = msg[3][:-1]
                save_json()
                batt_v = float(msg[1]) #db variable
                print(f'batt_v:{batt_v}')
            elif (len(msg) == 8) and (msg[0] == 'ts'): # sample print
                print(f'cnt: {cnt} appending: {msg}')
                writeCSV(csv_file, [float(msg[1]), float(msg[3]), float(msg[5]), float(msg[7])])
                do[cnt] =  float(msg[3]) # DO
                t [cnt] = round(9/5 * float(msg[5]) + 32, 1)   # tempuerature
                p [cnt] = float(msg[7]) # pressure
                print(f'p[cnt]:{p[cnt]}')
                cnt = cnt + 1   
            elif (len(msg)==2) and msg[0].startswith('ds'):#dstart
                print("starting sample")
                csv_file = init_file()
                cnt = 0               
                size = int(msg[1])
                p = np.zeros(size)
                t = np.zeros(size)
                do = np.zeros(size)               
            elif len(msg)==1:              
                if msg[0].startswith("df"):#dfinish
                    print("sample done")
                    #sdata['sample_loc'] = csv_file
                    save_json()
                    with image_lock:
                        print("got image_lock")
                        generate_graph(csv_file,image_path)
                        with open(image_path, "rb") as f:
                            latest_hash = hashlib.md5(f.read()).hexdigest()
                    ################### Upload to database ####################
                    try:                
                        truck_id =sdata['name'] #hard code it for now
                        print(f'len of do: {len(do)}')
                        avg_do_perc = 100*do[do > 0].mean()
                        print(f'pond_id:{pond_id} lat:{lat} lng:{lng} avg_do_perc:{avg_do_perc} init_do:{init_DO} init_pressure:{init_pressure}')
                        data = {'do': avg_do_perc, 'init_do': init_DO, 'init_pressure': init_pressure,
                                'lat': lat, 'lng': lng, 'pid': pond_id, 'pressure': [float(p.mean())], 'sid': truck_id,
                                'temp': [float(t.mean())],
                                'batt_v': batt_v, 'type': 'truck'}
                        db.reference('LH_Farm/pond_' + pond_id + '/' + message_time + '/').set(data)
                        fails['internet'] = 0
                    except:
                        logger.warning("uploading data to firebase failed")
                        print("uploading data to firebase failed")
                        fails['internet'] += 1
                        app = restart_firebase(app)             
                    ######## reset the values and counters    #################################        
                    cnt = 0
                    check_size = 1 # reset the check size flag to resume checking for next pond
                    prev_sample_size = -1
                    msg = 'sample reset' 
                    ble_uart_write(msg)
                    sleep(5)
                # this is return from the 'get' call            
                elif init_DO == -1:
                    init_DO=float(msg[0])
                    sdata['init_DO'] = init_DO
                    save_json() 
                    print(f'init_DO:{init_DO}')
                    msg = 'get init_p'
                    ble_uart_write(msg)
                    sleep(1)
                else: # if initial DO has been updated, then get pressure
                    init_pressure=float(msg[0])
                    sdata['init_pressure'] = init_pressure
                    save_json()
                    print(f'init_pressure:{init_pressure}')
                    check_size = 1
        # Check for changes
        changed, new_hash = has_image_changed(image_path, latest_hash)
        if changed:
            print(f"New image generated: Frame {counter}")
            prev_hash = new_hash  # Update hash
        counter += 1
        time.sleep(1)

def main():
    global latest_hash, thread, image_path
    #global init_DO
    #init_DO = -1
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
    global i2c, gps
    i2c = board.I2C()
    gps = adafruit_gps.GPS_GtopI2C(i2c)
    gps.send_command(b'PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
    gps.send_command(b"PMTK220,8000")
    update_GPS(5)

    # Ensure the image file exists
    if not os.path.exists(image_path):
        with Image.new("RGB", (800, 600), "white") as img:
            img.save(image_path)
            
    _, latest_hash_val = has_image_changed(image_path, "")
    latest_hash = latest_hash_val

    stop_event.clear()
    thread = threading.Thread(target=generate_image, daemon=True)
    thread.start()
    print("[Main] DO sensing process started at launch.")
    setup_gui()

if __name__ == "__main__":
    main()

