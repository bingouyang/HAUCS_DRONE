#!/bin/bash
source /home/haucs/mav/bin/activate
cd /home/haucs/Desktop/HAUCS_DRONE
python -u main_rc8_uart_direct.py >> /home/haucs/Desktop/HAUCS_DRONE/startup.log 2>&1
