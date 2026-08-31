wget https://github.com/joan2937/pigpio/archive/refs/tags/v79.tar.gz

# 3. Extract the tar file
tar zxf v79.tar.gz

# 4. Navigate into the folder, build, and install
cd pigpio-79
make
sudo make install
sudo ldconfig

# 1. Copy the systemd service template from the source folder to the system directory
sudo cp util/pigpiod.service /lib/systemd/system/

# 2. Reload systemd to recognize the newly added service file
sudo systemctl daemon-reload

# 3. Enable the service to start automatically on system boot
sudo systemctl enable pigpiod

# 4. Start the service immediately
sudo systemctl start pigpiod

# 5. Check the status to ensure it is actively running
sudo systemctl status pigpiod


[Unit]
Description=Pigpio daemon
After=network.target

[Service]
ExecStart=/usr/local/bin/pigpiod -g
Type=forking

[Install]
WantedBy=multi-user.target
