================================================================================
HAUCS DRONE PI - FLIGHT SCRIPT AUTOSTART SETUP (systemd)
Rev 090726.1
Applies to: Raspberry Pi companion computer, user "haucs"
Purpose:    Run main_rc8_uart_parm.py (or _latch.py) automatically at boot
================================================================================

--------------------------------------------------------------------------------
1. FILES INVOLVED
--------------------------------------------------------------------------------

  /home/haucs/Desktop/HAUCS_DRONE/startup.sh     launcher script
  /home/haucs/Desktop/HAUCS_DRONE/startup.log    script stdout/stderr
  /etc/systemd/system/winch.service              boot hook (systemd unit)

Note the unit is named winch.service, NOT haucs.service. When searching a
reconfigured Pi for an existing autostart hook, grep by path, not by name:

  grep -rn "startup.sh\|main_rc8\|HAUCS_DRONE" /etc/systemd/system/ 2>/dev/null


--------------------------------------------------------------------------------
2. startup.sh
--------------------------------------------------------------------------------

#!/bin/bash
source /home/haucs/mav/bin/activate
cd /home/haucs/Desktop/HAUCS_DRONE
python -u main_rc8_uart_parm.py >> /home/haucs/Desktop/HAUCS_DRONE/startup.log 2>&1

  - Swap main_rc8_uart_parm.py -> main_rc8_uart_latch.py on latch-drive builds.
  - -u (unbuffered) is required, otherwise startup.log stays empty until the
    process exits and you lose all diagnostics on a crash.
  - Make executable (not strictly required when launched via /bin/bash, but
    keeps the script usable standalone):

      chmod +x /home/haucs/Desktop/HAUCS_DRONE/startup.sh

  - If the file was ever edited on Windows, strip CRLF line endings or the
    shebang fails with a "bad interpreter: ^M" error:

      file /home/haucs/Desktop/HAUCS_DRONE/startup.sh     # look for "CRLF"
      sed -i 's/\r$//' /home/haucs/Desktop/HAUCS_DRONE/startup.sh


--------------------------------------------------------------------------------
3. /etc/systemd/system/winch.service
--------------------------------------------------------------------------------

[Unit]
Description=HAUCS winch flight script
After=multi-user.target

[Service]
User=haucs
WorkingDirectory=/home/haucs/Desktop/HAUCS_DRONE
ExecStart=/bin/bash /home/haucs/Desktop/HAUCS_DRONE/startup.sh
Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target

Field notes:
  User=haucs        Runs as the normal user, so the venv path and group
                    permissions match interactive testing. Group membership
                    must be correct in /etc/group - a service started at boot
                    inherits nothing from a login session.
  Restart=always    Script is restarted if it dies mid-mission. Also means a
                    script that crashes on startup will loop silently every
                    3 s - always check journalctl, not just "status".
  ExecStart         Launching via /bin/bash makes the shebang and exec bit
                    optional. "source" is a bash builtin, so /bin/sh will NOT
                    work here.


--------------------------------------------------------------------------------
4. INSTALL / ENABLE
--------------------------------------------------------------------------------

  sudo systemctl daemon-reload
  sudo systemctl enable --now winch.service
  systemctl status winch.service

"enable" creates the symlink in /etc/systemd/system/multi-user.target.wants/
that makes it start at boot. "--now" also starts it immediately.

Copying the unit file onto a new Pi is NOT sufficient - it must be enabled
separately. This is the most common cause of "it didn't start on the new Pi".


--------------------------------------------------------------------------------
5. VERIFY
--------------------------------------------------------------------------------

  systemctl is-enabled winch.service      expect: enabled
  systemctl is-active  winch.service      expect: active
  systemctl cat        winch.service      full unit + any drop-in overrides
  ls -l /etc/systemd/system/multi-user.target.wants/   symlink present?

  journalctl -u winch.service -b --no-pager     systemd's view, this boot
  tail -f /home/haucs/Desktop/HAUCS_DRONE/startup.log   the script's own output

Repeated "Started/Stopped" pairs every ~3 s in journalctl = restart loop;
the reason will be in startup.log.


--------------------------------------------------------------------------------
6. DAY-TO-DAY CONTROL
--------------------------------------------------------------------------------

  sudo systemctl stop    winch.service    stop (e.g. to run the script by hand)
  sudo systemctl start   winch.service
  sudo systemctl restart winch.service    after editing the flight script
  sudo systemctl disable winch.service    stop it running at boot

After editing winch.service itself: sudo systemctl daemon-reload, then restart.
After editing the Python flight script: restart only, no daemon-reload needed.


--------------------------------------------------------------------------------
7. RECONFIGURED / REIMAGED PI CHECKLIST
--------------------------------------------------------------------------------

These are the items that break on a fresh image even when the unit file and
startup.sh are copied over correctly.

  a) venv exists at the expected path
       ls /home/haucs/mav/bin/activate
       /home/haucs/mav/bin/python -c "import pymavlink, board, busio"

  b) group membership - needed for UART and I2C, requires reboot to take effect
       groups haucs                    expect dialout and i2c
       sudo usermod -aG dialout,i2c haucs

  c) serial console disabled, serial port hardware enabled
       raspi-config -> Interface Options -> Serial Port
         login shell over serial: NO
         serial port hardware:    YES
       An enabled login console holds the UART and the Cube link will not open.

  d) I2C enabled (ADS1115 Hall ADC, INA226)
       raspi-config -> Interface Options -> I2C -> YES
       ls /dev/i2c-1
       i2cdetect -y 1

  e) device nodes present
       ls -l /dev/serial0 /dev/ttyAMA0

  f) timezone - Pi is intentionally configured as Eastern (UTC-4) even for the
     Illinois sites. This is deliberate, not a bug. Do not "correct" it.


--------------------------------------------------------------------------------
8. ALTERNATIVE HOOKS (for reference only - not used here)
--------------------------------------------------------------------------------

If an older Pi in the fleet does not have winch.service, check these. Both
have failure modes on newer Raspberry Pi OS releases:

  /etc/rc.local
      Not shipped on Bookworm. rc-local.service exists but does nothing unless
      the file is present AND executable, so a long-working script can go
      silent after a reimage. If used, run the command with a trailing "&" -
      rc.local blocks boot until it returns, and the flight script never does.

  ~/.config/lxsession/LXDE-pi/autostart
      Ignored on Bookworm (default desktop is labwc/wayfire, not LXDE). The
      equivalents are ~/.config/autostart/*.desktop or an [autostart] entry in
      ~/.config/wayfire.ini. All desktop autostart methods require boot-to-
      desktop with autologin - they never fire on a console/headless boot.

  Check which applies before assuming:
      cat /etc/os-release | head -2
      systemctl get-default        graphical.target vs multi-user.target

systemd is preferred for all fleet units: it survives OS upgrades, restarts on
failure, gives per-unit boot logs via journalctl, and supports ordering
constraints via After= - which matters because the flight script claims the
UART and I2C bus early in boot.

================================================================================
