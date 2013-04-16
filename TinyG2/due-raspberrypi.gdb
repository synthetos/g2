# For Sam, RaspberryPi, etc.
set remote hardware-breakpoint-limit 6
set remote hardware-watchpoint-limit 4

#target extended-remote raspberrypi:3333
target extended-remote | ssh pi@raspberrypi -C 'sudo openocd -c "gdb_port pipe; log_output openocd.log" -s /usr/share/openocd/ -f board/raspberrypi-due.tcl'
