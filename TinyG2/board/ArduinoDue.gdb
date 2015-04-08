# Setup for non-wrapped lines and non-pages prints
set width 0
set height 0

# Open and connect to openocd with the ATMEL-ICE
target remote | /usr/local/bin/openocd -c "set CHIPNAME ${CHIP}" -f ${MOTATE_PATH}/openocd.cfg -f ${MOTATE_PATH}/platform/atmel_sam/atmel_sam.cfg -c "gdb_port pipe; log_output openocd.log"

# Turn on history saving
set history save on
set history expansion on

set print pretty on

# Halt the device
monitor reset halt
