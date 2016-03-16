# Setup for non-wrapped lines and non-pages prints
set width 0
set height 0

# Connect to the remote device
# echo \nConnecting to JLinkGDBServer on port 2331.\n
# echo (You mush have JLinkGDBServer running.)\n\n
# target remote localhost:2331

# Configure JLinkGDBServer
# monitor flash device = ATSAM3X8C
# monitor flash device = ATSAM3X8E
# monitor speed auto
# monitor endian little

# Open and connect to openocd with the ATMEL-ICE
target remote | /usr/local/bin/openocd -c "set CHIPNAME ${CHIP}" -f openocd.cfg -f platform/atmel_sam.cfg -c "gdb_port pipe; log_output openocd.log"

# Turn on history saving
set history save on
set history expansion on

set print pretty on

define boot_from_flash
  monitor at91sam3 gpnvm set 1
end

define reset
  boot_from_flash
  monitor reset init
end

define flash
  make
  load
  reset
end

reset
