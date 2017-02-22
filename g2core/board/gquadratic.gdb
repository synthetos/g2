# Open and connect to openocd with the ATMEL-ICE
#target remote | /usr/local/bin/openocd -c "set CHIPNAME ${CHIP}" -f ${MOTATE_PATH}/openocd.cfg -f ${MOTATE_PATH}/platform/atmel_sam/atmel_sam.cfg -c "gdb_port pipe; log_output openocd.log"

target extended-remote :2331

# Setup for non-wrapped lines and non-pages prints
set width 0
set height 0

# Turn on history saving
set history save on
set history expansion on

set print pretty on

define reset_deep
	monitor reset

	# Reset peripheral  (RSTC_CR)
	set *0x400E1800 = 0xA5000004

#	# Initializing PC and stack pointer
#   mon reg sp = (0x20400000)
#	set *0x20400004 = *0x20400004 & 0xFFFFFFFE
#	mon reg pc=(0x20400004)
	info reg
end

define flash
	make
	load
	reset
end

define dump_mb
	source ../Resources/debug/mb.gdb
end

define boot_from_flash
		#monitor atsamv gpnvm set 1
end

source -s arm.gdb
