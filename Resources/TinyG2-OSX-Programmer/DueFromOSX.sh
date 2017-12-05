#!/bin/bash
FLASH_TOOLS="arduino-flash-tools-master"
BOSSAC=$FLASH_TOOLS"/tools_darwin/bossac/bin/bossac"
OBJCOPY=$FLASH_TOOLS"/bins/arm-none-eabi-objcopy"


#downloading arduino build tools
if [ -d "arduino-flash-tools-master" ]; then
  # Control will enter here if $DIRECTORY exists.
  echo "We have the flashing tools.. moving on."
else
   echo "We do not have the flash tools.. Getting Them"
   wget https://github.com/synthetos/arduino-flash-tools/archive/master.zip && unzip -x master.zip && rm -f master.zip

fi

quit



function show_usage() {
	cat <<END
	USAGE: $0 -f TinyG.elf -p <native port>
    
	Replacing <native port> with the device name of the Arduino Due native port
	or TinyG v9 port, respctively.

	One of the following may be a valid choice:
	
END
	
	ls -1 /dev/tty.usbmodem* | xargs echo "          "
	
	cat <<END

	You may be able to use /dev/tty.usbmodem* (with the star) if there is only one.
END
}

file=""
port=""
native_port="true"

while getopts h?f:p:P: flag; do
	case $flag in
		f)
			file=$OPTARG
			;;
		p)
			port=$OPTARG
			;;
		P)
			port=$OPTARG
			native_port="false"
			;;
		[h])
			show_usage;
			exit 0
			;;
	esac
done

if [[ ${file} == "" ]]; then
	echo "Error: Please specify a file to program. The name will most likely be 'TinyG.elf'."
	exit 1;
fi

if [[ ${port} == "" ]]; then
	echo "Error: Please specify a port with -p or -P. Currently visible ports:"
	ls -1 /dev/tty.usbmodem* | xargs echo "    "
	exit 1;
fi


$OBJCOPY -O binary "${file}" "${file/.elf/.bin}" 

echo "Forcing reset using 1200bps open/close on port ${port}"
# perl -e 'open(my $fh, ">", "${port}"); close($fh);'
stty -f "${port}" 1200
echo "wait..."
# echo "" > "${port}"
# perl -e 'open(my $fh, ">", "${port}"); close($fh);'

sleep 0.5

# stty -f "${port}" 115200

echo "Starting programming of file ${file} -> ${file/.elf/.bin} on port ${port/\/dev\//}"
$BOSSAC -e -w -v -b "${file/.elf/.bin}"

echo
echo "WARNING: You may need to hit the RESET button on the device at this point."

##----------- PROGRAMMING port
#Forcing reset using 1200bps open/close on port /dev/tty.usbmodem26231
#bossac --port=tty.usbmodem26231 -U false -e -w -v -b $tmp/BareMinimum.cpp.bin -R 