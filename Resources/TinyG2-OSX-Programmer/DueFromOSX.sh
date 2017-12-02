#!/bin/bash
arduinoAppDir=/Applications
PATH=$PATH:$arduinoAppDir/Arduino.app/Contents/Resources/Java/hardware/tools/g++_arm_none_eabi/bin/:$arduinoAppDir/Arduino.app/Contents/Resources/Java/hardware/tools/

function show_usage() {
	cat <<END
	USAGE: $0 -f TinyG2.elf -p <native port>
    
	Replacing <native port> with the device name of the Arduino Due native port
	or TinyG v9 port, respectively.

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
	echo "Error: Please specify a file to program. The name will most likely be 'TinyG2.elf'."
	echo
	show_usage
	exit 1;
fi

if [[ ${port} == "" ]]; then
	echo "Error: Please specify a port with -p or -P. Currently visible ports:"
	ls -1 /dev/tty.usbmodem* | xargs echo "    "
	exit 1;
fi

# Check for bossac in the Arduino IDE 1.6+ locations
bossac_binary=`find $HOME/Library/Arduino15/packages -type f -name bossac 2>/dev/null`
if [[ ${bossac_binary}x == "x" ]]; then
	# Wasn't found, so lets check in the Arduino IDE <1.6 location
	bossac_binary=`find /Applications/Arduino.app/Contents/Resources/Java/hardware/tools -type f -name bossac 2>/dev/null`
	if [[ ${bossac_binary}x == "x" ]]; then
		echo "Error: bossac could not be found.  Is the Arduino IDE installed, along with the Arduino SAM Boards extension?"
		exit 1;
	fi
fi

# Check for objcopy in the Arduino IDE 1.6+ locations
objcopy_binary=`find $HOME/Library/Arduino15/packages -type f -name arm-none-eabi-objcopy 2>/dev/null`
if [[ ${objcopy_binary}x == "x" ]]; then
	# Wasn't found, so lets check in the Arduino IDE <1.6 location
	objcopy_binary=`find /Applications/Arduino.app/Contents/Resources/Java/hardware/tools -type f -name arm-none-eabi-objcopy 2>/dev/null`
	if [[ ${objcopy_binary}x == "x" ]]; then
		echo "Error: arm-none-eabi-objcopy could not be found.  Is the Arduino IDE installed, along with the Arduino SAM Boards extension??"
		exit 1;
	fi
fi

$objcopy_binary -O binary "${file}" "${file/.elf/.converted}"

echo "Forcing reset using 1200bps open/close on port ${port}"
# perl -e 'open(my $fh, ">", "${port}"); close($fh);'
stty -f "${port}" 1200
echo "wait..."
# echo "" > "${port}"
# perl -e 'open(my $fh, ">", "${port}"); close($fh);'

sleep 0.5

# stty -f "${port}" 115200

echo "Starting programming of file ${file} -> ${file/.elf/.converted} on port ${port/\/dev\//}"
$bossac_binary  -e -w -v -b "${file/.elf/.converted}"

echo
echo "WARNING: You may need to hit the RESET button on the device at this point."

##----------- PROGRAMMING port
#Forcing reset using 1200bps open/close on port /dev/tty.usbmodem26231
#bossac --port=tty.usbmodem26231 -U false -e -w -v -b $tmp/BareMinimum.cpp.bin -R
