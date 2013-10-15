#!/bin/bash
arduinoAppDir=/Applications
PATH=$PATH:$arduinoAppDir/Arduino.app/Contents/Resources/Java/hardware/tools/g++_arm_none_eabi/bin/:$arduinoAppDir/Arduino.app/Contents/Resources/Java/hardware/tools/

function show_usage() {
	cat <<END
	USAGE: $0 -f TinyG.elf (-p <native port>|-P <programming port>)

	Replacing <native port> OR <programming port> with the device name of
	the Arduino Due native port or programming port, respctively.

	You can find the avilable port with the command:

	ls /dev/tty.usbmodem*

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

if [[ ${programming_port} == "true" ]]; then
	echo "PROGRAMMING PORT"
	exit 1;
fi

if [[ ${port} == "" ]]; then
	echo "Error: Please specify a port with -p or -P. Currently visible ports:"
	ls -1 /dev/tty.usbmodem* | xargs echo "    "
	exit 1;
fi


arm-none-eabi-objcopy -O binary "${file}" "${file/.elf/.bin}" 

echo "Forcing reset using 1200bps open/close on port ${port}"
stty -f "${port}" 1200
echo "wait..."
# echo "" > "${port}"
perl -e 'open(my $fh, ">", "${port}"); close($fh);'

sleep 0.1

stty -f "${port}" 2400


echo "Starting programming of file ${file} -> ${file/.elf/.bin} on port ${port/\/dev\//}"
$arduinoAppDir/Arduino.app/Contents/Resources/Java/hardware/tools/bossac --port=${port/\/dev\//} -U ${native_port} -e -w -v -b "${file/.elf/.bin}" -R 

#bossac --port=tty.usbmodem26231 -U true -e -w -v -b $tmp/BareMinimum.cpp.bin -R 


##----------- PROGRAMMING port
#Forcing reset using 1200bps open/close on port /dev/tty.usbmodem26231
#bossac --port=tty.usbmodem26231 -U false -e -w -v -b $tmp/BareMinimum.cpp.bin -R 