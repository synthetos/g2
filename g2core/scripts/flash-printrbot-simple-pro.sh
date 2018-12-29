#! /bin/bash

FWFILE=$1

if [ "${FWFILE}" == "" ]; then
    FWFILE=./bin/PrintrbotSimple1608-printrboardG2v3/g2core.bin
fi

if [ ! -e ${FWFILE} ]; then
    echo "Error: Firmware file is not exist"
    exit 1
fi

BOSSAC=`which bossac`
if [ "${BOSSAC}" == "" ]; then
    echo "Error: bossac is not installed. Install bossa-cli package first."
    exit 1
fi

set -x
PORT=ttyACM0

stty -F /dev/ttyACM0 1200 hup
sleep 1

stty -F /dev/ttyACM0 9600
sleep 3

${BOSSAC}  --port=ttyACM0  -U true -e -w -v -i -b -R ${FWFILE}

echo ""

if [ "$?" != "0" ]; then
    echo "Error: Flashing firmware has been failed. Reset the printer and retry firmware flashing."
    exit 1
fi

echo "Flashing firmware has been accomplished."
echo "Reset the printer manually."

exit 0

