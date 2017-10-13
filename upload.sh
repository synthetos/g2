#!/bin/bash
cd ./g2core
make CONFIG=geratech

set -x

if [ ! -n "$PORT" ]; then
  PORT=$(python3 ../find_port.py --vendor Synthetos)
fi

if [ ! -n "$PORT" ]; then
  PORT=$(python3 ../find_port.py --vendor 03eb)
fi

if [ ! -n "$PORT" ]; then
  PORT=$(python3 ../find_port.py --vendor Arduino_LLC)
fi

if [ ! -n "$PORT" ]; then
  echo "No port found. Please enter a port"
  exit
fi

# Activate the bootloader
stty -F ${PORT} 1200 hup
sleep 1
stty -F ${PORT} 9600
sleep 3

# Program
bossac -e -w -v -i -b -R ./bin/geratech-geratech_proto/g2core.bin

cd ../
