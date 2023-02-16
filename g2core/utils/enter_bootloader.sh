#!/bin/bash

# Define function to print colored text
print_message () {
  if [ $# -eq 2 ]; then
    color=$1
    message=$2
  else
    color="default"
    message=$1
  fi

  case $color in
    "red")
      echo -e "\033[0;31m$message\033[0m"
      ;;
    "green")
      echo -e "\033[0;32m$message\033[0m"
      ;;
    *)
      echo $message
      ;;
  esac
}

# Print message to indicate entering bootloader mode
print_message "green" "Entering G2Core into bootloader mode..."

# List available serial ports and prompt user to select a port
serial_ports=($(ls /dev/ttyACM*))
if [ ${#serial_ports[@]} -eq 0 ]; then
  print_message "red" "No serial ports found. Make sure your device is connected and try again."
  exit 1
fi

while true; do
  print_message "default" "Select the serial port to use:"
  for i in "${!serial_ports[@]}"; do
    echo "$((i+1)). ${serial_ports[$i]}"
  done
  echo -n "Enter the number of the serial port to use: "
  read port_number

  # Check that the user entered a valid number
  if [[ ! "$port_number" =~ ^[0-9]+$ ]]; then
    print_message "red" "Invalid port number: $port_number"
  elif [ $port_number -lt 1 ] || [ $port_number -gt ${#serial_ports[@]} ]; then
    print_message "red" "Invalid port number: $port_number"
  else
    break
  fi
done

# Construct the serial port path from the selected number
serial_port=${serial_ports[$((port_number-1))]}

# Set the baud rate on the selected serial port
stty -F "$serial_port" 1200 hup
stty -F "$serial_port" 9600

# Print message to flash the board
print_message "green" "G2core on port $serial_port should now be in bootloader mode. Execute this command:"
print_message "green" "bossac --port=$serial_port -u=true -e -w -v -i -b -R <path-to-board>/g2core.bin"
print_message "green" "to flash your g2core board."
