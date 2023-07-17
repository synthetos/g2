#!/usr/bin/env python3

import os
import sys
import argparse
import serial.tools.list_ports
import subprocess
import shutil


def flash_board(serial_port, g2core_bin_path):
    # Set the baud rate on the selected serial port
   

    # Flash the board using bossac
    cmd = f"bossac --port={serial_port} -u=true -e -w -v -i -b -R {g2core_bin_path}"
    proc = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

    # Print the output of the command to the console
    while True:
        line = proc.stdout.readline()
        if not line:
            break
        print(line.decode("utf-8").rstrip())

    # Wait for the command to complete
    proc.wait()
    if proc.returncode != 0:
        print_message("red", f"Error flashing g2core board. bossac command exited with status {proc.returncode}")
    else:
        print_message("green", "g2core board successfully flashed!")

# Define function to print colored text
def print_message(color, message):
    if len(color) == 0:
        print(message)
    elif color == "red":
        print("\033[0;31m" + message + "\033[0m")
    elif color == "green":
        print("\033[0;32m" + message + "\033[0m")

# Define function to prompt user to select a serial port
def select_serial_port():
    # List available serial ports
    serial_ports = [port.device for port in serial.tools.list_ports.comports()]

    if not serial_ports:
        print_message("red", "No serial ports found. Make sure your device is connected and try again.")
        sys.exit(1)

    while True:
        print_message("", "Select the serial port to use:")
        for i, port in enumerate(serial_ports):
            print(f"{i+1}. {port}")
        port_number = input("Enter the number of the serial port to use: ")

        # Check that the user entered a valid number
        if not port_number.isdigit() or not 1 <= int(port_number) <= len(serial_ports):
            print_message("red", f"Invalid port number: {port_number}")
        else:
            return serial_ports[int(port_number)-1]



def check_dependencies():
    # Check for pyserial module
    try:
        import serial
    except ImportError:
        print("pyserial module is not installed.")
        return False

    # Check for bossac command
    if shutil.which("bossac") is None:
        print("bossac command not found. Please make sure it is installed and in your PATH.")
        return False

    return True

def enter_bootloader(serial_port):
    # Set the baud rate on the selected serial port
    # This is not needed on ArduinoDUE boards
    # You do however need to be in the programming port  for flashing to work
    try:
        with serial.Serial(port=serial_port, baudrate=1200) as ser:
            ser.setDTR()
        with serial.Serial(port=serial_port, baudrate=9600, timeout=1) as ser:
            ser.flushInput()
    except serial.serialutil.SerialException as e:
        print(f"Error setting baud rate on port {serial_port}: {str(e)}")
        sys.exit(1)



# Define main function
def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Flash a g2core board.')
    parser.add_argument('--port', type=str, help='The serial port to use')
    parser.add_argument('bin', type=str, help='The path to the g2core binary file to flash')
    args = parser.parse_args()

    if not check_dependencies():
        sys.exit()

    # Select a serial port if the --port argument is not given
    if not args.port:
        args.port = select_serial_port()
        
    args.bin = " ../bin/EnderLaser-gquintic-g/g2core.bin "

    # Detect operating system and display appropriate installation message for pyserial
    if sys.platform.startswith("linux"):
        print('asdf')

        if "microsoft" in os.uname().release.lower():
            print('asdf')
            # WSL 2
            enter_bootloader(args.port)
            flash_board(args.port, args.bin)
            print('asdf')

    elif sys.platform.startswith("darwin"):
        pass

    else:
        print_message("red", "Unsupported operating system. Cannot install pyserial module.")
        sys.exit(1)

if __name__ == "__main__":
    main()