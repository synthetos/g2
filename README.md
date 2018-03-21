# TinyG2

This is the Other Machine Co production fork of Synthetos's g2 firmware.

## Toolchain

This project requires gcc 4.8.

You can install the toolchain using any of the following commands:

```
make
make toolchain
cd Tools && make
```

## Compiling Firmware

The firmware is contained in the `TinyG2/` directory.

When compiling the TinyG firmware, you need to specify a platform:

```
make PLATFORM=Othermill
make PLATFORM=OthermillPro
make PLATFORM=G2v9i
```

You must build the firmware from the `TinyG2/` directory:

```
cd TinyG2 && make PLATFORM=OthermillPro
cd TinyG2 && make PLATFORM=G2v9i
```

Alternatively, you can use the top-level Makefile shim:

```
make fw
```

This command build the `G2v9i` version of firmware, which is targeted for boards in V2 machines.

### Firmware Binaries

Firmware binaries can be found in the `TinyG2/bin` folder. Output is organized by `PLATFORM`.

The build puts both an ELF and a binary. ELF is the executable linker format and has symbols for debugging. The .bin is the binary image that we supply to BOSSA.

### Clean Build

To clean, run `make clean` at the top-level. By default, this will run a clean build for the `G2v9i` platform.

If you need to clean another platform, specify it as you would during compilation:

```
cd TinyG2 && make PLATFORM=OthermillPro clean
cd TinyG2 && make PLATFORM=G2v9i clean
```

## Flashing the TinyG2

When in SAM-BA mode, we update firmware with the open source utility BOSSA (the SAM-BA version of avrdude).  Our branch of it is [here](https://github.com/omco/bossa/tree/arduino), though the open-source version can be used for standalone flashing.

To flash the program using BOSSA, use the following command template:

```
bossac -p tty.usbserialXX -e -w -v -b -R path/to/firmware.bin
```

Example:

```
bossac -p tty.usbmodem1411 -e -w -v -b -R g2/TinyG2/bin/G2v9i/G2v9i.bin
```

* `-p` specifies the serial port.  It is likely to be `/dev/tty.usbserialXX`.
	* Important: `-p` prepends "`/dev/`", so if you supply the `/dev/`, it won't find the serial port.
* `-e` is erase
* `-w` is write,
* `-v` is verify (aka read back and check that it's valid).
* `-b` sets the machine to boot into firmware next time it reboots
* `-R` tells it to reboot.
* The last argument is the filename of the firmware, in .bin format.

## Debugging

You can debug the TinyG through JTAG using a SAMA-ICE adapter and the JLink tools.

### Installing JLink Tools

You will need to download and install the JLink software package [from here](https://www.segger.com/downloads/jlink/). This will provide `JLinkGDBServer`, which we use for talking to the SAMA-ICE adapter.

### Starting the JLinkGDBServer

You can start the JLinkGDBServer from the command line:

```
JLinkGDBServer -USB -device ATSAM3X8C -endian little -if JTAG -speed auto -noir
```

### Connecting to the Device

Once the server is running, you can connect to the device using `arm-none-eabi-gdb`.

In the `TinyG/` directory, run `arm-none-eabi-gdb`. `gdb` will mention that it connected to JLinkGDBServer.

We recommend running `arm-none-eabi-gdb` inside of the `TinyG/` folder so that the `.gdbinit` file is detected.

### Make Debug Command

You can build the code and open a debug terminal by adding `debug` to the build command:

```
make PLATFORM=OthermillPro debug
```

This will automatically connect to `gdb`. The `JLinkGDBServer` needs to be running.

## Further Reading

* [g2 wiki](https://github.com/bantamtools/g2/wiki)
* [Bantam: Intro to V2 Firmware](https://sites.google.com/a/othermachine.co/software/dev-instructions-and-resources/intro-to-firmware?pli=1)
* [Bantam: Dev Instructions & Resources](https://sites.google.com/a/othermachine.co/software/dev-instructions-and-resources/)
