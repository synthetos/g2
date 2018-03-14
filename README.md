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

This will build the `G2v9i` version of firmware.

## Further Reading

* [g2 wiki](https://github.com/bantamtools/g2/wiki)
* [Bantam: Intro to V2 Firmware](https://sites.google.com/a/othermachine.co/software/dev-instructions-and-resources/intro-to-firmware?pli=1)
* [Bantam: Dev Instructions & Resources](https://sites.google.com/a/othermachine.co/software/dev-instructions-and-resources/)
