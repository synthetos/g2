# TinyG 2

This branch (`edge`) is for the adventurous. It is not guaranteed to be stable. It's not guaranteed AT ALL.

# Build Instructions

### Prerequisites

* You must have at least a valid POSIX-style shell environment with building utilities such as `make`, `mkdir`, and `bash`.
	* On OS X this easily achieved by installing the XCode Command-Line Tools. The easiest method of installing the command-line tools is to run `xcode-select --install` from the Terminal. Full official instructions for installing them are [here](https://developer.apple.com/library/ios/technotes/tn2339/_index.html).
	* On Linux you need to ensure that you have `make` installed, or use the package manager for you're flaovr of linux to obtain it. Something like this should work:
	`sudo apt-get install git-core make`
	* Command-line compiling on Windows is not currently supported. It's probably not difficult, we just don't have the instructions in place yet. (We're more than happy to accept pull requests! Thank you!) Please see [this wiki page](https://github.com/synthetos/g2/wiki/Compiling-G2-on-Windows-(Atmel-Studio-6.2)) for instructions on building using Atmel Studio 6.2.
* You need to have this repo cloned via git or downloaded from GitHub as a zip.

## Compiling

First we need to decide what values we need for the following variables:

* `PLATFORM` - To build TinyG2 to run on a Due and using the gShield pinout, you want the `PLATFORM` to be `gShield`. Luckily, that's the default. For other boards (most likely experimental or custom) please consult the `Makefile` for available options.
* `SETTINGS_FILE` - You can use the default settings found in `settings/settings_default.h`, or you can create a new file in the settings directory, preferrably by copying one of the existing settings files, and then specify the name of the file (**not** including `settings/`) in the `SETTINGS_FILE` variable. The contents of this file effects the default parameters when the hardware is powered up or rest, since there isn't an EEPROM on some of these boards to store settings to. Almost all of these values can all be overriden from the serial interface, however.

To specify one of the above variables, either set that variable name in the environment, or pass in the make command line:

```bash
# In the environment:
export PLATFORM=gShield
export SETTINGS_FILE=settings_default.h
make

# Or, on the make command line directly:
make PLATFORM=gShield SETTINGS_FILE=settings_default.h

#You can mix and match the above. The command line will trump the environment.
export PLATFORM=gShield
make SETTINGS_FILE=settings_default.h
```

The first time you call make, it will attempt to download and decompress the correct build tools (currently we only use the ARM gcc found [here](https://launchpad.net/gcc-arm-embedded)) into the Tools subdirectory, which may take a few minutes depending on your connection speed. (Note: This is the part we haven't complete for Windows currently.) Once that has been completed, it won't need to happen again.

Each platform has it's own build subdirectory, and the resulting files can be found in `bin/${PLATFORM}`.

## Debugging and loading

In order to debug you need a debugger capable of debugging the chip you use. For the Due and V9 we use either a Atmel SAM-ICE (which is a OEM Segger J-Link locked to Atmel ARMs) or the cheaper and more recently released Atmel ICE. Either can be found at Mouser.com.

You choose which debug adapter you wish to use by *copying* the `openocd.cfg.example` file to `openocd.cfg` and then editing the `openocd.cfg` file. Simply follow the instructions in that file. Hint: It's just uncommenting the correct lines.

Once that is configured, you'll need to make sure the debugger is correctly connected to the hardware and plugged into the computer, and all of the hardware is powered properly.

Then call `make debug` - don't forget to include any of the additional variables on your make command line, or it may build for the wrong platform. A few eaxmples:

```bash
# If you normally call this to build:
make PLATFORM=UltimakerTests SETTINGS_FILE=my_settings.h

# Then just add the "debug" to the end
make PLATFORM=UltimakerTests SETTINGS_FILE=my_settings.h debug
```

This will build the code (if it needs it) and then open `gdb` connected to the hardware and reset and halt the TinyG2 system.

### A very brief Embedded GDB primer

Even if you are familiar with GDB, you may find a few important differences when using it with an embedded project. This will not be a thorough how-to on this topic, but will hopefully serve enough to get you started.

Using `make debug` will call `gdb` (technically it's `arm-none-eabi-gdb` or whatever variant is specified in the makefile for your board, but we'll just call it `gdb` from now on) with parameters and configuration to:
* Know which binary/elf file to use for this debug session.
* Connect to the board using `openocd`.
* And halt the board, and leave you at a command line prompt.

Now for a few commands to get you started:

#### Flashing the firmware onto the board

`load` - Since `gdb` already knows what bianry/elf file to use, you simple have to type `load` at the `(gdb)` prompt. Here's an example of a succesful flash session:

```bash
(gdb) load
Loading section .text, size 0x1da10 lma 0x80000
Loading section .relocate, size 0x164 lma 0x9da10
Start address 0x82c88, load size 121716
Transfer rate: 14 KB/sec, 13524 bytes/write.
(gdb)
```

#### Resetting the processor

`monitor reset halt` - Once we flash new code, or if we just want the "program" to start over, we call `monitor reset halt` to reset and halt the processor. This will freeze the processor at the reset state, before any code has executed.

You *must* call `monitor reset halt` after (or *immediately* before) a `load` command so that the processor starts from the new code. Otherwise, the processor will likely crash, sometimes after appearing to work for some time. **To avoid nightmare debug sessions, be sure you always follow `load` with `monitor reset halt`!**

#### Quitting the debugger

`q` or `quit` will exit GDB. If the processor is running, you may need to type `CTRL-C` to get the prompt first.

You will often need to reset or power-cycle the board after quitting GDB, since it will likely leave it in a halt state.

#### Running the code

Full docs are [here](https://sourceware.org/gdb/current/onlinedocs/gdb/Continuing-and-Stepping.html#Continuing-and-Stepping) for `continue`, `step`, and `next`.

`c` or `continue` - To resume the processor operating normally, you type `c` (or the full word `continue`) and hit return.

Once it's running, gdb will not show a prompt, and you cannot see any output from the processor. To pause the processor again, hit `CTRL-C`.

NOTE: If you are used to using gdb in a desktop OS, you will notice that we didn't call `run` -- in fact, it won't work. In embedding programming, you're driving the whole OS on the processor, not just running a single program.

#### Step to the next line (into or over functions)

`s` or `step` - Continue running your program until control reaches a different source line, then stop it and return control to GDB. This will step *into* functions, since that would change which line is being executed.

`n` or `next` - Continue to the next source line in the current (innermost) stack frame. This will skip over function calls, since that would change which stack frame is executing.

`fin` or `finish` - Continue running until just after function in the selected stack frame returns. Print the returned value (if any).

#### Getting a backtrace (listing the function call stack)

Full documentation is [here](https://sourceware.org/gdb/current/onlinedocs/gdb/Backtrace.html#Backtrace) for `bt` and related.

`bt` or `backtrace` - Print a backtrace of the entire stack: one line per frame for all frames in the stack. (Note that this may get LONG, and you can make it stop with `CTRL-C`.)

Example:
```gdb
(gdb) bt
#0  0x00085df2 in _controller_HSM () at ./controller.cpp:172
#1  controller_run () at ./controller.cpp:150
#2  0x000889fc in main () at ./main.cpp:169
```

Note that the current execution frame is on the top, and `main()` is on the bottom.

Interpretation: `main()` called `controller_run()`, which called `_controller_HSM()`, which is where we are "currently." It also tells us that that is specifically at line 172 of `./controller.cpp`.

These are more advanced topics covered very well all over the internet, so I won't go into detail here. I will list a few usefull commands to start of your research, however:

#### Breakpoints

Documented [here](https://sourceware.org/gdb/current/onlinedocs/gdb/Set-Breaks.html#Set-Breaks).

* `b` -- set a breakpoint. Will default to "here" but you can pass it a function name or a `filename:line` to break on a specific line.
* `info b` -- show breakpoints.

#### Showing Variables

Documented [here](https://sourceware.org/gdb/current/onlinedocs/gdb/Variables.html#Variables)
* `p expression` - execute the given expression and print the results. Takes most C syntax (but not all) in the expression. **Warning!** It is a common mistake to accidentally change the state or variables when trying to display them. For example, this is a terrible way to test if the `should_blow_up` variable is true:
  ```gdb
	(gdb) p should_blow_up=1
	$1 = 1 '\001'
	```
	It will actually *set* `should_blow_up` to true!! A check, like normal C, would be with the double equals:
	```gdb
	(gdb) p should_blow_up==1
	$2 = false
	```
	`p` will also nicely print out entire structures. You may have to dereference pointers, however:
	```gdb
	(gdb) p mb.r
	$4 = (mpBuf_t *) 0x20071434 <mb+220>
	(gdb) p *mb.r
	$5 = {
		pv = 0x20071368 <mb+16>,
		nx = 0x20071500 <mb+424>,
		# ... clipped some for brevity ...
		jerk = 0,
		recip_jerk = 0,
		cbrt_jerk = 0,
		gm = {
			linenum = 0,
			motion_mode = 0 '\000',
			# ... clipped some for brevity ...
			spindle_mode = 0 '\000'
		}
	}
	(gdb)

	```

---

[![Build Status](https://travis-ci.org/synthetos/g2.svg)](https://travis-ci.org/synthetos/g2)
