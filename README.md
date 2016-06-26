# TinyG 2 - Edge Branch

This branch (`edge`) is for the adventurous. It is not guaranteed to be stable. It's not guaranteed AT ALL.

### Notes

- Get rid of the build instructions and point people at the correct wiki pages
- Take a pass through those same wiki pages to make sure they are up to date
- Add a Line Transmission Protocol wiki page - perhaps combined with the G sender information
- Update Status COdes page
- Document Tram command
- DOcucment temerature control commands (and M100's)
- Remove _debug_trap()s_


## Changelog for Edge Branch

### Edge branch, Build 100.06

Build 100.xx has a number of changes, mostly related to supporting 3D printing using G2. These include temperature controls, auto-bed leveling, planner performance improvements and active JSON comments in Gcode.

Communications has been advanced to support a linemode protocol to greatly simplify host communications and flow control for very rapid Gcode streams. Please read the Communications pages for details. Also see the NodeJS G Sender docs if you are building a UI or host controller.

Build 100.xx also significantly advances the project structure to support multiple processor architectures, hardware configurations and machine configurations in the same code base. Motate has been cleaved off into its own subproject. We recommend carefully reading the Dev pages if you are coding or compiling.

#### Functional Changes:
- Planner improvements to handle extreme cases found in some 3DP slicer outputs

- `{ja:n}` Junction Aggression replaces Junction Acceleration. Cornering now obeys full jerk limitation instead of the centripetal acceleration heuristic. JA is now a scaled value that is nominally set to 1.000. Set to less than 1 for slower cornering (less aggressive), greater than 1 (but probably less than 2) for more aggressive cornering.

- Deprecated `{_jd:n}` as it was in support of the Junction Acceleration scheme.

- Added `{fbc:n}` to report configuration file used during compilation

- Added `G38.3`, `G38.4`, `G38.5` Gcodes to complete the G38.2 probing suite

- Added automatic bed leveling (tramming) using 3 point probe and coordinate rotation

- Added `{he1:n}`, `{he2:n}`, `{he3:n}` heater control groups. These codes are experimental and will change.

- Added `{pid1:n}`, `{pid2:n}`, `{pid3:n}` ADC PID groups. These codes are experimental and may change.

- Added `{do1:n}` ... `{do12:n}` digital output controls for controlling fans. These are experimental and will change.

- Added `{out1:n}` ... `{out12:n}` digital output state readers for reading the condition of do's. These are experimental and may change.

- Added `M100 ({...})` active comment. Currently only supports temperature setting command. The semantics of the temperature commands are experimental and will be changed in later releases.

- Added `M101 ({...})` "wait-on-event" active comment. Currently only supports temperature wait command. The semantics of the temperature commands are experimental and will be changed in later releases.

- Added `Linemode` communication protocol, and provide guidance to use linemode for much simpler and more reliable application-level flow control

- Footer format has changed. Checksum is no longer supported and has been removed `(CONFIRM THAT THIS IS A build 100 change and not earlier)`

- Added `ENQ/ACK handshake`. If the host sends an ASCII ENQ (0x05) the board should respond with an ACK (0x06). This is provided to facilitate automated testing (See Github/Synthetos/tg_pytest)

- Added `setpoint` to homing to accommodate setting home position for non-zero switches.

- Exception reports now provide more information about the nature and location of the exception.

- Made changes to the Status Codes. See Status Codes wiki page

- Removed `{cv:n}` configuration version tag

- Code level changes are numerous. Here are a few:
  - Added `tinyg_info.h` to isolate revision info from tinyg.h
  - Removed char_t casts

#### G sender
- Provide links to Node G Sender and instruction how to install and use

#### Project Structure and Motate Changes
- Motate underpinnings and project structure have changed significantly to support multiple processor architectures, boards, and machine configurations cleanly in the same project. If this affects you please read up on the wiki.

#### Known Issues
- Communications bug for high-speed transmission
- sbv300 configuration does not compile
- Three compile warnings: `Changing start of section by 8 bytes` in ld.exe are thrown. These should be ignored (and if you know how to turn them off please let us know).


## Earlier Edges

### Edge branch, build 083.07
These changes are primarily fixes applied after testing
- Fixes to spindle speed settings (082.11)
- Fixes to build environments for Linux and other platforms
- Fixes to planner operation from edge-replan-replan
- Fixes for reporting error in inches mode

### Edge branch, build 082.10
These changes are still under test. If you find bugs or other issues please log to Issues.
- **[Digital IO (GPIO)](Digital-IO-(GPIO))** introduces major changes to the way switches and other inputs are handled. The digital inputs are completed, the digital outputs have not been. In short, inputs are now just numbered inputs that are mapped to axes, functions, and motion behaviors (feedholds).
  - **Your configurations will need to change to accommodate these changes.** See settings/settings_shapeoko2.h for an example of setup and use - pay particular attention to `axis settings` and the new `inputs` section.
  - Typing `$`, `$x`, `$di`, `$in` at the command line is also informative. Of course, all these commands are available as JSON, but in text mode you get the human readable annotations.
  - These changes also rev the firmware version to 0.98 from 0.97, as a new configuration wiki page will need to be generated (not started yet).
  - {lim:0}, {lim:1} was added to allow a limit override to backing off switches when a limit is tripped
  - See also [Alarm Processing](Alarm-Processing), which is intimately related to these changes.

- **[Alarm processing](Alarm-Processing)** has been significantly updated. There are now 3 alarm states:
  - [ALARM](Alarm-Processing#alarm) - used to support soft and hard limits, safety interlock behaviors (door open), and other conditions.
  - [SHUTDOWN](Alarm-Processing#shutdown) - used to support external ESTOP functions (the controller doe NOT do ESTOP - read the SHUTDOWN section as to why.
  - [PANIC](Alarm-Processing#panic) - shuts down the machine immediately if there is an assertion failure or some other unrecoverable error
  - [CLEAR](Alarm-Processing#clear) describes how to clear alarm states.

- **[Job Exception Handling](Job-Exception-Handling)** has been refined. A new Job Kill has been introduced which is different than a queue flush (%), as these are actually 2 very different use cases.

- **Homing** changes. Homing input switches are now configured differently.
  - The switch configurations have been removed from the axes and moved to the digital IO inputs.
  - Two new parameters have been added to the axis configs. All other parameters remain the same.
    - {xhd:1} - homing direction - 0=search-to-negative, 1=search-to-positive
    - {xhi:N} - homing input - 0=disable axis for homing, 1-N=enable homing for this input (switch)
  - Note that setting the homing input to a non-zero value (1) enables homing for this axis, and (2) overrides whatever settings for that input for the duration of homing. So it's possible to set di1 (Xmin) as a limit switch and a homing switch. When not in homing it will be used as a limit switch.

- **Safety Interlock** added.
  - An input configured for interlock will invoke a feedhold when the interlock becomes diseangaged and restart movement when re-engaged.
  - {saf:0}, {saf:1} was added to enable or disable the interlock system.
  - There are optional settings for spindle and coolant actions on feedhold. See below

- **Spindle Changes** Expect updates to spindle behaviors in future branches. Here's where it is now:
  - The spindle can be paused on feedhold with the Spindle-pause-on-hold global setting {spph:1}. For now we recommend not using this {spph:0} as there is not yet a delay in spindle restart.
  - Spindle enable and direction polarity can now be set using the {spep: } and {spdp: } commands.
  - Spindle enable and direction state can be returned using {spe:n} and {spd:n}, and these can be configured in status reports
  - Spindle speed can be returned using {sps:n} and can be configured in status reports

- **Coolant Changes** Expect coolant changes in future branches, in particular to accommodate changes in the digital outputs.
  - The coolant can be paused on feedhold with the Coolant-pause-on-hold global setting {coph:0}.
  - Flood and mist coolant polarity can now be set using the {cofp: } and {comp: } commands.
  - Flood and mist coolant state can be returned using {cof:n} and {com:n}, and these can be configured in status reports
  - In v9 the flood (M8) and mist (M7) commands are operative, but map the same pin. M9 clears them both, as expected. These should both be set to the same polarity for proper operation. On a Due or a platform with more output pins these can be separated - the code is written for this possibility. The changes should be limited to the pin mapping layers.

- **Power Management** is fully working, as far as we can tell. See $1pm for settings

- **Arc Changes** have been added. Please note any issues immediately. This is still under test.
 - Fixed bug on very large arcs
 - Fixed bug on G18 rotation direction
 - Added P parameter to allow for arcs > 360 degree rotation

- **G10 L20** was added for easier offset setting

- **Bug Fixes**
  - Fixed some units mode display errors for G20 mode (inches)

- **Still To Go**
  - SD card persistence
  - Spindle restart dwell
  - Digital output generalization and changes
  - Still needs rigorous testing for very fast feedhold/resume and flush cycles

###Edge branch, build 071.02

* **No Persistence**. Most ARM chips (including the ATSAM3X8C on v9 and ATSAM3X8E on the Arduino Due) do not have persistence. This is the main reason the v9 has a microSD slot. But this has not been programmed yet. So your options are to either load the board each time you fire it up or reset it, or to build yourself a profile and compile your own settings as the defaults.

* **Still working on feedhold.** The serial communications runs a native USB on the ARM instead of through an FTDI USB-to-Serial adapter. We are still shing some bugs out of the single character commands such as feedhold (!), queue flush (%) and cycle start (~).

* **Power Management needs work.** It doesn't always shut the motors off at the end of a cycle.

* **Different Behaviors**. There are some behaviors that are different.
  * Feedhold / queue flush on v8 works with !%~ in one line. In g2 it requires a newline. Use !\n%\n  This is due to using a USB stack that is partly on the chip and not being able to get at the individual characters that far upstream. This will probably not change in v9.


# ---- DEPRECATED ----

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
