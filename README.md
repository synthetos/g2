[Waffle.io](http://waffle.io/synthetos/g2): [![Issues in Ready](https://badge.waffle.io/synthetos/g2.svg?label=ready&title=Ready)](http://waffle.io/synthetos/g2) [![Issues in Progress](https://badge.waffle.io/synthetos/g2.svg?label=in%20progress&title=In%20Progress)](http://waffle.io/synthetos/g2)

# g2core - Edge Branch

The g2core code base is still under development. This branch (`edge`) is currently the main branch, and the best branch for testing and development, as Master is way out of date.

That said, Edge is for the adventurous. It is not guaranteed to be stable, but we do our best to achieve this. Once edge build 100 has settled out a bit it will be pushed to Master.

## Firmware Build 100 `{fb:100.00}`
### Feature Enhancements
The fb:100 release is a major change from the fb:089 and earlier branches. It represents about a year of development and has many major feature enhancements summarized below. These are described in more detail in the rest of this readme and the linked wiki pages.
- New Gcode and CNC features
- 3d printing support
- GPIO system enhancements
- Planner enhancements and other operating improvements for high-speed operation

### Project Changes
The project is now called g2core (even if the repo remains g2). As of this release the g2core code base is split from the TinyG code base. TinyG will continue to be supported for the Xmega 8-bit platform, and new features will be added, specifically as related to continued support for CNC milling applications. The g2core project will focus on various ARM platforms, as it currently does, and add functions that are not possible in the 8-bit platform.

In this release the Motate hardware abstraction layer has been split into a separate project, and is included in g2core as a git submodule. This release also provides better support for cross platform / cross target compilation. A summary of project changes is provided below, with details in this readme and linked wiki pages.
- Motate submodule
- Cross platform / cross target support
- Multiple processor support - ARM M3, M4, M7 cores
- Device tree / multiple motor types
- Simplified host-to-board communication protocol (line mode)
- NodeJS host module for host-to-board communications

### More To Come
The fb:100 release is the base for  number of other enhancements in the works and planned, including:
- Further enhancements to GPIO system
- Additional JSON processing and UI support
- Enhancements to 3d printer support, including a simplified g2 printer dialect

## Changelog for Edge Branch

### Edge branch, Build 100.00

Build 100.xx has a number of changes, mostly related to extending Gcode support and supporting 3D printing using g2core. These include temperature controls, auto-bed leveling, planner performance improvements and active JSON comments in Gcode.

Communications has advanced to support a linemode protocol to greatly simplify host communications and flow control for very rapid Gcode streams. Please read the Communications pages for details. Also see the NodeJS communications module docs if you are building a UI or host controller.

Build 100.xx also significantly advances the project structure to support multiple processor architectures, hardware configurations and machine configurations in the same code base. Motate has been cleaved off into its own subproject. We recommend carefully reading the Dev pages if you are coding or compiling.

#### Functional Changes:
- **Gcode and CNC Changes**
  - Included `G10 L1`, `G10 L10`, `G43`, `G49` tool length offset and added 32 slot tool table
  - Included `G10 L20` offset mode
  - Extended `G38.2` probing to also include `G38.3`, `G38.4`, `G38.5`
  - Homing can now be set to a non-zero switch. Homing will set to the travel value of the positive or negative switch, as determined by the search direction. This allows homing to home to a maximum - for example - and set the homed location to the non-zero switch.


- **Planner and Motion Changes**
  - Junction Integration Time - the `{jt:...}` parameter is now the way to set cornering velocity limits. Cornering now obeys full jerk limitation instead of the centripetal acceleration heuristic, making it much more accurate and more true to the jerk limits set for the machine. JT is a normalized scaled factor that is nominally set to 1.000. Set to less than 1 for slower cornering (less aggressive), greater than 1 (but probably less than 2) for more aggressive cornering. This command replaces Junction Acceleration `{ja:...}` and the axis Junction Deviation commands - e.g. `{xjd:0.01}`.
  - Deprecated `{ja:...}` global parameter. Will return error.
  - Deprecated `{_jd:...}` per-axis parameter. Will return error.


- **3D Printing Support**
  - Planner improvements to handle extreme cases found in some 3DP slicer outputs
  - Added automatic bed leveling (tramming) using 3 point probe and coordinate rotation
  - Added `{he1:n}`, `{he2:n}`, `{he3:n}` heater control groups
  - Added `{pid1:n}`, `{pid2:n}`, `{pid3:n}` ADC PID groups
  - Note: The semantics of `{he:...}` and `{pid:...}` are still in development and may change.


- **GPIO Changes**
  - Changed configuration for `{di1:n}` ... `{di12:n}` somewhat
  - Added `{do1:n}` ... `{do12:n}` digital output controls for controlling general outputs such as fans
  - Added `{out1:n}` ... `{out12:n}` digital output state readers for reading the condition of do's


- **Active JSON comments** - i.e. JSON called from Gcode
  - Added `M100 ({...})` active comment. Currently only supports temperature setting command.
  - Added `M101 ({...})` "wait-on-event" active comment. Currently only supports temperature wait command.


- **System and Communications**
  - Added `Linemode` communication protocol, and provide guidance to use linemode for much simpler and more reliable application-level flow control
  - Footer format has changed. Checksum is no longer supported and has been removed
  - Added `ENQ/ACK handshake`. If the host sends an ASCII `ENQ (0x05)` the board should respond with an `ACK (0x06)`. This is provided to facilitate low-level communications startup and automated testing
  - Added `{fbs:n}` as a read-only parameter to report the git commit used during compilation
  - Added `{fbc:n}` as a read-only parameter to report the configuration file used during compilation
  - Removed `{cv:n}` configuration version tag
  - Exception reports now provide more information about the nature and location of the exception
  - Changes to [Status Codes](Status-Codes) (...or see error.h for source)
  - Additional [`stat` machine states](Status-Reports#stat-values)
  - Removed code for embedded tests. These were a holdover from the TinyGv8 codebase and were not functional in g2. The code is now removed from the project.


- **Project Structure and Motate**
  - Motate underpinnings and project structure have changed significantly to support multiple processor architectures, boards, and machine configurations cleanly in the same project. If this affects you please read up on the wiki.


- **NodeJS g2core Communcications Module**
  - A pre-release of the NodeJS g2core communications module that uses Linemode protocol is available here. This will be superseded with the official release


- **Automated Regression Testing**
  - A simple Python functional and regression test suite is available in Githup/Synthetos/tg_pytest. Please feel free to use and extend, but be aware that we are not offering much support for this. If you are familiar with Python and JSON the Readme should have everything you need.


#### Known Issues
- Communications bug for high-speed transmission


## Earlier Edges

### Edge branch, build 083.07
These changes are primarily fixes applied after testing
- Fixes to spindle speed settings (082.11)
- Fixes to build environments for Linux and other platforms
- Fixes for reporting error in inches mode

### Edge branch, build 082.10
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

- **Safety Interlock** added
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

---

[![Build Status](https://travis-ci.org/synthetos/g2.svg)](https://travis-ci.org/synthetos/g2)
