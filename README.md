<img src="https://raw.githubusercontent.com/wiki/synthetos/g2/images/g2core.png" width="300" height="129" alt="g2core">

[![Build Status](https://travis-ci.org/synthetos/g2.svg?branch=edge)](https://travis-ci.org/synthetos/g2) [![Issues in Ready](https://badge.waffle.io/synthetos/g2.svg?label=ready&title=Ready)](http://waffle.io/synthetos/g2) [![Issues in Progress](https://badge.waffle.io/synthetos/g2.svg?label=in%20progress&title=In%20Progress)](http://waffle.io/synthetos/g2)

# What it is

g2core is a 9 axes (XYZABC+UVW) motion control system designed for high-performance on small to mid-sized machines.

* CNC
* 3D printing
* Laser cutting
* Robotics

Our default target is the [Arduino Due](https://store.arduino.cc/arduino-due), though it can also be used with other boards.

Some features:

* 9 axis motion (XYZABC+UVW).
  * **Note** - UVW is only in the `edge` branch for now.
* Jerk controlled motion for acceleration planning (3rd order motion planning)
* Status displays ('?' character)
* XON/XOFF and RTS/CTS protocol over USB serial
* RESTful interface using JSON

# Mailing List

For both user and developer discussions of g2core, we recently created a mailing list:

* https://lists.links.org/mailman/listinfo/g2core

Please feel welcome to join in. :smile:

# g2core - Edge Branch

G2 [Edge](https://github.com/synthetos/g2/tree/edge) is the branch for beta testing new features under development. New features are developed in feature branches and merged into the edge branch. Periodically edge is promoted to (stable) master.

Edge is for the adventurous. It is not guaranteed to be stable, but we do our best to achieve this. For production uses we recommend using the [Master branch](https://github.com/synthetos/g2/tree/master).

## Firmware Build 101 `{fb:101.xx}`
### Feature Enhancements
New features added. See linked issues and pull requests for details
- Added [UVW axes](https://github.com/synthetos/g2/wiki/9-Axis-UVW-Operation) for 9 axis control. [See also: Issue 304](https://github.com/synthetos/g2/issues/304)
- Added [Enhanced Feedhold Functions](https://github.com/synthetos/g2/wiki/Feedhold,-Resume,-and-Other-Simple-Commands)
- Added explicit [Job Kill  ^d](https://github.com/synthetos/g2/wiki/Feedhold,-Resume,-and-Other-Simple-Commands#job-kill) - has the effect of an M30 (program end)
- Documented [Communications Startup Tests](https://github.com/synthetos/g2/wiki/g2core-Communications#enqack---checking-for-clean-startup)


### Internal Changes and Bug Fixes
Many things have changed in the internals for this very large pull request. The following list highlights some of these changes but is not meant to be comprehensive.
- Added explicit typing and type testing to JSON variables.
- As part of the above, 32bit integers are not float casts, and therefore retain full accuracy. Line numbers may now reliably go to 2,000,000,000
- Movement towards getters and setters as initial stage of refactoring the Big Table :)
- Bugfix: Fixed root finding problem in feedhold exit velocity calculation
- Bugfix: fixed bug in B and C axis assignment in coordinate rotation code
- PR #334 A, B, C axes radius defaults to use motors 4, 5, & 6
- PR #336, Issue #336 partial solution to coolant initialization
- PR #299, Issue #298 fix for reading nested JSON value errors

## Feature Enhancements

### Firmware Build 101 `{fb:101.xx}`

The fb:101 release is a mostly internal change from the fb:100 branches. Here are the highlights, more detailed on each item are further below:
- Updated motion execution at the segment (smallest) level to be linear velocity instead of constant velocity, resulting in notably smoother motion and more faithful execution of the jerk limitations. (Incidentally, the sound of the motors is also slightly quieter and more "natural.")
- Updated JT (Junction integration Time, a.k.a. "cornering") handling to be more optimized, and to treat the last move as a corner to a move with no active axes. This allows a non-zero stopping velocity based on the allowed jerk and active JT value.
- Probing enhancements.
- Added support for gQuintic (rev B) and fixed issues with gQuadratic board support. (This mostly happened in Motate.)
- Temperature control enhancements
  - Temperature inputs are configured differently at compile time. (Ongoing.)
  - PID control has been adjusted to PID+FF (Proportional, Integral, and Derivative, with Feed Forward). In this case, the feed forward is a multiplier of the difference between the current temperature and the ambient temperature. Since there is no temperature sensor for ambient temperature at the moment, it uses an idealized room temperature of 21ºC.
- More complete support for TMC2130 by adding more JSON controls for live feedback and configuration.
- Initial support for Core XY kinematics.
- Boards are in more control of the planner settings.
- Experimental setting to have traverse (G0) use the 'high jerk' axis settings.
- Outputs are now configured at board initialization (and later) to honor the settings more faithfully. This includes setting the pin high or low as soon as possible.

### Firmware Build 100 `{fb:100.xx}`

The fb:100 release is a major change from the fb:089 and earlier branches. It represents about a year of development and has many major feature enhancements summarized below. These are described in more detail in the rest of this readme and the linked wiki pages.
- New Gcode and CNC features
- 3d printing support, including [Marlin Compatibility](https://github.com/synthetos/g2/wiki/Marlin-Compatibility)
- GPIO system enhancements
- Planner enhancements and other operating improvements for high-speed operation
- Initial support for new processors, including the ARM M7

### Project Changes

The project is now called g2core (even if the repo remains g2). As of this release the g2core code base is split from the TinyG code base. TinyG will continue to be supported for the Xmega 8-bit platform, and new features will be added, specifically as related to continued support for CNC milling applications. The g2core project will focus on various ARM platforms, as it currently does, and add functions that are not possible in the 8-bit platform.

In this release the Motate hardware abstraction layer has been split into a separate project and is included in g2core as a git submodule. This release also provides better support for cross platform / cross target compilation. A summary of project changes is provided below, with details in this readme and linked wiki pages.
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

### Edge branch, Build 101.xx

This build is primarily focused on support for the new boards based on the Atmel SamS70 family, as well as refining the motion control and long awaited feature enhancements. This list will be added to as development proceed.s

### Edge branch, Build 100.xx

Build 100.xx has a number of changes, mostly related to extending Gcode support and supporting 3D printing using g2core. These include temperature controls, auto-bed leveling, planner performance improvements and active JSON comments in Gcode.

Communications has advanced to support a linemode protocol to greatly simplify host communications and flow control for very rapid Gcode streams. Please read the Communications pages for details. Also see the NodeJS communications module docs if you are building a UI or host controller.

Build 100.xx also significantly advances the project structure to support multiple processor architectures, hardware configurations and machine configurations in the same code base. Motate has been cleaved off into its own subproject. We recommend carefully reading the Dev pages if you are coding or compiling.

#### Functional Changes:

*Note: Click the header next to the arrow to expand and display the details.*

<details><summary><strong>Linear-Velocity Segment Execution</strong></summary>

  - The overall motion is still jerk-controlled and the computation of motion remains largely the same (although slightly simplified). At the smallest level above raw steps (what we call "segments," which are nominally 0.25ms to 1ms in duration) we previously executed the steps at a constant velocity. We now execute them with a linear change from a start velocity to an end velocity. This results in smoother motion that is more faithful to the planned jerk constraints.
  - This changed the way the forward differences are used to compute the segment speeds as well. Previously, we were computing the curve at the midpoint (time-wise) of each segment in order to get the median velocity. Now that we want the start and end velocity of each segment we only compute the end (time-wise) of each segment, and use that again later as the start-point of the next segment.
</details>

<details><summary><strong>Probing enhancements</strong></summary>

  - Added `{"prbs":true}` to store the current position as if it were to position of a succesful probe.
  - Added `{"prbr":true}` to enable and `{"prbr":false}` to enable and disable (respectively) the JSON `{prb:{...}}` report after a probe.
</details>

<details><summary><strong>gQuintic support</strong></summary>

  - Support for the gQuintic rev B was added. Support for rev D will come shortly.
</details>

<details><summary><strong>Temperature control enhancements</strong></summary>

  - Added the following settings defines:
    - `HAS_TEMPERATURE_SENSOR_1`, `HAS_TEMPERATURE_SENSOR_2`, and `HAS_TEMPERATURE_SENSOR_3`
    - `EXTRUDER_1_OUTPUT_PIN`, `EXTRUDER_2_OUTPUT_PIN`, and `BED_OUTPUT_PIN`
    - Added `BED_OUTPUT_INIT` in order to control configuration of the Bed output pin settings.
    - Defaults to `{kNormal, fet_pin3_freq}`.
    - `EXTRUDER_1_FAN_PIN` for control of the temperature-enabled fan on extruder 1. (Only available on extruder 1 at the moment.)
  - (*Experimental*) Analog input is now interpreted through one of various `ADCCircuit` objects.
    - Three are provided currently: `ADCCircuitSimplePullup`, `ADCCircuitDifferentialPullup`, `ADCCircuitRawResistance`
    - `Thermistor` and `PT100` objects no longer take the pullup value in their constructor, but instead take a pointer to an `ADCCircuit` object.
  - `Thermistor` and `PT100` objects no longer assume an `ADCPin` is used, but now take the type that conforms to the `ADCPin` interface as a template argument.
  - **TODO:** Make more of these configurable at runtime. Separate the ADC input from the consumer, and allow other things than temperature to read it.
  - PID+FF control adds feed-forward (FF) to adjust the output to a reasonable minimum based on heat loss dues to room temperature.
    - This can be effectively disabled, making the controller a PID controller, by setting the F value to `0.0`.
    - **Warning** setting this value too high can cause thermal runaway. Set this value conservatively (low), since there's currently no ambient temperature, and the actual heat loss may be less than computed. This will be magnified by another heater (such as that on a heat bed of a 3D printer) in close proximity.

</details>

<details><summary><strong>TMC2130 JSON controls</strong></summary>

  - Added the following setting keys to the motors (`1` - `6`):
    - `ts`   - *(R)* get the value of the `TSTEP` register
    - `pth`  - *(R/W)* get/set the value of the `TPWMTHRS` register
    - `cth`  - *(R/W)* get/set the value of the `TCOOLTHRS` register
    - `hth`  - *(R/W)* get/set the value of the `THIGH` register
    - `sgt`  - *(R/W)* get/set the value of the `sgt` value of the `COOLCONF` register
    - `sgr`  - *(R)* get the `SG_RESULT` value of the `DRV_STATUS` register
    - `csa`  - *(R)* get the `CS_ACTUAL` value of the `DRV_STATUS` register
    - `sgs`  - *(R)* get the `stallGuard` value of the `DRV_STATUS` register
    - `tbl`  - *(R/W)* get/set the `TBL` value of the `CHOPCONF` register
    - `pgrd` - *(R/W)* get/set the `PWM_GRAD` value of the `PWMCONF` register
    - `pamp` - *(R/W)* get/set the `PWM_AMPL` value of the `PWMCONF` register
    - `hend` - *(R/W)* get/set the `HEND_OFFSET` value of the `CHOPCONF` register
    - `hsrt` - *(R/W)* get/set the `HSTRT/TFD012` value of the `CHOPCONF` register
    - `smin` - *(R/W)* get/set the `semin` value of the `COOLCONF` register
    - `smax` - *(R/W)* get/set the `semax` value of the `COOLCONF` register
    - `sup`  - *(R/W)* get/set the `seup` value of the `COOLCONF` register
    - `sdn`  - *(R/W)* get/set the `sedn` value of the `COOLCONF` register
  - Note that all gets retrieve the last cached value.
</details>

<details><summary><strong>Core XY Kinematics Support</strong></summary>

  - Enabled at compile-time by setting the `KINEMATICS` define to `KINE_CORE_XY`
    - The default (and only other valid value) for `KINEMATICS` is `KINE_CARTESIAN`
  - Note that the X and Y axes must have the same settings, or the behavior is undefined.
  - For the sake of motor mapping, the values `AXIS_COREXY_A` and `AXIS_COREXY_B` have been created.
  - Example usage:
  ```c++
  #define M1_MOTOR_MAP                AXIS_COREXY_A           // 1ma
  #define M2_MOTOR_MAP                AXIS_COREXY_B           // 2ma
  ```
</details>

<details><summary><strong>Planner settings control from board files</strong></summary>

  - The defines `PLANNER_QUEUE_SIZE` and `MIN_SEGMENT_MS` are now set in the `board/*/hardware.h` files.
  - `PLANNER_QUEUE_SIZE` sets the size of the planner buffer array.
    - Default value if not defined: `48`
  - `MIN_SEGMENT_MS` sets the minimum segment time (in milliseconds) and several other settings that are comuted based on it.
    - Default values if not defined: `0.75`
    - A few of the computed values are shown:
    ```c++
    #define NOM_SEGMENT_MS              ((float)MIN_SEGMENT_MS*2.0)        // nominal segment ms (at LEAST MIN_SEGMENT_MS * 2)
    #define MIN_BLOCK_MS                ((float)MIN_SEGMENT_MS*2.0)        // minimum block (whole move) milliseconds
    ```
</details>

<details><summary><strong>Experimental traverse at high jerk</strong></summary>

  - The new define `TRAVERSE_AT_HIGH_JERK` can be set to `true`, making traverse (`G0`) moves (including `E`-only moves in Marlin-flavored gcode mode) will use the jerk-high (`jh`) settings.
    - If set to `false` or undefined `G0` moves will continue to use the jerk-max (`jm`) settings that feed (`G1`) moves use.
</details>

<details><summary><strong>PID+FF - added feed forward</strong></summary>

  - There is a new JSON value `f` in each `pid`*`n`* object (read-only, for reporting) as well as an `f` setting in the `he`*`n`* objects (for control).
    - This is controlled in the settings file via `H`*`n`*`_DEFAULT_F`, such as `H1_DEFAULT_F`. Default value is `0.0`.
    - This is a value that is multiplied to by current temp - 21 and added to the current computed output.
    - **Warning!** Setting this value too high can result in thermal runaway. Set it conservatively (low) or disable it completely if in doubt.
    - Set the `he`*`n`*`f` value to `0.0` to effectively disable feed-forward.

</details>

<details><summary><strong>Output setting as soon as possible</strong></summary>

  - At board initialization, the output value on each of the `out` objects is set to whatever the pin is configured to be "inactive." This is based on the settings file `DO`*n*`_MODE` setting.
  - For example, if `DO10_MODE == IO_ACTIVE_LOW` then the pin at `DO10` is initialized as `HIGH` at board setup. This happen even before the `main()` function starts, shortly after the GPIO clocks are enabled for each port.
</details>
