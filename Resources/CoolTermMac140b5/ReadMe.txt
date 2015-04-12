
                    CoolTerm

       Copyright (c) 2007-2009 Roger Meier,
              All rights reserved

         http://freeware.the-meiers.org



WHAT IS IT?
===========

CoolTerm is an easy-to-use terminal for communication with hardware connected to serial ports.

CoolTerm is a simple serial port terminal application (no terminal emulation) that is geared towards hobbyists and professionals with a need to exchange data with hardware connected to serial ports such as servo controllers, robotic kits, GPS receivers, microcontrollers, etc.
The features of CoolTerm include:
- Capability of multiple concurrent connections if multiple serial ports are available.
- Display of received data in plain text or hexadecimal format.
- Sending data via keypresses as well as a "Send String" dialog that supports data entry in plain text or hexadecimal format.
- Sending data via copy-paste of text into the terminal window.
- Sending of text files.
- Capability of capturing received data to text files.
- Local echo of transmitted data.
- Local echo of received data (loop back to sender).
- Hardware (CTS, DTR) and software flow control (XON).
- Optical line status indicators.
- Capability of manually toggling line states of certain handshaking signals when hardware flow control is disabled.
- Configurable character and line delays.
- Capability of saving and retrieving connection options.
- and more...



INSTALLATION
============
CoolTerm comes without an installer and can be placed anywhere on the harddrive as long as the correct folder structure is maintained. I.e. for the Windows version the "CoolTerm Libs" folder must reside in the same location as the "CoolTerm.exe" executable.



HOW TO USE IT
=============

Please refer to the built-in help.



VERSION HISTORY
===============

1.4.0b5
-------
- AppleScript BETA: Added basic AppleScript handling for the most important terminal operations such as loading a settings file, opening/closing ports, and reading/writing data. The AppleScript functionality will start as a BETA in itself, even after CoolTerm 1.4.0 is released, to allow a broader audience to beta test this feature and provide feedback. Refer to the attached "AppleScript ReadMe.txt" file for more details.
- Added an option to keep the capture file open while capturing is in progress (default) or close it after writing and re-opening when new data arrives. This allows other applications to read the capture file while capturing is in progress.


1.4.0b4
-------
- Added error messages to alert the user of errors that occur while attempting to open the serial port.
- Added the option to specify additional baud rates via the "baudrates.ini" file. E.g. any baud rates that are known to be supported by the hardware that are not listed in the popup menu in the connection settings dialog can be added to a "baudrate.ini" file that resides in the same directory as CoolTerm.
- [Mac] Implemented a workaround for a known RB bug where the baudrates 3600, 7200, 14400, and 28800 baud would not be set correctly and default to 57600 baud instead.
- Fixed a bug that would show an error message when the user chooses cancel in the capture file save dialog.


1.4.0b3
-------
- Added check to warn the user if multiple files are dropped onto the CoolTerm window.
- Flow control settings are now displayed in the terminal window as part of the port configuration string. For XON/XOFF the state is displayed, i.e. XON or XOFF.
- Changed the name of SendTextPacketSize parameter to TXPacketSize.
- CoolTerm now opens a progress window whenever the length of the text to be transmitted exceeds TXPacketSize, and not only when text files are sent. 
- "Send String" windows can now be resized.
- The transmit code is now using the "BytesLeftToSend" property of the serial port control to throttle the data when sending large text files as well as to enable the progress bar to more accurately reflect the transmission progress.
- Added status LEDs for TX and RX to indicate activity related to sending and receiving data.
- Added preferences option to disable all menu shortcuts (on Windows and Linux only) in order to allow sending CTRL characters via the terminal. On Mac, the keyboard shortcuts use the Command key and thus don't interfere with CTRL characters.
- It is now possible to send CTRL characters when the terminal is set to Line Mode.
- Improved code for Line Mode to ensure that a pressed key is captured even if the command line didn't have the focus.


1.4.0b2
-------
- Compiled Linux build of CoolTerm. Since I don't have a Linux system I won't be able to support the Linux build, but preliminary testing by another user seems to indicate that it works as expected.
- Added the option to specify additional serial ports via the "ports.ini" file. E.g. any devices such as /dev/tty.xxx devices on OSX and Linux that CoolTerm can not enumerate can be added to a "ports.ini" file that resides in the same directory as CoolTerm.
- It is now possible to change baudrate, byte format settings, and flow control settings while the port is open.
- Made further improvements to the code that processes received data. A timer is now used to read data from the input buffer as opposed to reading in the DataAvailable event of the serial port class.
- Changed the default state of DTR to "active" with the exception of Windows platforms when DTR flow control is enabled, in which case the default is "inactive" in order to avoid serial port errors.
- Added a "SendTextPacketSize" parameter to the settings file for debug purposes. The default is 256.
- Changed behavior of data transmission from "quasi synchronous" to completely asynchronous, by no longer invoking XmitWait (and basically locking the application) until all data in the transmit buffer has been sent, which can lead to unresponsiveness and even crashes when XON/XOFF flow control is enabled. The downside is that the transmission status bar when sending text files now corresponds to the progress of writing data to the transmit buffer as opposed to data being sent out of the port, which is much faster. 
- Added a "XmitWaitEnable" parameter to the settings file for debug purposes. The default is false. Setting it to true reverts to transmission behavior back to "quasi synchronous".
- Fixed a bug that threw an exception when opening the connection settings on a system without serial ports installed.
- Fixed a bug the displayed an error message when the user cancelled out of the "Send Textfile" dialog.

1.4.0b1: 05/29/2011
-------------------
- Added the option to add timestamps to data captured to text files.
- Added a keyboard shortcut to connect/disconnect.
- New Connection options window with multiple pages.
- Connection options window now displays port information for the selected port.
- Added option to replace TAB key presses with a configurable number of spaces (default = 4).
- Added option to enable/disable capturing of local echo of transmitted data in capture files.
- Changed architecture of Receive Buffer to a circular buffer. This improves efficiency as well as robustness.
- Changed behavior of the status LEDs to better reflect the state of the signals. A green LED turned on now means that a signal is "active", the LED turned off means that it is "inactive".
- Changed the default state of DTR when a port is opened to "active" to conform with common practice.
- Improved handling of file IO errors when sending textile or capturing received data to textiles.
- Improved handling of file IO errors when reading and writing settings files.
- Improved error reporting. Crash reports will now include information about all open terminals.
- Slight change to the behavior during setting the break signal in that no characters are being read from the receive buffer. Received characters will be read after the break signal has been cleared.
- [Windows] Fixed a bug where the removal of a serial port adapter could cause an exception when starting a connection.


1.3.1: 1/11/2011
----------------
Improvements:
- Added a preferences option to automatically check for updates at startup.

Fixes:
- Fixed a bug that caused a StackOverFlowException when serial port devices were unexpectedly removed from the system, e.g. when a USB serial adapter was unplugged while the terminal was connected to that device. The error handling for this situation has been improved.
- Fixed a bug that caused an OutOfBoundsException when a serial port device failure occurred during enumeration.
- Fixed a bug that resulted in incorrect formatting of long crash reports.


1.3.0: 10/28/2010
-----------------
New features:
- Added a transmit line delay option which adds a specified delay after certain characters such as new line characters (configurable).
- Added a transmit character delay option (configurable).
- Added a "Connection/Send Break" menu item for sending serial breaks.
- Added the option to play a notification sound after a text file has been sent.
- Added auto-connect feature.
- Added the .hex file extension to the "Text Files" file type set (for the "Send Text File" dialog).
- It is now possible to have default settings loaded at startup and when a new terminal window is opened. If a default.stc settings file exists in the application folder of CoolTerm, it will be applied to new terminal windows.
- Added a menu item to save current settings as default settings.

Improvements:
- Pressing ENTER or RETURN in the connection settings dialog now envokes the "Ok" button, even if a textfield is currently selected.
- Pressing ESC in the connection settings dialog now invokes the "Cancel" button, even if a textfield is currently selected.
- Pressing Shift+ENTER or Shift+RETURN now invokes the "Send" button in "Send String" windows.
- Improved handling of command line arguments.
- The values for "Receive Buffer Size" and the character and line delays are now limited to a range from 0 to a maximum value (2,147,483,647 and 10,000, respectively).
- When a "Send String" window is opened, the text field now receives focus automatically.
- Improved exception handling and error reporting.
- Improved behavior of the command history buffer and menu.
- GUI improvements.

Fixes:
- Fixed a bug that allowed opening multiple "Save As..." for the same Terminal window dialogs on Windows.
- Fixed a bug that could cause a StackOverflow on serial port errors due to calling port.flush
- Fixed bug that could cause a crash when sending empty strings via a "Send  String" window.
- (Win) Fixed issue that would allow the terminal window to be activated via the taskbar when the connection options window is open.
- Several minor bug fixes.


1.2.0: 2/19/2010
----------------
- Added "Line Mode" to the communication settings. In "Line Mode" a line of typed text will not be sent to the serial port until the Enter key is pressed.
- Added "History" which is available in "Line Mode" the up and down arrow keys can be used to select previously typed lines.
- Added a receive buffer size limit option.
- Added handling of the bell character (ASCII code 7), which can be enabled through the communication settings.
- It is now possible to open the communication settings and edit certain options while the serial port is open.
- The viewer mode (plain or hex) is now saved as parameter in connection settings files.
- The size and position of terminal windows is now saved with connection settings.
- Fixed bug that converted occurrences CR+CR+LF strings to single spaces on Windows.


1.1.2: 7/17/2009
----------------
- Separated option to handle backspace characters in ASCII view from option to convert non-printable characters.
- Changed character used to display non-printable characters to a period (ASCII code 46) for better compatibility and consistency across platforms.
- Changed short cuts for "View/Autoscroll" and View Mode menu items to avoid conflict with other menu items such as "Edit/Select All".
- Windows build now associates .stc files with CoolTerm.
- Minor bug fixes.


1.1.1: 6/29/2009
----------------
- Added option to handle backspace characters in ASCII view to Connection Settings.
- Fixed bug in SendString that prevented typing 8 in hex mode.
- Fixed bug that printed the wrong character for cursor down key when ConvertNonPrint was enabled.
- Added a "Check for Updates" Menu item.


1.1.0: 6/18/2009
----------------
- Added an option to the connection settings to automatically terminate string sent from "Send String" windows with a configurable "Termination String", such as e.g. a linefeed etc.
- In ASCII view mode, all incoming "New Line" such as CR, LF, CR+LF, are now properly displayed as line breaks.
- Added an option to the connection settings to convert non-printable characters to generic dot characters in ASCII view.
- Added 'View' menu with menu item to switch between Hex and ASCII viewing.
- Moved 'Clear Data' menu item to 'View' menu.
- Added an 'Autoscroll' feature, accessible via the 'View' menu to enable/disable automatic scrolling of received data.
- Changed menu shortcut key for "Cycle through windows" from "0" to "`".
- Added code to produce an audible alert (system beep) when characters are typed while the serial port is not connected.
- Added a 'Help' button to the toolbar


1.0.0: 5/19/2009
----------------
- Initial Release




LICENSE
=======

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT OF THIRD PARTY RIGHTS. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR HOLDERS INCLUDED IN THIS NOTICE BE LIABLE FOR ANY CLAIM, OR ANY SPECIAL INDIRECT OR CONSEQUENTIAL DAMAGES, OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
