PROGRAMMING THE DUE FROM OS X
=============================

These notes are also available here and may be more up-to-date:
https://github.com/synthetos/g2/wiki/Programming-TinyG2

Download the Arduino 1.5.2 BETA (not the 1.0.5 release at the top of the page) from http://arduino.cc/en/Main/Software for OS X.

Expand the downloaded zip file and move Arduino.app to /Applications on your hard drive. (Often, this is found on the sidebar of Finder windows.)

* Open Terminal.app.
* Type "cd " (That's lowercase "c", then "d", then space), DON'T hit return.
* Now drag the *folder* that you found this ReadMe.txt file in to the Terminal.app window that has the "cd " you just typed. It should type in a path for you.
* Now hit return.

Either place the .elf file you want to program into that directory or remove the trailing version number from the elf file that is present; e.g. 
	mv TinyG2.elf.13.01 TinyG2.elf

Plug a USB cable from the computer to the Programming port of the Due (the one closest to the power jack). Make sure there are no shields, programmers or other devices plugged into the Due. The Due does not need external power for programming - it will be powered by the USB programming cable.

Copy and paste the following line into the Terminal window:
	./DueFromOSX.sh -f TinyG2.elf -p /dev/cu.usbmodem*

The output should look something like this (the numbers after cu.usbmodem will be different):

	Forcing reset using 1200bps open/close on port /dev/cu.usbmodem241311
	wait...
	Starting programming of file TinyG2.elf -> TinyG2.bin on port cu.usbmodem241311
	Erase flash
	Write 119380 bytes to flash
	[==============================] 100% (467/467 pages)
	Verify 119380 bytes of flash
	[==============================] 100% (467/467 pages)
	Verify successful
	Set boot flash true
	CPU reset.

Now disconnect the Due and plug the USB cable into the other port. You now have a Due/TinyG2.

-Synthetos Team