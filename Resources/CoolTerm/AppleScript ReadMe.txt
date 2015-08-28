LoadSetting(FilePath as String) as Boolean
------------------------------------------
Loads the connection setttings file with the specified path. FilePath can be a relative or absolute path.
Returns TRUE is the connection settings file was loaded successfully. The window name of the new terminal window will be identical to the file name of the settings file.


CloseWindow(TerminalWindowName as string)
-----------------------------------------
Closes the terminal window with the specified name.


Connect(TerminalWindowName as String) as Boolean
------------------------------------------------
Opens the serial port associtated with the terminal window with the specified name.
Returns TRUE if the port was successfully opened.


Disconnect(TerminalWindowName as String)
----------------------------------------
Closes the serial port associtated with the terminal window with the specified name.


Write(TerminalWindowName as String, Data as String)
---------------------------------------------------
Writes data to the serial port associtated with the terminal window with the specified name.


WriteLine(TerminalWindowName as String, Data as String)
-------------------------------------------------------
Writes data terminated by the "Enter Key Emulation" character specified in the connection settings to the serial port associtated with the terminal window with the specified name.


Poll(TerminalWindowName as String)
----------------------------------
Polls the serial port associtated with the terminal window with the specified name and causes all data currently available in the serial port receive buffer to be transferred to CoolTerm's receive buffer. It is recommended to call this method before calling Read, ReadAll, LookAhead, and BytesAvailable.


Read(TerminalWindowName as String, NumChars as Integer) as String
-----------------------------------------------------------------
Reads and removes the specified number of characters from the receive buffer of the terminal window with the specified name.


ReadAll(TerminalWindowName as string) as String
-----------------------------------------------
Reads and removes all characters from the receive buffer of the terminal window with the specified name.


BytesAvailable(TerminalWindowName as string) as Integer
-------------------------------------------------------
Returns the number of characters currently available in the receive buffer of the terminal window with the specified name.


LookAhead(TerminalWindowName as string) as String
-------------------------------------------------
Returns the contents of the receive buffer of the terminal window with the specified name without removing any data.


ClearBuffer(TerminalWindowName as string)
-----------------------------------------
Clears receive buffer of the terminal window with the specified name.


Hex2Str(HexStr as String) as String
-----------------------------------
Converts a hexadecimal string to a plain string that can be sent to a serial port.


Str2Hex(PlainStr as String) as String
-------------------------------------
Converts a plain string to a hexadecimal string that can be used to display non-printable characters received from a serial port.