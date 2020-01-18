+================================================
+                                                
+ Vectric machine output configuration file for
+ g2core.
+                                                
+================================================
+                                                
+ History                                        
+                                                  
+ Who      When       What                         
+ ======== ========== ===========================
+ MattL    11/17/2019 Written from grbl_mm.pp
+                     added descriptive header
+ MattL    11/24/2019 Modified to use dynamic
+                     laser mode (T32 + M4)
+ ======== ========== ===========================

POST_NAME = "G2Core Laser Arcs (mm) (*.gcode)"
 
FILE_EXTENSION = "gcode"
 
UNITS = "MM"

+------------------------------------------------
+    Line terminating characters                 
+------------------------------------------------
 
LINE_ENDING = "[13][10]"
 
+------------------------------------------------
+    Block numbering                             
+------------------------------------------------
 
LINE_NUMBER_START     = 0
LINE_NUMBER_INCREMENT = 10
LINE_NUMBER_MAXIMUM = 999999
 
+================================================
+                                                
+    Formatting for variables                     
+                                                
+================================================
 
VAR LINE_NUMBER = [N|A|N|1.0]
VAR POWER = [P|C|S|1.0|10.0]
VAR SPINDLE_SPEED = [S|A|S|1.0]
VAR FEED_RATE = [F|C|F|1.1]
VAR X_POSITION = [X|C|X|1.3]
VAR Y_POSITION = [Y|C|Y|1.3]
VAR Z_POSITION = [Z|C|Z|1.3]
VAR ARC_CENTRE_I_INC_POSITION = [I|A|I|1.3]
VAR ARC_CENTRE_J_INC_POSITION = [J|A|J|1.3]
VAR X_HOME_POSITION = [XH|A|X|1.3]
VAR Y_HOME_POSITION = [YH|A|Y|1.3]
VAR Z_HOME_POSITION = [ZH|A|Z|1.3]
 
+================================================
+                                                
+    Block definitions for toolpath output       
+                                                
+================================================
 
+---------------------------------------------------
+  Commands output at the start of the file
+---------------------------------------------------
 
begin HEADER
 
"( [TP_FILENAME] )"
"( File created: [DATE] - [TIME])"
"( for g2core from Vectric )"
"( Material Size)"
"( X= [XLENGTH], Y= [YLENGTH], Z= [ZLENGTH])"
"([FILE_NOTES])"
"(Tools used in this file: )"
"([TOOLS_USED])"
"T32"
"M6"
"G17"
"G21"
"G90"
"S0M4"
"G0[ZH]"
"G0[XH][YH]"

+---------------------------------------------------
+  Commands output for rapid moves 
+---------------------------------------------------
 
begin RAPID_MOVE
 
"G0[X][Y][Z]"
 
 
+---------------------------------------------------
+  Commands output for the first feed rate move
+---------------------------------------------------
 
begin FIRST_FEED_MOVE
 
"G1[X][Y][Z][P][F]"
 
 
+---------------------------------------------------
+  Commands output for feed rate moves
+---------------------------------------------------
 
begin FEED_MOVE
 
"G1[X][Y][Z][P]"
 
 
+---------------------------------------------------
+  Commands output for the first clockwise arc move
+---------------------------------------------------

begin FIRST_CW_ARC_MOVE

"G2[X][Y][I][J][F][P]"
 
 
+---------------------------------------------------
+  Commands output for clockwise arc  move
+---------------------------------------------------
 
begin CW_ARC_MOVE
 
"G2[X][Y][I][J]"
 
 
+---------------------------------------------------
+  Commands output for the first counterclockwise arc move
+---------------------------------------------------
 
begin FIRST_CCW_ARC_MOVE
 
"G3[X][Y][I][J][F][P]"
 
 
+---------------------------------------------------
+  Commands output for counterclockwise arc  move
+---------------------------------------------------
 
begin CCW_ARC_MOVE
 
"G3[X][Y][I][J]"

+---------------------------------------------------
+  Commands output for Plunge Moves
+---------------------------------------------------
 
begin PLUNGE_MOVE
 
"G1[Z][F]"
"[S]"

+---------------------------------------------------
+  Commands output for Retract Moves
+---------------------------------------------------
 
begin RETRACT_MOVE
 
"S0"
"G0[Z]"

+---------------------------------------------------
+  Commands output at the end of the file
+---------------------------------------------------
 
begin FOOTER

"S0"
"M5"
"G0[ZH]"
"G0[XH][YH]"
"M2"
