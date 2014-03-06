# Setup for non-wrapped lines and non-pages prints
set width 0
set height 0

# Connect to the remote device
echo \nConnecting to JLinkGDBServer on port 2331.\n
echo (You mush have JLinkGDBServer runing.)\n\n
target remote localhost:2331

# Turn on history saving
set history save on
