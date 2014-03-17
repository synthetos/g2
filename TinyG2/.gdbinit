# Setup for non-wrapped lines and non-pages prints
set width 0
set height 0

# Connect to the remote device
echo \nConnecting to JLinkGDBServer on port 2331.\n
echo (You mush have JLinkGDBServer running.)\n\n
target remote localhost:2331

# Configure JLinkGDBServer
monitor flash device = ATSAM3X8C
monitor speed auto
monitor endian little

# Turn on history saving
set history save on
