# Setup for non-wrapped lines and non-pages prints
set width 0
set height 0

# Turn on history saving
set history save on
set history expansion on

set print pretty on

monitor adapter_khz 5000

define reset
  boot_from_flash
  monitor reset init
end

define flash
  make
  load
  reset
end

define dump_mb
  source ../Resources/debug/mb.gdb
end
