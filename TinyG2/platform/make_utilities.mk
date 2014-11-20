# CREATE_DEVICE_LIBRARY
# Create the rules needed for a device specific library, and add
# it's objects to ALL_*_OBJECTS, and DEVICE_OBJECTS_* as necessary.
# setup: XXX_SOURCE_DIRS list
# call: with the same XXX and the resulting LIBNAME (without the .a)
# $(eval $(call CREATE_DEVICE_LIBRARY,XXX,LIBNAME))


define CREATE_DEVICE_LIBRARY
$(1)_C_SOURCES   := $(foreach dir,$($(1)_SOURCE_DIRS), $(wildcard $(dir)/*.c) )
$(1)_CXX_SOURCES := $(foreach dir,$($(1)_SOURCE_DIRS), $(wildcard $(dir)/*.cpp) )
$(1)_ASM_SOURCES := $(foreach dir,$($(1)_SOURCE_DIRS), $(wildcard $(dir)/*.s) )

$(1)_C_OBJECTS   := $$(addsuffix .o,$$(basename $$($(1)_C_SOURCES)))
$(1)_CXX_OBJECTS := $$(addsuffix .o,$$(basename $$($(1)_CXX_SOURCES)))
$(1)_ASM_OBJECTS := $$(addsuffix .o,$$(basename $$($(1)_ASM_SOURCES)))

C_OBJECTS   += $$($(1)_C_OBJECTS)
CXX_OBJECTS += $$($(1)_CXX_OBJECTS)
ASM_OBJECTS += $$($(1)_ASM_OBJECTS)
endef

