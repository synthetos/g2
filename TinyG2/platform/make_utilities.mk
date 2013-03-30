# CREATE_DEVICE_LIBRARY
# Create the rules needed for a device specific library, and add
# it's objects to ALL_*_OBJECTS, and DEVICE_OBJECTS_* as necessary.
# setup: XXX_SOURCE_DIRS list
# call: with the same XXX and the resulting LIBNAME (without the .a)
# $(eval $(call CREATE_DEVICE_LIBRARY,XXX,LIBNAME))

# Uses CREATE_DEVICE_LIBRARY_each as a helper

define CREATE_DEVICE_LIBRARY_each
OUTDIR = $(OBJ)/$(1)

$(2)_C_OBJECTS_$(1)   := $(addprefix $$(OUTDIR)/,$($(2)_C_OBJECTS))
$(2)_CXX_OBJECTS_$(1) := $(addprefix $$(OUTDIR)/,$($(2)_CXX_OBJECTS))

ALL_C_OBJECTS_$(1)   += $$($(2)_C_OBJECTS_$(1))
ALL_CXX_OBJECTS_$(1) += $$($(2)_CXX_OBJECTS_$(1))

DEVICE_OBJECTS_$(1) = $$(OUTDIR)/$(3).a

$$(OUTDIR)/$(3).a: $$(OUTDIR)/$(3).a($$($(2)_C_OBJECTS_$(1)) $$($(2)_CXX_OBJECTS_$(1)) )
endef

define CREATE_DEVICE_LIBRARY
$(1)_C_SOURCES   = $$(foreach dir,$$($(1)_SOURCE_DIRS), $$(wildcard $$(dir)/*.c) )
$(1)_CXX_SOURCES = $$(foreach dir,$$($(1)_SOURCE_DIRS), $$(wildcard $$(dir)/*.cpp) )

$(1)_C_OBJECTS   := $$(addsuffix .o,$$(basename $$($(1)_C_SOURCES)))
$(1)_CXX_OBJECTS := $$(addsuffix .o,$$(basename $$($(1)_CXX_SOURCES)))

$$(foreach MEMORY, $$(MEMORIES), $$(eval $$(call CREATE_DEVICE_LIBRARY_each,$$(MEMORY),$(1),$(2),$(3))))
endef

