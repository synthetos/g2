MKDIR = mkdir -p

TOOLS_SUBPATH = gcc-arm-none-eabi

TOOLS_VERSION_SHORT = 4.8
TOOLS_VERSION_LONG = 4.8-2014-q1-update
TOOLS_VERSION_FILENAME = 4_8-2014q1-20140314
TOOLS_EXPANDED_DIRNAME = gcc-arm-none-eabi-4_8-2014q1

TOOLS_URL_PREFIX = https://launchpad.net/gcc-arm-embedded/${TOOLS_VERSION_SHORT}/${TOOLS_VERSION_LONG}/+download/
TOOLS_OSX_TARBALL_FILENAME = gcc-arm-none-eabi-${TOOLS_VERSION_FILENAME}-mac.tar.bz2
TOOLS_LINUX_TARBALL_FILENAME = gcc-arm-none-eabi-${TOOLS_VERSION_FILENAME}-linux.tar.bz2
TOOLS_WIN32_ZIP_FILENAME = gcc-arm-none-eabi-${TOOLS_VERSION_FILENAME}-win32.zip

ifneq (,$(findstring /cygdrive/,$(PATH)))
OS := WIN32
else
ifneq (,$(findstring WINDOWS,$(PATH)))
OS := WIN32
else
UNAME := $(shell uname -s)

ifeq (Darwin,${UNAME})
OS = OSX
DOWNLOAD_TOOL = curl -i -L -O
else
ifeq (Linux,${UNAME})
OS = LINUX
DOWNLOAD_TOOL = wget
endif #LINUX
endif #Darwin

endif #cygdrive
endif #WINDOWS

all: ${TOOLS_SUBPATH}

.ONESHELL:
${TOOLS_SUBPATH}: ${TOOLS_EXPANDED_DIRNAME}
	ln -s "${TOOLS_EXPANDED_DIRNAME}" "${TOOLS_SUBPATH}"

${TOOLS_EXPANDED_DIRNAME}: ${TOOLS_${OS}_TARBALL_FILENAME}
	tar xjf "${TOOLS_${OS}_TARBALL_FILENAME}"

${TOOLS_${OS}_TARBALL_FILENAME}:
	${DOWNLOAD_TOOL} "${TOOLS_URL_PREFIX}${TOOLS_${OS}_TARBALL_FILENAME}"
	
.PHONY: clean

clean:
	rm -rf ${TOOLS_EXPANDED_DIRNAME}
	rm ${TOOLS_${OS}_TARBALL_FILENAME}
	rm ${TOOLS_SUBPATH}
