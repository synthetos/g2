MKDIR = mkdir -p

ARCH ?= gcc-arm-none-eabi

ifeq ($(ARCH), gcc-arm-none-eabi)
TOOLS_SUBPATH = gcc-arm-none-eabi

TOOLS_VERSION_SHORT = 4.9
TOOLS_VERSION_LONG = 4.9-2014-q4-major
TOOLS_VERSION_FILENAME = 4_9-2014q4-20141203
TOOLS_EXPANDED_DIRNAME = gcc-arm-none-eabi-4_9-2014q4

#https://launchpad.net/gcc-arm-embedded/4.9/4.9-2014-q4-major/+download/gcc-arm-none-eabi-4_9-2014q4-20141203-linux.tar.bz2
# I had to install the following to get this to run
# sudo apt-get install python-serial python-argparse openocd \
#    flex bison libncurses5-dev autoconf texinfo build-essential \
#    libftdi-dev libtool zlib1g-dev genromfs git-core wget zip
# sudo apt-get install libc6:i386 libgcc1:i386 gcc-4.6-base:i386 libstdc++5:i386 libstdc++6:i386

TOOLS_URL_PREFIX = https://launchpad.net/gcc-arm-embedded/${TOOLS_VERSION_SHORT}/${TOOLS_VERSION_LONG}/+download/
TOOLS_OSX_ARCHIVE_FILENAME = gcc-arm-none-eabi-${TOOLS_VERSION_FILENAME}-mac.tar.bz2
TOOLS_LINUX_ARCHIVE_FILENAME = gcc-arm-none-eabi-${TOOLS_VERSION_FILENAME}-linux.tar.bz2
TOOLS_WIN32_ARCHIVE_FILENAME = gcc-arm-none-eabi-${TOOLS_VERSION_FILENAME}-win32.zip

ifneq (,$(findstring /cygdrive/,$(PATH)))
OS := WIN32
else
ifneq (,$(findstring WINDOWS,$(PATH)))
OS := WIN32
else
UNAME := $(shell uname -s)

ifeq (Darwin,${UNAME})
OS = OSX
DOWNLOAD_TOOL = curl -OL
else
ifeq (Linux,${UNAME})
OS = LINUX
DOWNLOAD_TOOL = wget --no-check-certificate
endif #LINUX
endif #Darwin

endif #cygdrive
endif #WINDOWS

TEST_COMMAND=arm-none-eabi-gcc

endif #eq ($(ARCH), 'gcc-arm-none-eabi')


ifeq ($(shell command -v $(TEST_COMMAND) >/dev/null 2>&1 || echo "yes"), yes)
TOOLS_SUBPATH_IF_NEEDED = $(TOOLS_SUBPATH)
endif


all: TOOLS_INSTALLED

.PHONY:
TOOLS_INSTALLED: ${TOOLS_SUBPATH_IF_NEEDED}


.ONESHELL:
${TOOLS_SUBPATH}: ${TOOLS_EXPANDED_DIRNAME}
	ln -s "${TOOLS_EXPANDED_DIRNAME}" "${TOOLS_SUBPATH}"

${TOOLS_EXPANDED_DIRNAME}: ${TOOLS_${OS}_ARCHIVE_FILENAME}
	tar xjf "${TOOLS_${OS}_ARCHIVE_FILENAME}"

${TOOLS_${OS}_ARCHIVE_FILENAME}:
	$(info The $(ARCH) tools need installed. Downloading and installing now.)
	${DOWNLOAD_TOOL} "${TOOLS_URL_PREFIX}${TOOLS_${OS}_ARCHIVE_FILENAME}"

.PHONY: clean

clean:
	rm -rf ${TOOLS_EXPANDED_DIRNAME}
	rm ${TOOLS_${OS}_ARCHIVE_FILENAME}
	rm ${TOOLS_SUBPATH}
