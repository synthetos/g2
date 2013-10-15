## In order to download and decompress the reference coded *useful* for symbolic debugging,
## Just call "make" from this directory.

# Program paths
WGET=curl -O
TAR=tar xzvf
TOUCH=touch

# Newlib locations
NEWLIB_VERSION=1.20.0
NEWLIB_URL=ftp://sources.redhat.com/pub/newlib/newlib-$(NEWLIB_VERSION).tar.gz
NEWLIB_DIRECTORY=./newlib-$(NEWLIB_VERSION)/


# Rules
all: $(NEWLIB_DIRECTORY)

$(NEWLIB_DIRECTORY): | newlib-$(NEWLIB_VERSION).tar.gz
	$(TAR) $|

newlib-$(NEWLIB_VERSION).tar.gz:
	$(WGET) $(NEWLIB_URL)