/*
 * xio.c - eXtended IO devices - common code file
 * Part of Kinen project
 *
 * Copyright (c) 2010 - 2013 Alden S. Hart Jr.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
 * BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
/* ----- XIO - eXtended Device IO System ----
 *
 * XIO provides common access to native and derived IO devices
 * XIO devices are compatible with avr-gcc stdio and also provide some special 
 * functions that are not found in stdio.
 *
 * Stdio support:
 *	- http://www.nongnu.org/avr-libc/user-manual/group__avr__stdio.html
 * 	- Stdio compatible putc() and getc() functions provided for each device
 *	    This enables fgets, printf, scanf, and other stdio functions
 * 	- Full support for formatted printing is provided (including floats)
 * 	- Assignment of a default device(s) to stdin, stdout & stderr.
 *	    printf() and printf_P() send to stdout, so use fprintf() to stderr
 *		for things that should't go over RS485 in SLAVE mode 
 *
 * Facilities provided beyond stdio:
 *	- Supported devices include:
 *		- USB (derived from USART)
 *		- RS485 (derived from USART)
 *		- SPI devices and slave channels
 *		- Program memory "files" (read only)
 *	- Stdio FILE streams are managed as bindings to the above devices
 *	- Functions provided include:
 *		- open() - initialize parameters, addresses and flags
 *		- gets() - non-blocking input line reader - extends fgets
 *		- ctrl() - ioctl-like knockoff for setting device parameters (flags)
 *		- signal handling: interrupt on: feedhold, cycle_start, ctrl-x software reset
 *		- interrupt buffered RX and TX functions 
 *		- XON/XOFF software flow control
 */
/* ----- XIO - Some Internals ----
 *
 * XIO layers are: (1) xio virtual device (root), (2) xio device type, (3) xio devices
 *
 * The virtual device has the following methods:
 *	xio_init() - initialize the entire xio system
 *	xio_open() - open a device indicated by the XIO_DEV number
 *	xio_ctrl() - set control flags for XIO_DEV device
 *	xio_gets() - get a string from the XIO_DEV device (non blocking line reader)
 *	xio_getc() - read a character from the XIO_DEV device (not stdio compatible)
 *	xio_putc() - write a character to the XIO_DEV device (not stdio compatible)
 *  xio_set_baud() - set baud rates for devices for which this is meaningful
 *
 * The device type layer currently knows about USARTS, SPI, and File devices. Methods are:
 *	xio_init_<type>() - initializes the devices of that type
 *
 * The device layer currently supports: USB, RS485, SPI channels, PGM file reading. methods:
 *	xio_open_<device>() - set up the device for use or reset the device
 *	xio_ctrl_<device>() - change device flag controls
 *	xio_gets_<device>() - get a string from the device (non-blocking)
 *	xio_getc_<device>() - read a character from the device (stdio compatible)
 *	xio_putc_<device>() - write a character to the device (stdio compatible)
 *
 * The virtual level uses XIO_DEV_xxx numeric device IDs for reference. 
 * Lower layers are called using the device structure pointer xioDev *d
 * The stdio compatible functions use pointers to the stdio FILE structs.
 */

//#include "stdbool.h"
//#include <stdio.h>					// precursor for xio.h
#include "xio.h"					// all device sub-system includes are nested here

/***********************************************************************************
 * PUBLIC ENTRY POINTS - access functions via the XIO_DEV device number
 * xio_open() 		- open device for use or re-use
 * xio_gets() 		- non-blocking get line function
 * xio_getc() 		- getc (not stdio compatible)
 * xio_putc() 		- putc (not stdio compatible)
 * xio_ctrl() 		- set control flags (top-level XIO_DEV access)
 * xio_set_baud() 	- set baud rate (currently this only works on USART devices)
 * xio_set_stdin()  - set stdin from device number
 * xio_set_stdout() - set stdout from device number
 * xio_set_stderr() - set stderr from device number
 *
 * It might be prudent to run an assertion like below, but we trust the callers:
 * 	if (dev < XIO_DEV_COUNT) blah blah blah
 *	else  return (_FDEV_ERR);	// XIO_NO_SUCH_DEVICE
 */
FILE *xio_open(uint8_t dev, const char *addr, flags_t flags) { return (ds[dev]->x_open(dev, addr, flags));}
int xio_gets(const uint8_t dev, char *buf, const int size) { return (ds[dev]->x_gets(ds[dev], buf, size));}
int xio_getc(const uint8_t dev) { return (ds[dev]->x_getc(&(ds[dev]->stream)));}
int xio_putc(const uint8_t dev, const char c) { return (ds[dev]->x_putc(c, &(ds[dev]->stream)));}
int xio_ctrl(const uint8_t dev, const flags_t flags) { return (xio_ctrl_device(ds[dev], flags));}
int xio_set_baud(const uint8_t dev, const uint8_t baud) { xio_set_baud_usart(ds[dev], baud); return (XIO_OK);}
void xio_set_stdin(const uint8_t dev)  { stdin  = &(ds[dev]->stream);}
void xio_set_stdout(const uint8_t dev) { stdout = &(ds[dev]->stream);}
void xio_set_stderr(const uint8_t dev) { stderr = &(ds[dev]->stream);}

/***********************************************************************************
 * xio_init() 			- initialize entire xio sub-system
 * xio_reset_device()	- common function used by opens()
 * xio_ctrl_device() 	- set control-flags
 * xio_null() 			- xio null function
 */
void xio_init()
{
	// run device constructors and register devices in dev array
	ds[XIO_DEV_USART] = xio_init_usart(XIO_DEV_USART);
	ds[XIO_DEV_SPI]   = xio_init_spi(XIO_DEV_SPI);
//	ds[XIO_DEV_PGM]   = xio_init_file(XIO_DEV_PGM);

	// open individual devices (file device opens occur at time-of-use)
	xio_open(XIO_DEV_USART, NULL, USART_XIO_FLAGS);
	xio_open(XIO_DEV_SPI, NULL, SPI_XIO_FLAGS);

	// setup std devices for printf/fprintf to work
	xio_set_stdin(XIO_DEV_USART);
	xio_set_stdout(XIO_DEV_USART);
	xio_set_stderr(XIO_DEV_SPI);
}

void xio_reset_device(xioDev_t *d,  const flags_t flags)
{
	if (d->rx != NULL) {			// don't write on a wild pointer
		d->rx->wr = 1;				// can't use location 0 in circular buffer
		d->rx->rd = 1;
	}
	if (d->tx != NULL) {
		d->tx->wr = 1;
		d->tx->rd = 1;
	}
	d->flag_in_line = 0;			// reset the working flags
	d->flag_eol = 0;
	d->flag_eof = 0;

	xio_ctrl_device(d, flags);		// setup control flags

	// setup stdio stream structure
	fdev_setup_stream(&d->stream, d->x_putc, d->x_getc, _FDEV_SETUP_RW);
	fdev_set_udata(&d->stream, d);	// reference yourself for udata 
}

void xio_null(xioDev_t *d) { return;}

#define SETFLAG(t,f) if ((flags & t) != 0) { d->f = true; }
#define CLRFLAG(t,f) if ((flags & t) != 0) { d->f = false; }

int xio_ctrl_device(xioDev_t *d, const flags_t flags)
{
	SETFLAG(XIO_BLOCK,		flag_block);
	CLRFLAG(XIO_NOBLOCK,	flag_block);

	SETFLAG(XIO_ECHO,		flag_echo);
	CLRFLAG(XIO_NOECHO,		flag_echo);

	SETFLAG(XIO_LINEMODE,	flag_linemode);
	CLRFLAG(XIO_NOLINEMODE,	flag_linemode);

//	SETFLAG(XIO_XOFF,		flag_xoff);
//	CLRFLAG(XIO_NOXOFF,		flag_xoff);
//	SETFLAG(XIO_CRLF,		flag_crlf);
//	CLRFLAG(XIO_NOCRLF,		flag_crlf);
//	SETFLAG(XIO_IGNORECR,	flag_ignorecr);
//	CLRFLAG(XIO_NOIGNORECR,	flag_ignorecr);
//	SETFLAG(XIO_IGNORELF,	flag_ignorelf);
//	CLRFLAG(XIO_NOIGNORELF,	flag_ignorelf);
	return (XIO_OK);
}

/* 
 * Generic getc() putc() - these are typically subclassed at the type level
 *	xio_getc_device() - get a character from the device 
 *	xio_putc_device() - write a character to the device 
 */
int xio_getc_device(FILE *stream)
{
	return (xio_read_buffer(((xioDev_t *)stream->udata)->rx));
}

int xio_putc_device(const char c, FILE *stream)
{
	return (xio_write_buffer(((xioDev_t *)stream->udata)->tx, c));
}

/* 
 *	xio_gets_device() - read a complete line (message) from a device
 *
 *	Reads from the local RX buffer until it's empty. Retains line context 
 *	across calls - so it can be called multiple times. Reads as many characters 
 *	as it can until any of the following is true:
 *
 *	  - Encounters newline indicating a complete line. Terminate the buffer
 *		but do not write the newline into the buffer. Reset line flag. Return XIO_OK.
 *
 *	  - Encounters an empty buffer. Leave in_line. Return XIO_EAGAIN.
 *
 *	  - A successful read would cause output buffer overflow. Return XIO_BUFFER_FULL
 *
 *	Note: LINEMODE flag in device struct is ignored. It's ALWAYS LINEMODE here.
 *	Note: CRs are not recognized as NL chars - master must send LF to terminate a line
 */
int xio_gets_device(xioDev_t *d, char *buf, const int size)
{
	int c_out;

	// first time thru initializations
	if (d->flag_in_line == false) {
		d->flag_in_line = true;					// yes, we are busy getting a line
		d->buf = buf;							// bind the output buffer
		d->len = 0;								// zero the buffer count
		d->size = size;							// set the max size of the message
	}
	while (true) {
		if (d->len >= (d->size)-1) {			// size is total count - aka 'num' in fgets()
			d->buf[d->size] = NUL;				// string termination preserves latest char
			return (XIO_BUFFER_FULL);
		}
		if ((c_out = xio_read_buffer(d->rx)) == _FDEV_ERR) { return (XIO_EAGAIN);}
		if (c_out == LF) {
//			d->buf[(d->len)++] = LF;			// ++++++++++++++++ for diagnostics only
			d->buf[(d->len)++] = NUL;
			d->flag_in_line = false;			// clear in-line state (reset)
			return (XIO_OK);					// return for end-of-line
		}
		d->buf[d->len++] = c_out;				// write character to buffer
	}
}

/* 
 * Buffer read and write primitives
 *
 * 	You can make these blocking routines by calling them in a while(true)
 * 	while waiting for something other than _FDEV_ERR to be returned. 
 *	Of course, this only works if some interrupt is loading things behind
 *	the scenes.
 */
//int xio_read_buffer(xioBuf_t *b) 
int8_t xio_read_buffer(xioBuf_t *b) 
{
	if (b->wr == b->rd) { return (_FDEV_ERR);}	// return if queue empty
	if ((--(b->rd)) == 0) { b->rd = b->size;}	// advance tail with wrap
	return (b->buf[b->rd]);						// return character from buffer
}												// leave rd on *returned* char

//int xio_write_buffer(xioBuf_t *b, char c) 
int8_t xio_write_buffer(xioBuf_t *b, char c) 
{
	buffer_t next_wr = b->wr-1;					// pre-advance wr to temporary variable
	if (next_wr == 0) { next_wr = b->size;}		// advance wr with wrap
	if (next_wr == b->rd) { return (_FDEV_ERR);}// return if queue full
	b->buf[next_wr] = c;						// write char to buffer
	b->wr = next_wr;							// advance wr from temp value
	return (XIO_OK);							// leave wr on *written* char
}

/*
 *	xio_queue_RX_string() - put a string in an RX buffer
 *	String must be NUL terminated but doesn't require a CR or LF
 */
void xio_queue_RX_string(const uint8_t dev, const char *buf)
{
	uint8_t i=0;
	while (buf[i] != NUL) {
		xio_write_buffer(ds[dev]->rx, buf[i++]);
	}
}

/******************************************************************************
 * XIO UNIT TESTS
 ******************************************************************************/

#ifdef __XIO_UNIT_TESTS
static void _transmit_test(uint8_t dev);
//static void _loopback_test(uint8_t dev);
//static void _loopfake_test(uint8_t dev);
static void _message_test(uint8_t dev);
//static void _pgm_read_test();

int c;
uint8_t status;
char buffer[64];
char sequence[8] = {"01234567"};

void xio_unit_tests()
{
//	_transmit_test(XIO_DEV_USART);			// never returns
//	_loopback_test(XIO_DEV_USART);			// never returns
//	_loopfake_test(XIO_DEV_USART);			// never returns

//	_loopback_test(XIO_DEV_SPI);			// never returns
	_message_test(XIO_DEV_SPI);				// never returns
}

static void _transmit_test(uint8_t dev)		// never returns
{
//	while (true) { xio_putc(dev, '5');}

	c = xio_getc(dev);

	uint8_t i = 0;
	while (true) {
		if (xio_putc(dev, sequence[i]) != _FDEV_ERR) {
			i = ((i+1) & 0x07);						
		}
	}
}

static void _message_test(uint8_t dev)		// never returns
{
	while (true) {
		if ((status = xio_gets(dev, buffer, sizeof(buffer))) == XIO_OK) {
			printf("%s",buffer);
		}
	}
}
/*

static void _loopback_test(uint8_t dev)		// never returns
{
	while (true) {
		c = xio_getc(dev);
		if (c != (char)_FDEV_ERR) {
			xio_putc(dev, c);
		}
	}
}

static void _loopfake_test(uint8_t dev)		// never returns
{
	uint8_t i=0;

	while (true) {
		xio_write_buffer(ds[dev]->rx, sequence[i]);
		i = ((i+1) & 0x07);						
		c = xio_getc(dev);
//		c = '5';
		if (c != (char)_FDEV_ERR) {
			xio_putc(dev, c);
		}
	}
}
*/
/*
void _pgm_read_test()
{	
	FILE *fdev;

	fdev = xio_open(XIO_DEV_PGM, 0, PGM_FLAGS);
	xio_puts_pgm("ABCDEFGHIJKLMNOP\n", fdev);
	xio_putc_pgm('A', fdev);
	xio_putc_pgm('B', fdev);
	xio_putc_pgm('C', fdev);
	xio_getc_pgm(fdev);
	xio_getc_pgm(fdev);
	xio_getc_pgm(fdev);
}
*/
#endif // __UNIT_TESTS

