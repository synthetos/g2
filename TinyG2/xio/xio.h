/*
 * xio.h - eXtended IO devices - common header file
 * Part of Kinen project
 *
 * Copyright (c) 2010 - 2013 Alden S. Hart Jr.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY 
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
/* ---- General Notes and Project help ----
 *
 * XIO devices are compatible with avr-gcc stdio, Formatted printing is supported.
 *
 * Anything that includes xio.h first needs the following:
 * 	#include <stdio.h>				// needed for FILE def'n
 *	#include <stdbool.h>			// needed for true and false 
 *	#include <avr/pgmspace.h>		// defines prog_char, PSTR
 *
 * It must include the following libraries:
 * 	   libm.a
 * 	   libprintf_flt.a
 *
 * It must be linked with these options:
 * 	  -Wl,-u,vfprintf    (NOTE: Wl --->thats: W"lower-case ell" not W"the number one"
 *	  -lprintf_flt
 * 	  -lm
 *
 * ref: http://www.avrfreaks.net/index.php?name=PNphpBB2&file=printview&t=92299&start=0
 */
/* --- Memory Footprint Notes ---
 *
 *	The program memory footprint for the XIO system with USART and SPI drivers is just 
 *	under 3K bytes. 1.5K is the code itself and the rest is the stdlibs, mostly for 
 *	support for formatted print and floating point print. These numbers are using 
 *	Atmel Studio6 AVR_8_bit_GNU_Toolchain_3.4.0_663 (4.6.2) and stdlib 1.8 compiling
 *	to atmega328p with -Os. These are much more space efficient than older revs.
 *
 *	The RAM footprint is dominated by the RX/TX buffer sizes selected, but looks like 
 *	about 500 bytes (+/-) is sufficient. Note that each buffer can be sized independently 
 *	and does not need to be a binary multiple. Also note that the newer 
 *
 *	By way of comparison the grbl serial.c / print.c for the USART (no SPI) is about 1K of 
 *	program space. It does not use formatted printing. RAM is also dominated by buffer sizing.
 */
/* --- Circular Buffer Notes (RX/TX buffers) ---
 *
 * 	The circular buffers used by low-level character IO functions are an attempt 
 *	to optimize for execution speed. The circular buffers are unsigned char arrays 
 *	that fill down from the top element and wrap back to the top when index zero 
 *	is reached. This allows the assembly code to use pre-decrement operations, Z bit 
 *	tests, and eliminates modulus, masks, subtractions and other less efficient 
 *	bounds checking. 
 *
 *	Buffers are limited to 254 usable locations. One location is lost to read/write
 *	pointer collision detection and one is lost to the zero position
 *
 *	It is possible to use buffers > 254 bytes by setting buffer_t to uint16_t. 
 *	This supports buffers with a 16 bit index at some penalty to performance.
 *
 *	See xio_read_buffer() and xio_read_buffer() for functionality
 */
/* --- What's the the int characters? ---
 *	Single characters returned from buffer queues are treated as ints in order to 
 *	ease compatibility with stdio. This is a bit of a pain but is necessary to 
 *	support _FDEV_ERR (which is -1) to be cascaded in returns to stdio functions.
 *	Not using ints in the right place is a bug generator if messing with this code.
 */
/*
 * --- Including devices or removing unused devices ---
 * I'd like for this to be cleaner, but for now...
 * To remove a device do the following
 *	- comment out the entry in the xioDev enum
 *	- comment out the device include file in this file
 *	- comment out the device inits and opens in xio_init()
 *	- remove the .c and .h files form the project
 */
#ifndef xio_h
#define xio_h

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

//#include "../system.h"

#define _FDEV_ERR -1
#define _FDEV_EOF -2

// see other xio_.h includes below the structures and typedefs

/*************************************************************************
 *	Device configurations
 *************************************************************************/
// Pre-allocated XIO devices (configured devices)
// Unused devices are commented out. All this needs to line up.

enum xioDev {			// TYPE:	DEVICE:
	XIO_DEV_USART = 0,	// USART	USART device
	XIO_DEV_SPI,		// SPI		SPI device
	XIO_DEV_PGM,		// FILE		Program memory files
	XIO_DEV_COUNT		// total device count (must be last entry)
};

/******************************************************************************
 * Device structures
 *
 * Each device has one or 2 structs. 1. The device struct declared below.
 * It embeds a stdio stream struct "FILE". The FILE struct uses the udata
 * field to back-reference the generic struct so getc & putc can get at it.
 * Optionally a device may have an 'x' struct which contains extended data 
 * specific to that device or device type
 *
 * NOTE" "FILE *" is another way of saying "struct __file *"
 *
 ******************************************************************************/

#define flags_t uint16_t
#define buffer_t uint8_t					// fast, but limits buffer to 255 char max

typedef struct xioBuffer {
	buffer_t size;							// buffer size -1 (for wrapping)
	volatile buffer_t rd;					// read index
	volatile buffer_t wr;					// write index
	char buf[];								// array size set by device RX/TX definitions
} xioBuf_t;

typedef struct xioDEVICE {					// common device struct (one per dev)
	uint8_t dev;							// self referential device number
	FILE *(*x_open)(const uint8_t dev, const char *addr, const flags_t flags);
//	int (*x_close)(struct xioDEVICE *d);
	int (*x_ctrl)(struct xioDEVICE *d, const flags_t flags);
	int (*x_gets)(struct xioDEVICE *d, char *buf, const int size);	// non-blocking line reader
	int (*x_getc)(FILE *);					// read char (stdio compatible)
	int (*x_putc)(char, FILE *);			// write char (stdio compatible)
	void (*x_flow)(struct xioDEVICE *d);	// flow control callback function
	xioBuf_t *rx;							// RX buffer struct binding
	xioBuf_t *tx;							// TX buffer struct binding
	void *x;								// extended device struct binding
	FILE stream;							// stdio stream structure

	// device flags (some are not used in the 328 implementation)
	uint8_t flag_block;
	uint8_t flag_echo;
	uint8_t flag_linemode;
	uint8_t flag_in_line;					// used as a state variable for line reads
	uint8_t flag_eol;						// end of line (message) detected
	uint8_t flag_eof;						// end of file detected

	// gets() working data
	int size;								// text buffer length (dynamic)
	uint8_t len;							// chars read so far (buf array index)
	char *buf;								// text buffer binding (can be dynamic)	
} xioDev_t;

typedef FILE *(*x_open_t)(const uint8_t dev, const char *addr, const flags_t flags);
typedef int (*x_close_t)(xioDev_t *d);
typedef int (*x_ctrl_t)(xioDev_t *d, const flags_t flags);
typedef int (*x_gets_t)(xioDev_t *d, char *buf, const int size);
typedef int (*x_getc_t)(FILE *);
typedef int (*x_putc_t)(char, FILE *);
typedef void (*x_flow_t)(xioDev_t *d);

/*******************************************************************************
 *	Sub-Includes and static allocations
 *******************************************************************************/
// all sub-includes here so only xio.h is needed externally
#include "xio_spi.h"
#include "xio_usart.h"
#include "xio_file.h"

xioDev_t *ds[XIO_DEV_COUNT];			// array of device structure pointers 
extern struct controllerSingleton tg;	// needed by init for default source

/*******************************************************************************
 *	Function Prototypes
 *******************************************************************************/

// public functions (virtual class) 
FILE *xio_open(const uint8_t dev, const char *addr, const flags_t flags);
int xio_ctrl(const uint8_t dev, const flags_t flags);
int xio_gets(const uint8_t dev, char *buf, const int size);
int xio_getc(const uint8_t dev);
int xio_putc(const uint8_t dev, const char c);
int xio_set_baud(const uint8_t dev, const uint8_t baud_rate);
void xio_set_stdin(const uint8_t dev);
void xio_set_stdout(const uint8_t dev);
void xio_set_stderr(const uint8_t dev);

// private functions (excuse me sir, this is a private function)
void xio_init(void);
void xio_reset_device(xioDev_t *d, const flags_t flags);
int xio_ctrl_device(xioDev_t *d, const flags_t flags);
void xio_null(xioDev_t *d);				// NULL callback (used for flow control)

int xio_getc_device(FILE *stream);
int xio_putc_device(const char c, FILE *stream);
int xio_gets_device(xioDev_t *d, char *buf, const int size);

//int xio_read_buffer(xioBuf_t *b);
//int xio_write_buffer(xioBuf_t *b, char c);
int8_t xio_read_buffer(xioBuf_t *b);
int8_t xio_write_buffer(xioBuf_t *b, char c);
void xio_queue_RX_string(const uint8_t dev, const char *buf);

/*************************************************************************
 * SUPPORTING DEFINITIONS - SHOULD NOT NEED TO CHANGE
 *************************************************************************/

/* XIO return codes
 * These codes are the "inner nest" for the TG_ return codes. 
 * The first N TG codes correspond directly to these codes.
 * This eases using XIO by itself (without tinyg) and simplifes using
 * tinyg codes with no mapping when used together. This comes at the cost 
 * of making sure these lists are aligned. TG_should be based on this list.
 */
enum xioCodes {
	XIO_OK = 0,				// OK - ALWAYS ZERO
	XIO_ERR,				// generic error return (errors start here)
	XIO_EAGAIN,				// function would block here (must be called again)
	XIO_NOOP,				// function had no-operation	
	XIO_COMPLETE,			// operation complete
	XIO_TERMINATE,			// operation terminated (gracefully)
	XIO_RESET,				// operation reset (ungraceful)
	XIO_EOL,				// function returned end-of-line
	XIO_EOF,				// function returned end-of-file 
	XIO_FILE_NOT_OPEN,		// file is not open
	XIO_FILE_SIZE_EXCEEDED, // maximum file size exceeded
	XIO_NO_SUCH_DEVICE,		// illegal or unavailable device
	XIO_BUFFER_EMPTY,		// more of a statement of fact than an error code
	XIO_BUFFER_FULL,
	XIO_BUFFER_FULL_FATAL,
	XIO_INITIALIZING,		// system initializing, not ready for use
	XIO_ERROR_16,			// reserved
	XIO_ERROR_17,			// reserved
	XIO_ERROR_18,			// reserved
	XIO_ERROR_19			// NOTE: XIO codes align to here
};
#define XIO_ERRNO_MAX XIO_BUFFER_FULL_NON_FATAL

/*
 * xio control flag values
 *
 * if using 32 bits must cast 1 to uint32_t for bit evaluations to work correctly
 * #define XIO_BLOCK	((uint32_t)1<<1)		// 32 bit example. Change flags_t to uint32_t
 */
#define XIO_BLOCK		((uint16_t)1<<0)		// enable blocking reads
#define XIO_NOBLOCK		((uint16_t)1<<1)		// disable blocking reads
#define XIO_XOFF 		((uint16_t)1<<2)		// enable XON/OFF flow control
#define XIO_NOXOFF 		((uint16_t)1<<3)		// disable XON/XOFF flow control
#define XIO_ECHO		((uint16_t)1<<4)		// echo reads from device to stdio
#define XIO_NOECHO		((uint16_t)1<<5)		// disable echo
#define XIO_CRLF		((uint16_t)1<<6)		// convert <LF> to <CR><LF> on writes
#define XIO_NOCRLF		((uint16_t)1<<7)		// do not convert <LF> to <CR><LF> on writes
#define XIO_IGNORECR	((uint16_t)1<<8)		// ignore <CR> on reads
#define XIO_NOIGNORECR	((uint16_t)1<<9)		// don't ignore <CR> on reads
#define XIO_IGNORELF	((uint16_t)1<<10)		// ignore <LF> on reads
#define XIO_NOIGNORELF	((uint16_t)1<<11)		// don't ignore <LF> on reads
#define XIO_LINEMODE	((uint16_t)1<<12)		// special <CR><LF> read handling
#define XIO_NOLINEMODE	((uint16_t)1<<13)		// no special <CR><LF> read handling


/* Some useful ASCII definitions */

#define NUL (char)0x00		//  ASCII NUL char (0) (not "NULL" which is a pointer)
#define STX (char)0x02		// ^b - STX
#define ETX (char)0x03		// ^c - ETX
#define ENQ (char)0x05		// ^e - ENQuire
#define BEL (char)0x07		// ^g - BEL
#define BS  (char)0x08		// ^h - backspace 
#define TAB (char)0x09		// ^i - character
#define LF	(char)0x0A		// ^j - line feed
#define VT	(char)0x0B		// ^k - kill stop
#define CR	(char)0x0D		// ^m - carriage return
#define XON (char)0x11		// ^q - DC1, XON, resume
#define XOFF (char)0x13		// ^s - DC3, XOFF, pause
#define NAK (char)0x15		// ^u - Negative acknowledgement
#define CAN (char)0x18		// ^x - Cancel, abort
#define ESC (char)0x1B		// ^[ - ESC(ape)
#define DEL (char)0x7F		//  DEL(ete)
#define Q_EMPTY (char)0xFF	// signal no character

/******************************************************************************
 * SETUP XIO UNIT TESTS
 ******************************************************************************/

#define __XIO_UNIT_TESTS	// uncomment this to compile nd run XIO unit tests
#ifdef __XIO_UNIT_TESTS
void xio_unit_tests(void);
#define	XIO_UNIT_TESTS xio_unit_tests();
#else
#define	XIO_UNIT_TESTS
#endif // __XIO_UNIT_TESTS

#endif // xio_h
