/*
 *  xio_file.c	- device driver for program memory "files"
 * 				- works with avr-gcc stdio library
 *
 * Part of Kinen project
 *
 * Copyright (c) 2011 - 2013 Alden S. Hart Jr.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY 
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <stdio.h>				// precursor for xio.h
#include <stdbool.h>			// true and false
#include <avr/pgmspace.h>		// precursor for xio.h
#include "xio.h"				// includes for all devices are in here

/******************************************************************************
 * FILE DEVICE CONFIGURATION
 ******************************************************************************/

xioFile_t file_x0;				// extended file struct

xioDev_t file0 = {
		XIO_DEV_PGM,
		xio_open_pgm,
		xio_ctrl_device,
		xio_gets_pgm,
		xio_getc_pgm,
		xio_putc_pgm,
		xio_null,
		(xioBuf_t *)NULL,		// file IO is not buffered
		(xioBuf_t *)NULL,
		(xioFile_t *)&file_x0	// unnecessary to initialize the rest of the struct 
};

// Fast accessors
#define PGMx ((xioFile_t *)(d->x))

/******************************************************************************
 * FUNCTIONS
 ******************************************************************************/

/*
 *	xio_init_file() - general purpose FILE device initialization (shared)
 *					  requires open() to be performed to complete the device init
 */
xioDev_t *xio_init_file(uint8_t dev)
{
	file0.dev = dev;	// overwite the structure initialization value in case it was wrong
	return (&file0);
}

/*
 *	xio_open_pgm() - open the program memory device to a specific string address
 *
 *	OK, so this is not really a UNIX open() except for its moral equivalent
 *  Returns a pointer to the stdio FILE struct or -1 on error
 */
FILE *xio_open_pgm(const uint8_t dev, const char *addr, const flags_t flags)
{
	xioDev_t *d = ds[dev];			// convenience device struct pointer
	xio_reset_device(d, flags);
//	((xioFile_t *)(d->x))->filebase_P = (PROGMEM const char *)addr;	// might want to range check this
//	((xioFile_t *)(d->x))->max_offset = PGM_ADDR_MAX;
	PGMx->filebase_P = (PROGMEM const char *)addr;	// might want to range check this
	PGMx->max_offset = PGM_ADDR_MAX;
	return (&d->stream);			// return stdio FILE reference
}

/*
 *  xio_getc_pgm() - read a character from program memory device
 *
 *  Get next character from program memory file.
 *
 *  END OF FILE (EOF)
 *		- set flag_eof when you encounter NUL
 *
 *  LINEMODE and SEMICOLONS behaviors
 *		- consider both <cr> and <lf> to be EOL chars
 *		- convert any EOL char to <lf> to signal end-of-string (e.g. to fgets())
 *
 *  ECHO behaviors
 *		- if ECHO is enabled echo character to stdout
 *		- echo all line termination chars as newlines ('\n')
 */
int xio_getc_pgm(FILE *stream)
{
	char c;
	xioDev_t *d = (xioDev_t *)stream->udata;

	if (d->flag_eof) { return (_FDEV_EOF);}
	if ((c = pgm_read_byte(&PGMx->filebase_P[PGMx->rd_offset])) == NUL) {
		d->flag_eof = true;
	}
	++(PGMx->rd_offset);

	// processing is simple if not in LINEMODE
	if (d->flag_linemode == false) {
		if (d->flag_echo) putchar(c);		// conditional echo
		return (c);
	}
	// now do the LINEMODE stuff
	if (c == NUL) {							// perform newline substitutions
		c = '\n';
	} else if (c == '\r') {
		c = '\n';
	}
	if (d->flag_echo) putchar(c);			// conditional echo
	return (c);
}

/* 
 *	xio_putc_pgm() - write character to to program memory device
 *  Always returns error. You cannot write to program memory
 */
int xio_putc_pgm(const char c, FILE *stream)
{
	return -1;			// always returns an error. Big surprise.
}

/* 
 *	xio_gets_pgm() - main loop task for program memory device
 *
 *	Non-blocking, run-to-completion return a line from memory
 *	Note: LINEMODE flag is ignored. It's ALWAYS LINEMODE here.
 */
int xio_gets_pgm(xioDev_t *d, char *buf, const int size)
{
	if ((PGMx->filebase_P) == 0) {		// return error if no file is open
		return (XIO_FILE_NOT_OPEN);
	}
	if (fgets(buf, size, &d->stream) == NULL) {
		PGMx->filebase_P = NULL;
//		clearerr(&PGM.file);
		clearerr(&d->stream);
		return (XIO_EOF);
	}
	return (XIO_OK);
}
