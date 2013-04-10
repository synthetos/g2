/*
 * xio_spi.c	- General purpose SPI device driver for xmega family
 * 				- works with avr-gcc stdio library
 *
 * Part of Kinen project
 *
 * Copyright (c) 2013 Alden S. Hart Jr.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY 
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
/* ---- SPI Protocol ----
 *
 * The SPI master/slave protocol is designed to be as simple as possible. 
 * In short, the master transmits whenever it wants to and the slave returns
 * the next character in its output 		buffer whenever there's an SPI transfer.
 * No flow control is needed as the master initiates and drives all transfers.
 *
 * More details:
 *
 *	- A "message" is a line of text. Examples of messages are requests from the 
 *		master to a slave, responses to these requests, and asynchronous messages 
 *		(from a slave) that are not tied to a request.
 *
 *		Messages are terminated with a newline (aka NL, LF, line-feed). The 
 *		terminating NL is considered part of the message and should be transmitted.
 *
 *		If multiple NLs are transmitted each trailing NL is interpreted as a blank
 *		message. This is generally not good practice - so watch it.
 *
 *		Carriage return (CR) is not recognized as a newline. A CR in a message is
 *		treated as any other non-special ASCII character.
 *
 *		NULs (0x00) are not transmitted in either direction (e.g. string terminations).
 *		Depending on the master or slave internals, it may convert internal string 
 *		terminating NULs to NLs for transmission. 
 *
 *	- A slave is always in RX state - it must always be able to receive message data (MOSI).
 *
 *	- All SPI transmissions are initiated by the master and are 8 bits long. As the 
 *		slave is receiving the byte on MOSI it should be returning the next character 
 *		in its output buffer on MISO. Note that there is no inherent correlation between
 *		the char (or message) being received from the master and transmitted from the
 *		slave. It's just IO.
 *
 *		If the slave has no data to send it should return ETX (0x03) on MISO. This is 
 *		useful to distinghuish between an "empty" slave and a non-responsive SPI slave or
 *		unpopulated Kinen socket - which would return NULs or possibly 0xFFs.
 *
 *	- The master may poll for message data from the slave by sending STX chars to
 *		the slave. The slave discards all STXs and simply returns output data on these
 *		transfers. Presumably the master would stop polling once it receives an ETX 
 *		from the slave.
 */
#include <stdio.h>					// precursor for xio.h
#include <stdbool.h>				// true and false
#include <avr/interrupt.h>
#include "xio.h"					// nested includes for all devices and types

// allocate and initialize SPI structs
xioSpiRX_t spi0_rx = { SPI_RX_BUFFER_SIZE-1,1,1 };
xioSpiTX_t spi0_tx = { SPI_TX_BUFFER_SIZE-1,1,1 };
xioDev_t spi0 = {
		XIO_DEV_SPI,
		xio_open_spi,
		xio_ctrl_device,				// use device generic function
		xio_gets_device,				// " "
		xio_getc_device,				// " "
		xio_putc_device,				// " "
		xio_null,
		(xioBuf_t *)&spi0_rx,
		(xioBuf_t *)&spi0_tx,			// unecessary to initialize from here on...
};

// Fast accessors
//#define SPIrx ds[XIO_DEV_SPI]->rx		// these compile to static references
//#define SPItx ds[XIO_DEV_SPI]->tx
#define SPI0rx spi0.rx					// these compile to static references
#define SPI0tx spi0.tx

/*
 *	xio_init_spi() - general purpose SPI initialization (shared)
 *					 requires open() to be performed to complete the device init
 */
xioDev_t *xio_init_spi(uint8_t dev)
{
	spi0.dev = dev;	// overwrite the structure initialization value in case it was wrong
	return (&spi0);
}

/*
 *	xio_open_spi() - open a specific SPI device
 */
FILE *xio_open_spi(const uint8_t dev, const char *addr, const flags_t flags)
{
	xioDev_t *d = ds[dev];			// convenience device struct pointer
	xio_reset_device(d, flags);

	// setup the SPI hardware device
	PRR &= ~PRSPI_bm;				// Enable SPI in power reduction register (system.h)
	SPCR |= SPI_MODE;
	DDRB |= SPI_OUTBITS;

	return (&d->stream);			// return stdio FILE reference
}

/*
 * xio_getc_spi() - read char from the RX buffer. Return error if no character available
 * xio_putc_spi() - Write a character into the TX buffer for MISO piggyback transmission
 * SPI Slave Interrupt() - interrupts on RX byte received
 */
 /*
int xio_getc_spi(FILE *stream)
{
	return (xio_read_buffer(((xioDev_t *)stream->udata)->rx));
}

int xio_putc_spi(const char c, FILE *stream)
{
	return (xio_write_buffer(((xioDev_t *)stream->udata)->tx, c));
}
*/
ISR(SPI_STC_vect)
{
	char c = SPDR;								// read the incoming character; save it
	int c_out = xio_read_buffer(SPI0tx); 		// stage the next char to transmit on MISO from the TX buffer
	if (c_out ==_FDEV_ERR) SPDR = ETX; else SPDR = (char)c_out;	// stage next TX char or ETX if none	
	xio_write_buffer(SPI0rx, c);				// write incoming char into RX buffer

//	char c = SPDR;									// read the incoming character; save it
//	if (SPI0rx->head == SPI0rx->tail) { SPDR = NAK;}	// RX buffer is full. - send NAK to master
//	if (SPI0tx->head == SPI0tx->tail) { SPDR = ETX;}	// TX buffer is empty - send ETX to master
//	if ((--(SPI0tx->tail)) == 0) { SPI0tx->tail = SPI0tx->size;}	// advance tail with wrap
//	SPDR = (SPI0tx->buf[SPI0tx->tail]);				// get character from TX buffer
//	xio_write_buffer(SPI0rx, c);					// write received char to RX buffer

//	int c_out = xio_read_buffer(SPI0tx); 		// stage the next char to transmit on MISO from the TX buffer
//	if (c_out ==_FDEV_ERR) { c_out = ETX;}
//	char c = SPDR;								// read the incoming character; save it
//	SPDR = c_out;
//	xio_write_buffer(SPI0rx, c);				// write incoming char into RX buffer

//	char c = SPDR;								// read the incoming character; save it
//	if (SPI0tx->head == SPI0tx->tail) { SPDR = ETX;}
//	if ((--(SPI0tx->tail)) == 0) { SPI0tx->tail = SPI0tx->size;}	// advance tail with wrap
//	SPDR = SPI0tx->buf[SPI0tx->tail];				// stage the character from TX buffer
//	xio_write_buffer(SPI0rx, c);					// write incoming char into RX buffer
}

/* 
 *	xio_gets_spi() - read a complete line (message) from an SPI device
 *
 *	Reads from the local RX buffer until it's empty. Retains line context 
 *	across calls - so it can be called multiple times. Reads as many characters 
 *	as it can until any of the following is true:
 *
 *	  - Encounters newline indicating a complete line. Terminate the buffer
 *		but do not write the newlinw into the buffer. Reset line flag. Return XIO_OK.
 *
 *	  - Encounters an empty buffer. Leave in_line. Return XIO_EAGAIN.
 *
 *	  - A successful read would cause output buffer overflow. Return XIO_BUFFER_FULL
 *
 *	Note: LINEMODE flag in device struct is ignored. It's ALWAYS LINEMODE here.
 *	Note: CRs are not recognized as NL chars - master must send LF to terminate a line
 */
/*
int xio_gets_spi(xioDev_t *d, char *buf, const int size)
{
	int c;

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
		if ((c = xio_read_buffer(d->rx)) == _FDEV_ERR) { return (XIO_EAGAIN);}
		if (c == LF) {
			d->buf[(d->len)++] = LF;			//+++++++++++++++
			d->buf[(d->len)++] = NUL;
			d->flag_in_line = false;			// clear in-line state (reset)
			return (XIO_OK);					// return for end-of-line
		}
		d->buf[d->len++] = c;					// write character to output buffer
	}
}
*/

