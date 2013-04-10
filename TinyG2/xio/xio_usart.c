/*
 * xio_usart.c	- General purpose USART device driver for xmega family
 * 				- works with avr-gcc stdio library
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
#include <stdio.h>					// precursor for xio.h
#include <stdbool.h>				// true and false
#include <avr/interrupt.h>
#include "xio.h"					// nested includes for all devices and types

// allocate and initialize USART structs
xioUsartRX_t usart0_rx = { USART_RX_BUFFER_SIZE-1,1,1 };
xioUsartTX_t usart0_tx = { USART_TX_BUFFER_SIZE-1,1,1 };
xioDev_t usart0 = {
		XIO_DEV_USART,
		xio_open_usart,
		xio_ctrl_device,
		xio_gets_device,
		xio_getc_usart,
		xio_putc_usart,
		xio_null,
		(xioBuf_t *)&usart0_rx,
		(xioBuf_t *)&usart0_tx,		// unnecessary to initialize the rest of the struct 
};

// Fast accessors
//#define USARTrx ds[XIO_DEV_USART]->rx	// these compile to static references
//#define USARTtx ds[XIO_DEV_USART]->tx
#define USART0rx usart0.rx			// these compile to static references
#define USART0tx usart0.tx

/*
 *	xio_init_usart() - general purpose USART initialization (shared)
 *					   requires open() to be performed to complete the device init
 */
xioDev_t *xio_init_usart(uint8_t dev)
{
	usart0.dev = dev;	// overwite the structure initialization value in case it was wrong
	return (&usart0);
}

/*
 *	xio_open_usart() - general purpose USART open
 *	open() assumes that init() has been run previously
 */
FILE *xio_open_usart(const uint8_t dev, const char *addr, const flags_t flags)
{
	xioDev_t *d = ds[dev];			// convenience device struct pointer
	xio_reset_device(d, flags);

	// setup the hardware
	PRR &= ~PRUSART0_bm;			// Enable the USART in the power reduction register (system.h)
	UCSR0A = USART_BAUD_DOUBLER;
	UCSR0B = USART_ENABLE_FLAGS;
	xio_set_baud_usart(d, USART_BAUD_RATE);

	return (&d->stream);			// return stdio FILE reference
}

/* 
 * xio_set_baud_usart() - baud rate setting routine
 * Broken out so it can be exposed to the config system
 */
void xio_set_baud_usart(xioDev_t *d, const uint32_t baud)
{
	UBRR0 = (F_CPU / (8 * baud)) - 1;
	UCSR0A &= ~(1<<U2X0);		// baud doubler off
}

/* 
 * xio_putc_usart() - stdio compatible char writer for usart devices
 * USART TX ISR() - hard-wired for atmega328p 
 */
int xio_putc_usart(const char c, FILE *stream)
{
	int status = xio_write_buffer(((xioDev_t *)stream->udata)->tx, c);
	UCSR0B |= (1<<UDRIE0); 		// enable TX interrupts - they will keep firing
	return (status);
}

ISR(USART_UDRE_vect)
{
	int c = xio_read_buffer(USART0tx);
	if (c == _FDEV_ERR) {
		UCSR0B &= ~(1<<UDRIE0); // disable interrupts
	} else {
		UDR0 = (char)c;			// write char to USART xmit register
	}
}

/*
 *  xio_getc_usart() - generic char reader for USART devices
 *  USART RX ISR() - This function is hard-wired for the atmega328p config
 *
 *	Compatible with stdio system - may be bound to a FILE handle
 *	This version is non-blocking. To add blocking behaviors use the alternate code
 *
 *  BLOCKING behavioir
 *	 	- execute blocking or non-blocking read depending on controls
 *		- return character or -1 & XIO_SIG_WOULDBLOCK if non-blocking
 *		- return character or sleep() if blocking
 *
 *  ECHO behaviors
 *		- if ECHO is enabled echo character to stdout
 *		- echo all line termination chars as newlines ('\n')
 *		- Note: putc is responsible for expanding newlines to <cr><lf> if needed
 */
ISR(USART_RX_vect) 
{ 
	xio_write_buffer(USART0rx, UDR0);
}

int xio_getc_usart(FILE *stream)
{
	// non-blocking version - returns _FDEV_ERR if no char available
	xioDev_t *d = (xioDev_t *)stream->udata;		// get device struct pointer
	int c = xio_read_buffer(d->rx);
	d->x_flow(d);									// run the flow control function callback
	if (d->flag_echo) { d->x_putc(c, stdout);}		// conditional echo 
	if ((c == CR) || (c == LF)) { if (d->flag_linemode) { return('\n');}}
	return (c);

/* blocking version - requires #include <avr/sleep.h>
	xioDev_t *d = (xioDev_t *)stream->udata;		// get device struct pointer
	int c;
	while ((c = xio_read_buffer(d->rx)) == _FDEV_ERR) { sleep_mode();}
//	...or just this:
	while ((c = xio_read_buffer(d->rx)) == _FDEV_ERR);

	d->x_flow(d);									// run the flow control function callback
	if (d->flag_echo) { d->x_putc(c, stdout);}		// conditional echo 
	if ((c == CR) || (c == LF)) { if (d->flag_linemode) { return('\n');}}
	return (c);
*/
}
