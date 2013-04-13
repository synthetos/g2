/*
 * xio_usart.h - Common USART definitions 
 * Part of Kinen project
 *
 * Copyright (c) 2012 - 2013 Alden S. Hart Jr.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY 
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef xio_usart_h
#define xio_usart_h

/******************************************************************************
 * USART DEVICE CONFIGS AND STRUCTURES
 ******************************************************************************/

#define USART_BAUD_RATE		115200
#define USART_BAUD_DOUBLER	0									// 0=turns baud doubler off
#define USART_ENABLE_FLAGS	( 1<<RXCIE0 | 1<<TXEN0 | 1<<RXEN0)  // enable recv interrupt, TX and RX
#define USART_XIO_FLAGS 	(XIO_BLOCK |  XIO_ECHO | XIO_XOFF | XIO_LINEMODE )

// Buffer structs must be the same as xioBuf except that the buf array size is defined.
#define USART_RX_BUFFER_SIZE 32
#define USART_TX_BUFFER_SIZE 32

typedef struct xioUsartRX {
	buffer_t size;						// initialize to USART_RX_BUFFER_SIZE-1
	volatile buffer_t rd;				// read index
	volatile buffer_t wr;				// write index
	char buf[USART_RX_BUFFER_SIZE];
} xioUsartRX_t;

typedef struct xioUsartTX {
	buffer_t size;						// initialize to USART_RX_BUFFER_SIZE-1
	volatile buffer_t rd;				// read index
	volatile buffer_t wr;				// write index (written by ISR)
	char buf[USART_TX_BUFFER_SIZE];
} xioUsartTX_t;

/******************************************************************************
 * USART CLASS AND DEVICE FUNCTION PROTOTYPES AND ALIASES
 ******************************************************************************/

xioDev_t *xio_init_usart(uint8_t dev);
FILE *xio_open_usart(const uint8_t dev, const char *addr, const flags_t flags);
void xio_set_baud_usart(xioDev_t *d, const uint32_t baud);
int xio_getc_usart(FILE *stream);
int xio_putc_usart(const char c, FILE *stream);

//int xio_gets_usart(xioDev_t *d, char *buf, const int size);
//void xio_queue_RX_char_usart(const uint8_t dev, const char c);
//void xio_queue_RX_string_usart(const uint8_t dev, const char *buf);

#endif
