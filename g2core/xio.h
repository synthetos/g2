/*
 * xio.h - extended IO functions
 * This file is part of the g2core project
 *
 * Copyright (c) 2013 - 2016 Alden S. Hart Jr.
 * Copyright (c) 2013 - 2016 Robert Giseburt
 *
 * This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software.  If not, see <http://www.gnu.org/licenses/>.
 *
 * As a special exception, you may use this file as part of a software library without
 * restriction. Specifically, if other files instantiate templates or use macros or
 * inline functions from this file, or you compile this file and link it with  other
 * files to produce an executable, this file does not by itself cause the resulting
 * executable to be covered by the GNU General Public License. This exception does not
 * however invalidate any other reasons why the executable file might be covered by the
 * GNU General Public License.
 *
 * THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT ANY
 * WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
 * SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
/*
 * CAVEAT EMPTOR: File under "watch out!":
 *
 *  - Short story: Do not call ANYTHING that can print (i.e. send chars to the TX
 *    buffer) from a medium or hi interrupt. This obviously includes any printf()
 *    function, but also exception reports, cm_soft_alarm(), cm_hard_alarm() and a
 *    few other functions that call stdio print functions.
 *
 *  - Longer Story: The stdio printf() functions use character drivers provided by
 *    g2core to access the low-level Xmega devices. Specifically xio_putc_usb() in xio_usb.c,
 *    and xio_putc_rs485() in xio_rs485.c. Since stdio does not understand non-blocking
 *    IO these functions must block if there is no space in the TX buffer. Blocking is
 *    accomplished using sleep_mode(). The IO system is the only place where sleep_mode()
 *    is used. Everything else in g2core is non-blocking. Sleep is woken (exited) whenever
 *    any interrupt fires. So there must always be a viable interrupt source running when
 *    you enter a sleep or the system will hang (lock up). In the IO functions this is the
 *    TX interrupts, which fire when space becomes available in the USART for a TX char. This
 *    Means you cannot call a print function at or above the level of the TX interrupts,
 *    which are set to medium.
 */
#ifndef XIO_H_ONCE
#define XIO_H_ONCE

//#include "g2core.h"                // not required if used in g2core project
#include "config.h"                 // required for nvObj typedef

/**** Defines, Macros, and  Assorted Parameters ****/

#undef  _FDEV_ERR
#define _FDEV_ERR -1

#undef  _FDEV_EOF
#define _FDEV_EOF -2

#define USB_LINE_BUFFER_SIZE    255         // text buffer size

//*** Device flags ***
typedef uint16_t devflags_t;                // might need to bump to 32 be 16 or 32

// device capabilities flags
#define DEV_CAN_BE_CTRL       (0x0001)        // device can be a control channel
#define DEV_CAN_BE_DATA       (0x0002)        // device can be a data channel
#define DEV_IS_ALWAYS_BOTH    (0x0004)        // device is always a control and a data channel
#define DEV_IS_MUTE_SECONDARY (0x0008)        // device is "muted" as a non-primary device
#define DEV_CAN_READ          (0x0010)
#define DEV_CAN_WRITE         (0x0020)

// Device state flags
// channel state
#define DEV_IS_CTRL         (0x0001)        // device is set as a control channel
#define DEV_IS_DATA         (0x0002)        // device is set as a data channel
#define DEV_IS_PRIMARY      (0x0004)        // device is the primary control channel
#define DEV_IS_MUTED        (0x0008)        // device is muted as it is currently the non-primary device

// device connection state
#define DEV_IS_CONNECTED    (0x0020)        // device is connected (e.g. USB)
#define DEV_IS_READY        (0x0040)        // device is ready for use
#define DEV_IS_ACTIVE       (0x0080)        // device is active

// device exception flags
#define DEV_THROW_EOF       (0x0100)        // end of file encountered

// device specials
#define DEV_IS_BOTH         (DEV_IS_CTRL | DEV_IS_DATA)
#define DEV_FLAGS_CLEAR     (0x0000)        // Apply as flags = DEV_FLAGS_CLEAR;

enum xioDeviceEnum {                        // reconfigure this enum as you add more physical devices
    DEV_NONE=-1,                            // no device is bound
    DEV_USB0=0,                             // must be 0
    DEV_USB1,                               // must be 1
    DEV_UART1,                              // must be 2
//  DEV_SPI0,                               // We can't have it here until we actually define it
    DEV_MAX
};

enum xioSPIMode {
    SPI_DISABLE=0,                          // tri-state SPI lines
    SPI_ENABLE                              // enable SPI lines for output
};


/**** readline stuff -- TODO *****/

#define RX_BUFFER_MIN_SIZE       256        // minimum requested buffer size (they are usually larger)

/**** function prototypes ****/

void xio_init(void);
stat_t xio_test_assertions(void);

size_t xio_write(const char *buffer, size_t size, bool only_to_muted = false);
char *xio_readline(devflags_t &flags, uint16_t &size);
int16_t xio_writeline(const char *buffer, bool only_to_muted = false);
bool xio_connected();

stat_t xio_set_spi(nvObj_t *nv);

/**** newlib-nano support function(s) ****/
extern "C" {
    int _write( int file, char *ptr, int len );
}

/* Some useful ASCII definitions */

#define NUL (char)0x00      //  ASCII NUL char (0) (not "NULL" which is a pointer)
#define STX (char)0x02      // ^b - STX (start text)
#define ETX (char)0x03      // ^c - ETX (end of text) (queue flush marker)
#define EOT (char)0x04      // ^d - EOT (end of transmission)
#define ENQ (char)0x05      // ^e - ENQuire
#define ACK (char)0x06      // ^f - ACKnowledge
#define BEL (char)0x07      // ^g - BEL
#define BS  (char)0x08      // ^h - backspace
#define TAB (char)0x09      // ^i - character
#define LF (char)0x0A       // ^j - line feed
#define VT (char)0x0B       // ^k - kill stop
#define CR (char)0x0D       // ^m - carriage return
#define XON (char)0x11      // ^q - DC1, XON, resume
#define XOFF (char)0x13     // ^s - DC3, XOFF, pause
#define NAK (char)0x15      // ^u - Negative acknowledgment
#define CAN (char)0x18      // ^x - Cancel, abort
#define ESC (char)0x1B      // ^[ - ESC(ape)
#define SPC (char)0x20      // ' '  Space character
#define DEL (char)0x7F      //  DEL(ete)

#define Q_EMPTY (char)0xFF  // signal no character

/* Signal character mappings */

#define CHAR_RESET CAN
#define CHAR_ALARM EOT
#define CHAR_FEEDHOLD (char)'!'
#define CHAR_CYCLE_START (char)'~'
#define CHAR_QUEUE_FLUSH (char)'%'
//#define CHAR_BOOTLOADER ESC

#ifdef __TEXT_MODE

    void xio_print_spi(nvObj_t *nv);

#else

    #define xio_print_spi tx_print_stub

#endif // __TEXT_MODE


#endif // XIO_H_ONCE

/* Handy reference
Binary        Oct    Dec    Hex    Glyph
010 0000    040    32    20    ?
010 0001    041    33    21    !
010 0010    042    34    22    "
010 0011    043    35    23    #
010 0100    044    36    24    $
010 0101    045    37    25    %
010 0110    046    38    26    &
010 0111    047    39    27    '
010 1000    050    40    28    (
010 1001    051    41    29    )
010 1010    052    42    2A    *
010 1011    053    43    2B    +
010 1100    054    44    2C    ,
010 1101    055    45    2D    -
010 1110    056    46    2E    .
010 1111    057    47    2F    /
011 0000    060    48    30    0
011 0001    061    49    31    1
011 0010    062    50    32    2
011 0011    063    51    33    3
011 0100    064    52    34    4
011 0101    065    53    35    5
011 0110    066    54    36    6
011 0111    067    55    37    7
011 1000    070    56    38    8
011 1001    071    57    39    9
011 1010    072    58    3A    :
011 1011    073    59    3B    ;
011 1100    074    60    3C    <
011 1101    075    61    3D    =
011 1110    076    62    3E    >
011 1111    077    63    3F    ?

Binary        Oct    Dec    Hex    Glyph
100 0000    100    64    40    @
100 0001    101    65    41    A
100 0010    102    66    42    B
100 0011    103    67    43    C
100 0100    104    68    44    D
100 0101    105    69    45    E
100 0110    106    70    46    F
100 0111    107    71    47    G
100 1000    110    72    48    H
100 1001    111    73    49    I
100 1010    112    74    4A    J
100 1011    113    75    4B    K
100 1100    114    76    4C    L
100 1101    115    77    4D    M
100 1110    116    78    4E    N
100 1111    117    79    4F    O
101 0000    120    80    50    P
101 0001    121    81    51    Q
101 0010    122    82    52    R
101 0011    123    83    53    S
101 0100    124    84    54    T
101 0101    125    85    55    U
101 0110    126    86    56    V
101 0111    127    87    57    W
101 1000    130    88    58    X
101 1001    131    89    59    Y
101 1010    132    90    5A    Z
101 1011    133    91    5B    [
101 1100    134    92    5C    \
101 1101    135    93    5D    ]
101 1110    136    94    5E    ^
101 1111    137    95    5F    _

Binary        Oct    Dec    Hex    Glyph
110 0000    140     96    60    `
110 0001    141     97    61    a
110 0010    142     98    62    b
110 0011    143     99    63    c
110 0100    144    100    64    d
110 0101    145    101    65    e
110 0110    146    102    66    f
110 0111    147    103    67    g
110 1000    150    104    68    h
110 1001    151    105    69    i
110 1010    152    106    6A    j
110 1011    153    107    6B    k
110 1100    154    108    6C    l
110 1101    155    109    6D    m
110 1110    156    110    6E    n
110 1111    157    111    6F    o
111 0000    160    112    70    p
111 0001    161    113    71    q
111 0010    162    114    72    r
111 0011    163    115    73    s
111 0100    164    116    74    t
111 0101    165    117    75    u
111 0110    166    118    76    v
111 0111    167    119    77    w
111 1000    170    120    78    x
111 1001    171    121    79    y
111 1010    172    122    7A    z
111 1011    173    123    7B    {
111 1100    174    124    7C    |
111 1101    175    125    7D    }
111 1110    176    126    7E    ~
*/
