/*
 * xio.cpp
 *
 * Created: 4/13/2013 1:05:24 PM
 *  Author: Administrator
 */ 

#include "Arduino.h"
#include "tinyg2.h"
#include "xio.h"


/*
 * read_char() - returns single char or -1 (_FDEV_ERR) is none available
 */
int read_char (void)
{
	return SerialUSB.read();
}

/* 
 *	read_line() - read a complete line from stdin
 *
 *	Returns:
 *
 *	  ERR_OK		  Returns a complete null terminated string. 
 *					  Index contains total character count (less terminating NUL)
 *					  The terminating LF is not written to the string.
 *
 *	  ERR_EAGAIN	  Line is incomplete because input has no more characters.
 *					  Index is left at the first available space.
 *					  Retry later to read more of the string. Use index from previous call.
 * 
 *	  ERR_EOF		  Line is incomplete because end of file was reached (file devices)
 *					  Index can be used as a character count.
 *
 *	  ERR_BUFFER_FULL Incomplete because size was reached.
 *                    Index will equal size.
 *
 *	  ERR_FILE_SIZE_EXCEEDED returned if the starting index exceeds the size.
 */

err_t read_line (uint8_t *buffer, size_t *index, size_t size)
{
	if (*index >= size) { return (ERR_FILE_SIZE_EXCEEDED);}

	for (int c; *index < size; (*index)++ ) {
		if ((c = read_char()) != _FDEV_ERR) {
			if (c == LF) {
				buffer[*index] = NUL;
				return (ERR_OK);
			}
			buffer[*index] = (char)c;
			continue;
		}
		return (ERR_EAGAIN);
	}	
	return (ERR_BUFFER_FULL);
}

size_t write(const uint8_t *buffer, size_t size)
{
	SerialUSB.write(buffer, size);
	return (size);
}