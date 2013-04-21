/*
 * util.cpp - a random assortment of useful functions
 * Part of TinyG project
 *
 */
/* util contains a dog's breakfast of supporting functions that are 
 * not specific to tinyg: including:
 *	  - math and min/max utilities and extensions 
 *	  - vector manipulation utilities
 *	  - support for debugging routines
 */  
#include "util.h"

/*
#include <ctype.h>
#include <stdio.h>				// precursor for xio.h
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include <avr/pgmspace.h>		// precursor for xio.h

#include "tinyg.h"
#include "config.h"
#include "controller.h"
#include "canonical_machine.h"
#include "planner.h"
#include "stepper.h"
#include "report.h"
*/
#ifdef __cplusplus
extern "C"{
#endif

uint8_t * strcpy_U( uint8_t * dst, const uint8_t * src )
{
	uint16_t index = 0;
	do {
		dst[index] = src[index];	
	} while (src[index++] != 0);
	return dst;

//	for (uint16_t index=0; src[index] != 0; index++) {
//		dst[index] = src[index];
//	}
//	dst[index] = 0;		// terminate string
//	return dst;
}

/**** Vector functions ****
 * copy_vector()			- copy vector of arbitrary length
 * copy_axis_vector()		- copy an axis vector
 * set_unit_vector()		- populate a unit vector by pos. & target
 * get_axis_vector_length()	- return the length of an axis vector
 * set_vector()				- load values into vector form
 * set_vector_by_axis()		- load a single value into a zero vector
 */

inline void copy_vector(float dest[], const float src[], uint8_t length) 
{
	for (uint8_t i=0; i<length; i++) {
		dest[i] = src[i];
	}
}

inline void copy_axis_vector(float dest[], const float src[]) 
{
	memcpy(dest, src, sizeof(float)*AXES);
}

float get_axis_vector_length(const float a[], const float b[]) 
{
	return (sqrt(square(a[AXIS_X] - b[AXIS_X]) +
				 square(a[AXIS_Y] - b[AXIS_Y]) +
				 square(a[AXIS_Z] - b[AXIS_Z]) +
				 square(a[AXIS_A] - b[AXIS_A]) +
				 square(a[AXIS_B] - b[AXIS_B]) +
				 square(a[AXIS_C] - b[AXIS_C])));
}

float *set_vector(float x, float y, float z, float a, float b, float c)
{
	vector[AXIS_X] = x;
	vector[AXIS_Y] = y;
	vector[AXIS_Z] = z;
	vector[AXIS_A] = a;
	vector[AXIS_B] = b;
	vector[AXIS_C] = c;
	return (vector);
}

float *set_vector_by_axis(float value, uint8_t axis)
{
	clear_vector(vector);
	switch (axis) {
		case (X): vector[AXIS_X] = value; break;
		case (Y): vector[AXIS_Y] = value; break;
		case (Z): vector[AXIS_Z] = value; break;
		case (A): vector[AXIS_A] = value; break;
		case (B): vector[AXIS_B] = value; break;
		case (C): vector[AXIS_C] = value;
	}
	return (vector);
}

/**** Math and other general purpose functions ****/

/* Slightly faster (*) multi-value min and max functions
 * 	min3() - return minimum of 3 numbers
 * 	min4() - return minimum of 4 numbers
 * 	max3() - return maximum of 3 numbers
 * 	max4() - return maximum of 4 numbers
 *
 * Implementation tip: Order the min and max values from most to least likely in the calling args
 *
 * (*) Macro min4 is about 20uSec, inline function version is closer to 10 uSec
 * 	#define min3(a,b,c) (min(min(a,b),c))
 *	#define min4(a,b,c,d) (min(min(a,b),min(c,d)))
 *	#define max3(a,b,c) (max(max(a,b),c))
 *	#define max4(a,b,c,d) (max(max(a,b),max(c,d)))
 */

inline float min3(float x1, float x2, float x3)
{
	float min = x1;
	if (x2 < min) { min = x2;} 
	if (x3 < min) { return (x3);} 
	return (min);
}

inline float min4(float x1, float x2, float x3, float x4)
{
	float min = x1;
	if (x2 < min) { min = x2;} 
	if (x3 < min) { min = x3;} 
	if (x4 < min) { return (x4);}
	return (min);
}

inline float max3(float x1, float x2, float x3)
{
	float max = x1;
	if (x2 > max) { max = x2;} 
	if (x3 > max) { return (x3);} 
	return (max);
}

inline float max4(float x1, float x2, float x3, float x4)
{
	float max = x1;
	if (x2 > max) { max = x2;} 
	if (x3 > max) { max = x3;} 
	if (x4 > max) { return (x4);}
	return (max);
}

/*
 * isnumber() - isdigit that also accepts plus, minus, and decimal point
 */

uint8_t isnumber(char_t c)
{
	if (c == '.') { return (true); }
	if (c == '-') { return (true); }
	if (c == '+') { return (true); }
	return (isdigit(c));
}

/* 
 * read_float() - read a float from a normalized char array
 *
 *	buf			normalized char array (line)
 *	i			char array index must point to start of number
 *	float_ptr	pointer to float to write value into
 *
 *	The line is normalized when it is all caps, has no white space,
 *	no non-alphnumeric characters, and no newline or CR.
 */

uint8_t read_float(uint8_t *buf, uint8_t *index, float *float_ptr) 
{
	char_t *start = buf + *index;
	char_t *end;
  
	*float_ptr = (float)strtod(start, &end);
	if(end == start) { 
		return(false); 
	}
	*i = (uint8_t)(end - buf);
	return(true);
}

/* 
 * compute_checksum() - calculate the checksum for a string
 * 
 *	Stops calculation on null termination or length value if non-zero.
 *
 * 	This is based on the the Java hashCode function. 
 *	See http://en.wikipedia.org/wiki/Java_hashCode()
 */
#define HASHMASK 9999

uint16_t compute_checksum(char_t const *string, const uint16_t length) 
{
	uint32_t h = 0;
	uint16_t len = strlen(string);

	if (length != 0) {
		len = min(len, length);
	}
    for (uint16_t i=0; i<len; i++) {
		h = 31 * h + string[i];
    }
    return (h % HASHMASK);
}

#ifdef __cplusplus
}
#endif
