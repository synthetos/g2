/*
 * util.h - a random assortment of useful functions
 * This file is part of the g2core project
 *
 * Copyright (c) 2010 - 2016 Alden S. Hart, Jr.
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
/* util.c/.h contains a dog's breakfast of supporting functions that are
 * not specific to g2core: including:
 *
 *  - math and min/max utilities and extensions
 *  - vector manipulation utilities
 *  - support for debugging routines
 */

#include "hardware.h" // for AXES

#ifndef UTIL_H_ONCE
#define UTIL_H_ONCE

#include <stdint.h>
//#include "sam.h"
#include "MotateTimers.h"
using Motate::delay;
using Motate::SysTickTimer;

#include <type_traits>
#include <algorithm> // min, max
#include <cmath> // isnan, isinf

/****** Global Scope Variables and Functions ******/

//*** debug utilities ***

#pragma GCC push_options
#pragma GCC optimize ("O0")
inline void _debug_trap(const char *reason) {
    // We might be able to put a print here, but it MIGHT interrupt other output
    // and might be deep in an ISR, so we had better just _NOP() and hope for the best.
    __NOP();
#if IN_DEBUGGER == 1
    __asm__("BKPT");
#endif
}
#pragma GCC reset_options

//*** vector utilities ***

extern float vector[AXES]; // vector of axes for passing to subroutines

#define clear_vector(a) (memset(a,0,sizeof(a)))
#define copy_vector(d,s) (memcpy(d,s,sizeof(d)))

float get_axis_vector_length(const float a[], const float b[]);
uint8_t vector_equal(const float a[], const float b[]);
float *set_vector(float x, float y, float z, float a, float b, float c);
float *set_vector_by_axis(float value, uint8_t axis);

//*** math utilities ***

float min3(float x1, float x2, float x3);
float min4(float x1, float x2, float x3, float x4);
float max3(float x1, float x2, float x3);
float max4(float x1, float x2, float x3, float x4);
//float std_dev(float a[], uint8_t n, float *mean);

//*** string utilities ***

//#ifdef __ARM
//uint8_t * strcpy_U( uint8_t * dst, const uint8_t * src );
//#endif

uint8_t isnumber(char c);
char *escape_string(char *dst, char *src);
char inttoa(char *str, int n);
char floattoa(char *buffer, float in, int precision, int maxlen = 16);
//char fntoa(char *str, float n, uint8_t precision);

uint16_t compute_checksum(char const *string, const uint16_t length);

//*** other utilities ***

uint32_t SysTickTimer_getValue(void);

//**** Math Support *****

// See http://www.cplusplus.com/doc/tutorial/namespaces/#using

using std::isnan;
using std::isinf;
using std::min;
using std::max;

template <typename T>
inline T square(const T x) { return (x)*(x); }        /* UNSAFE */

inline float abs(const float a) { return fabs(a); }

#ifndef avg
template <typename T>
inline T avg(const T a,const T b) {return (a+b)/2; }
#endif

#ifndef EPSILON
#define EPSILON     ((float)0.00001)    // allowable rounding error for floats
#define EPSILON4    ((float)0.0001)     // reduced precision epsilon
#define EPSILON3    ((float)0.001)      // reduced precision epsilon
#define EPSILON2    ((float)0.01)       // reduced precision epsilon
#endif

// These functions all require math.h to be included in each file that uses them
#ifndef fp_EQ
#define fp_EQ(a,b) (fabs(a-b) < EPSILON)
#endif
#ifndef fp_NE
#define fp_NE(a,b) (fabs(a-b) > EPSILON)
#endif
#ifndef fp_GE
#define fp_GE(a,b) (fabs(a-b) < EPSILON || a-b > EPSILON)
#endif
#ifndef fp_ZERO
#define fp_ZERO(a) (fabs(a) < EPSILON)
#endif
#ifndef fp_NOT_ZERO
#define fp_NOT_ZERO(a) (fabs(a) > EPSILON)
#endif
#ifndef fp_FALSE
#define fp_FALSE(a) (a < EPSILON)
#endif
#ifndef fp_TRUE
#define fp_TRUE(a) (a > EPSILON)
#endif

// Constants
#define MAX_LONG (2147483647)
#define MAX_ULONG (4294967295)
#define MM_PER_INCH (25.4)
#define INCHES_PER_MM (1/25.4)
#define MICROSECONDS_PER_MINUTE ((float)60000000)
#define MINUTES_PER_MICROSECOND ((float)1/MICROSECONDS_PER_MINUTE)
#define uSec(a) ((float)(a * MICROSECONDS_PER_MINUTE))

#define RADIAN (57.2957795)

#ifndef M_PI    // M_PI is pi usually defined in math.h, but not always (C++11)
#define M_PI (3.14159265358979323846264338327950288)
#endif

#ifndef M_SQRT2 // M_SQRT2 is radical2 as defined in math.h
#define M_SQRT2 (1.41421356237310)
#endif

#ifndef M_SQRT3
#define M_SQRT3 (1.73205080756888)
#endif

// Fraction part
constexpr float c_atof_frac_(char *&p_, float v_, float m_) {
    return ((*p_ >= '0') && (*p_ <= '9')) ? (v_ = ((v_) + ((*p_) - '0') * m_), c_atof_frac_(++p_, v_, m_ / 10.0)) : v_;
}

// Integer part
template <typename int_type>
constexpr float c_atof_int_(char *&p_, int_type v_) {
    return (*p_ == '.')
    ? (float)(v_) + c_atof_frac_(++p_, 0, 1.0 / 10.0)
    : (((*p_ >= '0') && (*p_ <= '9')) ? ((v_ = ((*p_) - '0') + (v_ * 10)), c_atof_int_(++p_, v_)) : v_);
}

// Start portion
constexpr float c_atof(char *&p_) { return (*p_ == '-') ? (c_atof_int_(++p_, 0) * -1.0) : ( (*p_ == '+') ? c_atof_int_(++p_, 0) : (c_atof_int_(p_, 0))); }

// It's assumed that the string buffer contains at lest count_ non-\0 chars
//constexpr int c_strreverse(char * const t, const int count_, char hold = 0) {
//    return count_>1 ? (hold=*t, *t=*(t+(count_-1)), *(t+(count_-1))=hold), c_strreverse(t+1, count_-2), count_ : count_;
//}

template <int32_t length>
void str_concat(char *&dest, const char (&data)[length]) {
    // length includes the \0
    strncpy(dest, data, length); dest += length-1;
};

#endif    // End of include guard: UTIL_H_ONCE
