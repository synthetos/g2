/*
 * g2core_info.h - g2core build information
 * This file is part of the g2core project
 *
 * Copyright (c) 2010 - 2016 Alden S. Hart, Jr.
 * Copyright (c) 2010 - 2016 Robert Giseburt
 *
 * This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software.  If not, see <http://www.gnu.org/licenses/>.
 *
 * THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT ANY
 * WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
 * SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef G2CORE_INFO_H_ONCE
#define G2CORE_INFO_H_ONCE

#define G2CORE_FIRMWARE_BUILD			100.15  // fix for #200 and other USB issues  
#ifdef GIT_VERSION
#define G2CORE_FIRMWARE_BUILD_STRING	GIT_VERSION
#else
#define G2CORE_FIRMWARE_BUILD_STRING	"unknown"
#endif
#define G2CORE_FIRMWARE_VERSION			0.99						// firmware major version
#define G2CORE_HARDWARE_PLATFORM		HW_PLATFORM_V9		        // hardware platform indicator (2 = Native Arduino Due)
#define G2CORE_HARDWARE_VERSION		    HW_VERSION_TINYGV9K			// hardware platform revision number
#define G2CORE_HARDWARE_VERSION_MAX     (G2CORE_HARDWARE_VERSION)

#endif // G2CORE_INFO_H_ONCE
