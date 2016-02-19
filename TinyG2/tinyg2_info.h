/*
 * tinyg2_info.h - tinyg2 build information
 * This file is part of the TinyG project
 *
 * Copyright (c) 2010 - 2015 Alden S. Hart, Jr.
 * Copyright (c) 2010 - 2015 Robert Giseburt
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

#ifndef INFO_H_ONCE
#define INFO_H_ONCE

// You must tag the build first using "git tag ###.##" (you chose the build number for the tag, obviously)
//#define TINYG_FIRMWARE_BUILD            GIT_EXACT_VERSION           // extract build number from tag
//#define TINYG_FIRMWARE_BUILD_STRING     GIT_VERSION                 // extract extended build info from git

#define TINYG_FIRMWARE_BUILD            098.11 // updated tinyg2_info.h to remove git dependencies from compile; updated board files to be consistent with the new pin mapping scheme and not cause compile failures
#define TINYG_FIRMWARE_VERSION		    0.98						// firmware major version
#define TINYG_CONFIG_VERSION		    7							// CV values started at 5 to provide backwards compatibility
#define TINYG_HARDWARE_PLATFORM		    HW_PLATFORM_TINYG_V9		// hardware platform indicator (2 = Native Arduino Due)
#define TINYG_HARDWARE_VERSION		    HW_VERSION_TINYGV9K			// hardware platform revision number
#define TINYG_HARDWARE_VERSION_MAX      (TINYG_HARDWARE_VERSION)

#endif // INFO_H_ONCE
