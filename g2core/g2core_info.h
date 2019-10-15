/*
 * g2core_info.h - g2core build information
 * This file is part of the g2core project
 *
 * Copyright (c) 2010 - 2018 Alden S. Hart, Jr.
 * Copyright (c) 2010 - 2018 Robert Giseburt
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

#define G2CORE_FIRMWARE_BUILD			101.03  // Issue #320, #354 - Added stepper polarity {1pl:n},  Fixed SR setting bug
#define G2CORE_FIRMWARE_VERSION         0.99

#ifdef GIT_VERSION
#define G2CORE_FIRMWARE_BUILD_STRING   GIT_VERSION
#else
#define G2CORE_FIRMWARE_BUILD_STRING   "unknown"
#endif

#endif // G2CORE_INFO_H_ONCE
