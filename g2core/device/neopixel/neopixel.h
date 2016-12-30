/*
 * neopixel.h - control over a variety of WS2811- and WS2812-based RGB
 *   LED devices such as Adafruit FLORA RGB Smart Pixels and NeoPixel strips.
 * This file is part of G2 project
 *
 * Copyright (c) 2016 Alden S. Hart, Jr.
 * Copyright (c) 2016 Robert Giseburt
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
#ifndef NEOPIXEL_DRIVER_H_ONCE
#define NEOPIXEL_DRIVER_H_ONCE

#include "MotatePins.h"
#include "MotateTimers.h"
#include <type_traits>

#pragma mark Color objects

// Bonus, color objects

struct NeopixelColorTag {};

// HSI Color object
// See C. of https://en.wikipedia.org/wiki/HSL_and_HSV#/media/File:Hsl-hsv_models.svg for Hue
// Remember, I (intensity) is, effectively, brightness. Unlike L (lightness) and V (value), the hue is visible for all
// I.
// Hue: Red = 0, Yellow = 60, Green = 120, Indigo = 180, Blue = 240, Magenta = 300
struct HSI_Color_t : NeopixelColorTag {
    float hue;
    float saturation;
    float intensity;

    const uint32_t _update_timeout_ms;

    Motate::Timeout _update_timeout;

    uint32_t _transition_steps_left = 0;
    // forward difference values
    float _hue_fd_0, _hue_fd_1;
    float _saturation_fd_0, _saturation_fd_1;
    float _intensity_fd_0, _intensity_fd_1;

    HSI_Color_t(uint32_t _update_every_ms = 1) : _update_timeout_ms{_update_every_ms} { _update_timeout.set(0); };

    HSI_Color_t(float _hue, float _saturation, float _intensity, uint32_t _update_every_ms = 1)
        : hue{_hue}, saturation{_saturation}, intensity{_intensity}, _update_timeout_ms{_update_every_ms} {
        _update_timeout.set(0);
    };

    void startTransition(uint32_t milliseconds, float to_hue, float to_saturation, float to_intensity) {
        _transition_steps_left = 0.5 + (milliseconds / _update_timeout_ms);
        float h                = 1.0 / _transition_steps_left;
        float h_2              = h * h;

        // to_hue needs to be the closest transition
        if (fabs(hue - to_hue) > fabs(hue - (360.0 + to_hue))) {
            to_hue += 360.0;
        }

        _hue_fd_1 = h_2 * (hue - to_hue);
        _hue_fd_0 = 2 * _hue_fd_1;

        _saturation_fd_1 = h_2 * (saturation - to_saturation);
        _saturation_fd_0 = 2 * _saturation_fd_1;

        _intensity_fd_1 = h_2 * (intensity - to_intensity);
        _intensity_fd_0 = 2 * _intensity_fd_1;

        _update_timeout.set(_update_timeout_ms);
    };

    bool update() {
        if (!_update_timeout.isPast()) {
            return false;
        }
        if (_transition_steps_left) {
            hue = hue + _hue_fd_1;
            _hue_fd_1 += _hue_fd_0;

            saturation = saturation + _saturation_fd_1;
            _saturation_fd_1 += _saturation_fd_0;

            intensity = intensity + _intensity_fd_1;
            _intensity_fd_1 += _intensity_fd_0;

            --_transition_steps_left;
            _update_timeout.set(_update_timeout_ms);
            return true;
        }
        return false;
    };

    bool isTransitionDone() { return _transition_steps_left == 0; };

    // Inspired by http://blog.saikoled.com/post/43693602826/why-every-led-light-should-be-using-hsi
    void getRGB(uint8_t& r, uint8_t& g, uint8_t& b) {
        float cos_h, cos_1047_h;

        // check ranges
        while (hue > 360.0) { hue = hue - 360.0; }
        while (hue < 0.0) { hue = 360.0 + hue; }
        if (saturation > 1.0) {
            saturation = 1.0;
        }
        if (saturation < 0.0) {
            saturation = 0.0;
        }
        if (intensity > 1.0) {
            saturation = 1.0;
        }
        if (intensity < 0.0) {
            saturation = 0.0;
        }

        float H = 3.14159 * hue / (float)180.0;  // Convert to radians.
        float S = saturation;                    // clamp S and I to interval [0,1]
        float I = intensity;

        if (H < 2.09439) {
            cos_h      = cos(H);
            cos_1047_h = cos(1.047196667 - H);
            r          = 255 * I / 3 * (1 + S * cos_h / cos_1047_h);
            g          = 255 * I / 3 * (1 + S * (1 - cos_h / cos_1047_h));
            b          = 255 * I / 3 * (1 - S);
        } else if (H < 4.188787) {
            cos_h      = cos(H);
            cos_1047_h = cos(1.047196667 - H);
            H          = H - 2.09439;
            g          = 255 * I / 3 * (1 + S * cos_h / cos_1047_h);
            b          = 255 * I / 3 * (1 + S * (1 - cos_h / cos_1047_h));
            r          = 255 * I / 3 * (1 - S);
        } else {
            cos_h      = cos(H);
            cos_1047_h = cos(1.047196667 - H);
            H          = H - 4.188787;
            b          = 255 * I / 3 * (1 + S * cos_h / cos_1047_h);
            r          = 255 * I / 3 * (1 + S * (1 - cos_h / cos_1047_h));
            g          = 255 * I / 3 * (1 - S);
        }
    };

    // Inspired by http://blog.saikoled.com/post/44677718712/how-to-convert-from-hsi-to-rgb-white
    void getRGBW(uint8_t& r, uint8_t& g, uint8_t& b, uint8_t& w) {
        float cos_h, cos_1047_h;

        // check ranges
        while (hue > 360.0) { hue = hue - 360.0; }
        while (hue < 0.0) { hue = 360.0 + hue; }
        if (saturation > 1.0) {
            saturation = 1.0;
        }
        if (saturation < 0.0) {
            saturation = 0.0;
        }
        if (intensity > 1.0) {
            saturation = 1.0;
        }
        if (intensity < 0.0) {
            saturation = 0.0;
        }

        float H = 3.14159 * hue / (float)180.0;  // Convert to radians.
        float S = saturation;                    // clamp S and I to interval [0,1]
        float I = intensity;

        if (H < 2.09439) {
            cos_h      = cos(H);
            cos_1047_h = cos(1.047196667 - H);
            r          = S * 255 * I / 3 * (1 + cos_h / cos_1047_h);
            g          = S * 255 * I / 3 * (1 + (1 - cos_h / cos_1047_h));
            b          = 0;
            w          = 255 * (1 - S) * I;
        } else if (H < 4.188787) {
            H          = H - 2.09439;
            cos_h      = cos(H);
            cos_1047_h = cos(1.047196667 - H);
            g          = S * 255 * I / 3 * (1 + cos_h / cos_1047_h);
            b          = S * 255 * I / 3 * (1 + (1 - cos_h / cos_1047_h));
            r          = 0;
            w          = 255 * (1 - S) * I;
        } else {
            H          = H - 4.188787;
            cos_h      = cos(H);
            cos_1047_h = cos(1.047196667 - H);
            b          = S * 255 * I / 3 * (1 + cos_h / cos_1047_h);
            r          = S * 255 * I / 3 * (1 + (1 - cos_h / cos_1047_h));
            g          = 0;
            w          = 255 * (1 - S) * I;
        }
    };
};


struct RGB_Color_t : NeopixelColorTag {
    float red;
    float green;
    float blue;

    enum ColorFilter {
        Set = 0,
        Lighten,
        Darken
    };

    const uint32_t _update_timeout_ms;

    Motate::Timeout _update_timeout;

    uint32_t _transition_steps_left = 0;

    // forward difference values
    float _red_fd_0, _red_fd_1;
    float _green_fd_0, _green_fd_1;
    float _blue_fd_0, _blue_fd_1;

    RGB_Color_t(uint32_t _update_every_ms = 1) : _update_timeout_ms{_update_every_ms} { _update_timeout.set(0); };

    RGB_Color_t(float _red, float _green, float _blue, uint32_t _update_every_ms = 1)
        : red{_red}, green{_green}, blue{_blue}, _update_timeout_ms{_update_every_ms} {
        _update_timeout.set(0);
    };

    void startTransition(uint32_t milliseconds, float to_red, float to_green, float to_blue, ColorFilter cf = Set) {
        if (cf == Lighten) {
            to_red = std::max(to_red, red);
            to_green = std::max(to_green, green);
            to_blue = std::max(to_blue, blue);
        }
        else if (cf == Darken) {
            to_red = std::min(to_red, red);
            to_green = std::min(to_green, green);
            to_blue = std::min(to_blue, blue);
        }

        _transition_steps_left = 0.5 + (milliseconds / _update_timeout_ms);
        float h                = 1.0 / _transition_steps_left;
        float h_2              = h * h;

        // to_hue needs to be the closest transition

        _red_fd_1 = h_2 * (to_red - red);
        _red_fd_0 = 2 * _red_fd_1;

        _green_fd_1 = h_2 * (to_green - green);
        _green_fd_0 = 2 * _green_fd_1;

        _blue_fd_1 = h_2 * (to_blue - blue);
        _blue_fd_0 = 2 * _blue_fd_1;

        _update_timeout.set(0);
    };

    bool update() {
        if (!_update_timeout.isPast()) {
            return false;
        }
        if (_transition_steps_left) {
            red = red + _red_fd_1;
            _red_fd_1 += _red_fd_0;
            if (red > 1.0) {
                red = 1.0;
            } else if (red < 0.0) {
                red = 0.0;
            }

            green = green + _green_fd_1;
            _green_fd_1 += _green_fd_0;
            if (green > 1.0) {
                green = 1.0;
            } else if (green < 0.0) {
                green = 0.0;
            }

            blue = blue + _blue_fd_1;
            _blue_fd_1 += _blue_fd_0;
            if (blue > 1.0) {
                blue = 1.0;
            } else if (blue < 0.0) {
                blue = 0.0;
            }

            --_transition_steps_left;
            _update_timeout.set(_update_timeout_ms);
            return true;
        }
        return false;
    };

    bool isTransitionDone() { return _transition_steps_left == 0; };

    void getRGB(float& r, float& g, float& b) {
        r = red;
        g = green;
        b = blue;
    };

    void getRGB(uint8_t& r, uint8_t& g, uint8_t& b) {
        r = red * 255;
        g = green * 255;
        b = blue * 255;
    };

    void getRGBW(uint8_t& r, uint8_t& g, uint8_t& b, uint8_t& w) {
        float white = std::min(red, std::min(green, blue));

        r = (red)*255;
        g = (green)*255;
        b = (blue)*255;
        w = white * 255;

        //        r = (red-white) * 255;
        //        g = (green-white) * 255;
        //        b = (blue-white) * 255;
        //        w = white * 255;
    };
};


#pragma mark NeoPixel object and supporting enums

enum class NeoPixelOrder : uint32_t {
    // RGB NeoPixel permutations; white offset == red offset
    // Offset:   W          R          G     B
    RGB = ((0 << 6) | (0 << 4) | (1 << 2) | (2)),
    RBG = ((0 << 6) | (0 << 4) | (2 << 2) | (1)),
    GRB = ((1 << 6) | (1 << 4) | (0 << 2) | (2)),
    GBR = ((2 << 6) | (2 << 4) | (0 << 2) | (1)),
    BRG = ((1 << 6) | (1 << 4) | (2 << 2) | (0)),
    BGR = ((2 << 6) | (2 << 4) | (1 << 2) | (0)),

    // RGBW NeoPixel permutations; all 4 offsets are distinct
    // Offset:    W          R          G     B
    WRGB = ((0 << 6) | (1 << 4) | (2 << 2) | (3)),
    WRBG = ((0 << 6) | (1 << 4) | (3 << 2) | (2)),
    WGRB = ((0 << 6) | (2 << 4) | (1 << 2) | (3)),
    WGBR = ((0 << 6) | (3 << 4) | (1 << 2) | (2)),
    WBRG = ((0 << 6) | (2 << 4) | (3 << 2) | (1)),
    WBGR = ((0 << 6) | (3 << 4) | (2 << 2) | (1)),

    RWGB = ((1 << 6) | (0 << 4) | (2 << 2) | (3)),
    RWBG = ((1 << 6) | (0 << 4) | (3 << 2) | (2)),
    RGWB = ((2 << 6) | (0 << 4) | (1 << 2) | (3)),
    RGBW = ((3 << 6) | (0 << 4) | (1 << 2) | (2)),
    RBWG = ((2 << 6) | (0 << 4) | (3 << 2) | (1)),
    RBGW = ((3 << 6) | (0 << 4) | (2 << 2) | (1)),

    GWRB = ((1 << 6) | (2 << 4) | (0 << 2) | (3)),
    GWBR = ((1 << 6) | (3 << 4) | (0 << 2) | (2)),
    GRWB = ((2 << 6) | (1 << 4) | (0 << 2) | (3)),
    GRBW = ((3 << 6) | (1 << 4) | (0 << 2) | (2)),
    GBWR = ((2 << 6) | (3 << 4) | (0 << 2) | (1)),
    GBRW = ((3 << 6) | (2 << 4) | (0 << 2) | (1)),

    BWRG = ((1 << 6) | (2 << 4) | (3 << 2) | (0)),
    BWGR = ((1 << 6) | (3 << 4) | (2 << 2) | (0)),
    BRWG = ((2 << 6) | (1 << 4) | (3 << 2) | (0)),
    BRGW = ((3 << 6) | (1 << 4) | (2 << 2) | (0)),
    BGWR = ((2 << 6) | (3 << 4) | (1 << 2) | (0)),
    BGRW = ((3 << 6) | (2 << 4) | (1 << 2) | (0)),
};

// Usage: NeoPixel<Motate::kLED_RGBWPixelPinNumber, 3> pixels {NeoPixelOrder::GRBW};
template <Motate::pin_number pixel_pin_pumber, uint8_t pixel_count, uint32_t base_frequency = 800000>
struct NeoPixel {
    static constexpr uint8_t count = pixel_count;

    // This is heavily borrowed from the defines at:
    // https://github.com/adafruit/Adafruit_NeoPixel/blob/master/Adafruit_NeoPixel.h
    // The defines are converted to enums for name pollution prevention, type safety,
    // and for readbility.

    const NeoPixelOrder _pixel_order;

    const uint32_t _white_offset;
    const uint32_t _red_offset;
    const uint32_t _green_offset;
    const uint32_t _blue_offset;
    const bool     _has_white;

    Motate::PWMOutputPin<pixel_pin_pumber> _pixel_pin;

    // Note: 0 = 1/4 on-time
    //       1 = 1/2 on-time
    const uint16_t led_ON    = _pixel_pin.getTopValue() >> 1;
    const uint16_t led_OFF   = _pixel_pin.getTopValue() >> 2;
    const uint16_t led_RESET = 0;

    // 1 bit of "buffer"
    // 32 bits per pixel
    // 1 bit to turn the PWM off
    uint16_t _period_buffer[1 + 32 * pixel_count + 1];

    Motate::Timeout _update_timeout;
    const uint32_t  _update_timeout_ms;
    bool            _pixels_changed = true;

    constexpr NeoPixel(NeoPixelOrder new_order, uint32_t update_ms = 1)
        : _pixel_order{new_order},
          _white_offset{(((uint32_t)_pixel_order >> 6) & 0b11) << 3},
          _red_offset{(((uint32_t)_pixel_order >> 4) & 0b11) << 3},
          _green_offset{(((uint32_t)_pixel_order >> 2) & 0b11) << 3},
          _blue_offset{((uint32_t)_pixel_order & 0b11) << 3},
          _has_white{(_white_offset != _red_offset)},
          _pixel_pin{Motate::kNormal, base_frequency},
          _update_timeout_ms{update_ms} {

        _pixel_pin = 0.0;  // start at 0
        _pixel_pin.stop();

        // Set it to sync via DMA
        _pixel_pin.setSync(true);
        // ... on every period
        _pixel_pin.setSyncMode(Motate::kTimerSyncDMA, 1);

        _update_timeout.set(0);
    };

    void setPixel(uint8_t pixel, uint8_t red, uint8_t green, uint8_t blue, int16_t white = -1) {
        uint8_t data_width = 32;
        if (!_has_white) {
            data_width = 24;
        }
        if (_has_white && (white == -1)) {
            // Adjust all of the RGB to accomodate white
            white = std::min(red, std::min(green, blue));
            // red -= white;
            // green -= white;
            // blue -= white;
        }
        _period_buffer[0 + 1 + _red_offset + (pixel * data_width)] = (red & 0b10000000) ? led_ON : led_OFF;
        _period_buffer[1 + 1 + _red_offset + (pixel * data_width)] = (red & 0b01000000) ? led_ON : led_OFF;
        _period_buffer[2 + 1 + _red_offset + (pixel * data_width)] = (red & 0b00100000) ? led_ON : led_OFF;
        _period_buffer[3 + 1 + _red_offset + (pixel * data_width)] = (red & 0b00010000) ? led_ON : led_OFF;
        _period_buffer[4 + 1 + _red_offset + (pixel * data_width)] = (red & 0b00001000) ? led_ON : led_OFF;
        _period_buffer[5 + 1 + _red_offset + (pixel * data_width)] = (red & 0b00000100) ? led_ON : led_OFF;
        _period_buffer[6 + 1 + _red_offset + (pixel * data_width)] = (red & 0b00000010) ? led_ON : led_OFF;
        _period_buffer[7 + 1 + _red_offset + (pixel * data_width)] = (red & 0b00000001) ? led_ON : led_OFF;

        _period_buffer[0 + 1 + _green_offset + (pixel * data_width)] = (green & 0b10000000) ? led_ON : led_OFF;
        _period_buffer[1 + 1 + _green_offset + (pixel * data_width)] = (green & 0b01000000) ? led_ON : led_OFF;
        _period_buffer[2 + 1 + _green_offset + (pixel * data_width)] = (green & 0b00100000) ? led_ON : led_OFF;
        _period_buffer[3 + 1 + _green_offset + (pixel * data_width)] = (green & 0b00010000) ? led_ON : led_OFF;
        _period_buffer[4 + 1 + _green_offset + (pixel * data_width)] = (green & 0b00001000) ? led_ON : led_OFF;
        _period_buffer[5 + 1 + _green_offset + (pixel * data_width)] = (green & 0b00000100) ? led_ON : led_OFF;
        _period_buffer[6 + 1 + _green_offset + (pixel * data_width)] = (green & 0b00000010) ? led_ON : led_OFF;
        _period_buffer[7 + 1 + _green_offset + (pixel * data_width)] = (green & 0b00000001) ? led_ON : led_OFF;

        _period_buffer[0 + 1 + _blue_offset + (pixel * data_width)] = (blue & 0b10000000) ? led_ON : led_OFF;
        _period_buffer[1 + 1 + _blue_offset + (pixel * data_width)] = (blue & 0b01000000) ? led_ON : led_OFF;
        _period_buffer[2 + 1 + _blue_offset + (pixel * data_width)] = (blue & 0b00100000) ? led_ON : led_OFF;
        _period_buffer[3 + 1 + _blue_offset + (pixel * data_width)] = (blue & 0b00010000) ? led_ON : led_OFF;
        _period_buffer[4 + 1 + _blue_offset + (pixel * data_width)] = (blue & 0b00001000) ? led_ON : led_OFF;
        _period_buffer[5 + 1 + _blue_offset + (pixel * data_width)] = (blue & 0b00000100) ? led_ON : led_OFF;
        _period_buffer[6 + 1 + _blue_offset + (pixel * data_width)] = (blue & 0b00000010) ? led_ON : led_OFF;
        _period_buffer[7 + 1 + _blue_offset + (pixel * data_width)] = (blue & 0b00000001) ? led_ON : led_OFF;

        if (_has_white) {
            _period_buffer[0 + 1 + _white_offset + (pixel * 32)] = (white & 0b10000000) ? led_ON : led_OFF;
            _period_buffer[1 + 1 + _white_offset + (pixel * 32)] = (white & 0b01000000) ? led_ON : led_OFF;
            _period_buffer[2 + 1 + _white_offset + (pixel * 32)] = (white & 0b00100000) ? led_ON : led_OFF;
            _period_buffer[3 + 1 + _white_offset + (pixel * 32)] = (white & 0b00010000) ? led_ON : led_OFF;
            _period_buffer[4 + 1 + _white_offset + (pixel * 32)] = (white & 0b00001000) ? led_ON : led_OFF;
            _period_buffer[5 + 1 + _white_offset + (pixel * 32)] = (white & 0b00000100) ? led_ON : led_OFF;
            _period_buffer[6 + 1 + _white_offset + (pixel * 32)] = (white & 0b00000010) ? led_ON : led_OFF;
            _period_buffer[7 + 1 + _white_offset + (pixel * 32)] = (white & 0b00000001) ? led_ON : led_OFF;
        }

        _pixels_changed = true;
    };

    template <
        typename color_t,
        typename std::enable_if<std::is_base_of<NeopixelColorTag, color_t>::value, NeopixelColorTag>::type* = nullptr>
    void setPixel(uint8_t pixel, color_t& color) {
        if (_has_white) {
            uint8_t red, green, blue, white;
            color.getRGBW(red, green, blue, white);
            setPixel(pixel, red, green, blue, white);
        } else {
            uint8_t red, green, blue;
            color.getRGB(red, green, blue);
            setPixel(pixel, red, green, blue);
        }
    }

    void update() {
        if (!_pixel_pin.isTransferDone()) {
            return;
        }
        //        if (!_pixels_changed) { return; }
        //        _pixels_changed = false;

        if (_update_timeout.isPast()) {
            _pixel_pin.startTransfer(_period_buffer);
            _update_timeout.set(_update_timeout_ms);
        }
    }
};

#endif  // NEOPIXEL_DRIVER_H_ONCE
