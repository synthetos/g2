/*
 * util.cpp - a random assortment of useful functions
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
/* util contains a dog's breakfast of supporting functions that are not specific to g2core:
 * including:
 *  - math and min/max utilities and extensions
 *  - vector manipulation utilities
 */

#include "g2core.h"
#include "util.h"

bool FLAGS_NONE[AXES] = { false, false, false, false, false, false };
bool FLAGS_ONE[AXES]  = { true, false, false, false, false, false };
bool FLAGS_ALL[AXES]  = { true, true, true, true, true, true };

//*** debug utilities ***

/**** Vector utilities ****
 * copy_vector()            - copy vector of arbitrary length
 * vector_equal()           - test if vectors are equal
 * get_axis_vector_length(  - return the length of an axis vector
 * set_vector()             - load values into vector form
 * set_vector_by_axis()     - load a single value into a zero vector
 */

float vector[AXES];    // statically allocated global for vector utilities

/*
void copy_vector(float dst[], const float src[])
{
    memcpy(dst, src, sizeof(dst));
}
*/

uint8_t vector_equal(const float a[], const float b[])
{
    if ((fp_EQ(a[AXIS_X], b[AXIS_X])) &&
        (fp_EQ(a[AXIS_Y], b[AXIS_Y])) &&
        (fp_EQ(a[AXIS_Z], b[AXIS_Z])) &&
        (fp_EQ(a[AXIS_A], b[AXIS_A])) &&
        (fp_EQ(a[AXIS_B], b[AXIS_B])) &&
        (fp_EQ(a[AXIS_C], b[AXIS_C]))) {
        return (true);
    }
    return (false);
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
        case (AXIS_X): vector[AXIS_X] = value; break;
        case (AXIS_Y): vector[AXIS_Y] = value; break;
        case (AXIS_Z): vector[AXIS_Z] = value; break;
        case (AXIS_A): vector[AXIS_A] = value; break;
        case (AXIS_B): vector[AXIS_B] = value; break;
        case (AXIS_C): vector[AXIS_C] = value;
    }
    return (vector);
}

/**** Math and other general purpose functions ****/

/* Slightly faster (*) multi-value min and max functions
 *   min3() - return minimum of 3 numbers
 *   min4() - return minimum of 4 numbers
 *   max3() - return maximum of 3 numbers
 *   max4() - return maximum of 4 numbers
 *
 * Implementation tip: Order the min and max values from most to least likely in the calling args
 *
 * (*) Macro min4 is about 20uSec, inline function version is closer to 10 uSec (Xmega 32 MHz)
 *    #define min3(a,b,c) (min(min(a,b),c))
 *    #define min4(a,b,c,d) (min(min(a,b),min(c,d)))
 *    #define max3(a,b,c) (max(max(a,b),c))
 *    #define max4(a,b,c,d) (max(max(a,b),max(c,d)))
 */

float min3(float x1, float x2, float x3)
{
    float min = x1;
    if (x2 < min) { min = x2;}
    if (x3 < min) { return (x3);}
    return (min);
}

float min4(float x1, float x2, float x3, float x4)
{
    float min = x1;
    if (x2 < min) { min = x2;}
    if (x3 < min) { min = x3;}
    if (x4 < min) { return (x4);}
    return (min);
}

float max3(float x1, float x2, float x3)
{
    float max = x1;
    if (x2 > max) { max = x2;}
    if (x3 > max) { return (x3);}
    return (max);
}

float max4(float x1, float x2, float x3, float x4)
{
    float max = x1;
    if (x2 > max) { max = x2;}
    if (x3 > max) { max = x3;}
    if (x4 > max) { return (x4);}
    return (max);
}

/**** String utilities ****
 * strcpy_U()      - strcpy workalike to get around initial NUL for blank string - possibly wrong
 * isnumber()      - isdigit that also accepts plus, minus, and decimal point
 * escape_string() - add escapes to a string - currently for quotes only
 */

/*
uint8_t * strcpy_U( uint8_t * dst, const uint8_t * src )
{
    uint16_t index = 0;
    do {
        dst[index] = src[index];
    } while (src[index++] != 0);
    return dst;
}
*/

uint8_t isnumber(char c)
{
    if (c == '.') { return (true); }
    if (c == '-') { return (true); }
    if (c == '+') { return (true); }
    return (isdigit(c));
}

char *escape_string(char *dst, char *src)
{
    char c;
    char *start_dst = dst;

    while ((c = *(src++)) != 0) {           // NUL
        if (c == '"') { *(dst++) = '\\'; }
        if (c == 0x0d) { continue; }        // CR happens in some pathological malformed input cases
        if (c == 0x0a) { continue; }        // LF happens in some pathological malformed input cases
        *(dst++) = c;
    }
    *dst = 0;
    return (start_dst);
}

/*
 * fntoa() - return ASCII string given a float and a decimal precision value
 *
 *    Like sprintf, fntoa returns length of string, less the terminating NUL character
 */
char fntoa(char *str, float n, uint8_t precision)
{
    // handle special cases
    if (isnan(n)) {
        strcpy(str, "nan");
        return (3);

    } else if (isinf(n)) {
        strcpy(str, "inf");
        return (3);

    } else if (precision == 0 ) { return(sprintf(str, "%0.0f", (double) n));
    } else if (precision == 1 ) { return(sprintf(str, "%0.1f", (double) n));
    } else if (precision == 2 ) { return(sprintf(str, "%0.2f", (double) n));
    } else if (precision == 3 ) { return(sprintf(str, "%0.3f", (double) n));
    } else if (precision == 4 ) { return(sprintf(str, "%0.4f", (double) n));
    } else if (precision == 5 ) { return(sprintf(str, "%0.5f", (double) n));
    } else if (precision == 6 ) { return(sprintf(str, "%0.6f", (double) n));
    } else if (precision == 7 ) { return(sprintf(str, "%0.7f", (double) n));
    } else                        { return(sprintf(str, "%f", (double) n)); }
}

/*
 * compute_checksum() - calculate the checksum for a string
 *
 *  Stops calculation on null termination or length value if non-zero.
 *
 *  This is based on the the Java hashCode function.
 *  See http://en.wikipedia.org/wiki/Java_hashCode()
 */
#define HASHMASK 9999

uint16_t compute_checksum(char const *string, const uint16_t length)
{
    uint32_t h = 0;
    uint16_t len = strlen(string);
    if (length != 0) len = min(len, length);
    for (uint16_t i=0; i<len; i++) {
        h = 31 * h + string[i];
    }
    return (h % HASHMASK);
}

/*
 * SysTickTimer_getValue() - this is a hack to get around some compatibility problems
 */

uint32_t SysTickTimer_getValue()
{
    return (SysTickTimer.getValue());
}

/***********************************************
 **** Very Fast Number to ASCII Conversions ****
 ***********************************************/
/*
 * inttoa() - integer to ASCII
 *
 *  Taking advantage of the fact that most ints we display are 8 bit quantities,
 *  and we have plenty of FLASH
 */
// static ASCII numbers
static const char itoa_00[] = "0";
static const char itoa_01[] = "1";
static const char itoa_02[] = "2";
static const char itoa_03[] = "3";
static const char itoa_04[] = "4";
static const char itoa_05[] = "5";
static const char itoa_06[] = "6";
static const char itoa_07[] = "7";
static const char itoa_08[] = "8";
static const char itoa_09[] = "9";
static const char itoa_10[] = "10";
static const char itoa_11[] = "11";
static const char itoa_12[] = "12";
static const char itoa_13[] = "13";
static const char itoa_14[] = "14";
static const char itoa_15[] = "15";
static const char itoa_16[] = "16";
static const char itoa_17[] = "17";
static const char itoa_18[] = "18";
static const char itoa_19[] = "19";
static const char itoa_20[] = "20";
static const char itoa_21[] = "21";
static const char itoa_22[] = "22";
static const char itoa_23[] = "23";
static const char itoa_24[] = "24";
static const char itoa_25[] = "25";
static const char itoa_26[] = "26";
static const char itoa_27[] = "27";
static const char itoa_28[] = "28";
static const char itoa_29[] = "29";
static const char itoa_30[] = "30";
static const char itoa_31[] = "31";

static const char itoa_32[] = "32";
static const char itoa_33[] = "33";
static const char itoa_34[] = "34";
static const char itoa_35[] = "35";
static const char itoa_36[] = "36";
static const char itoa_37[] = "37";
static const char itoa_38[] = "38";
static const char itoa_39[] = "39";
static const char itoa_40[] = "40";
static const char itoa_41[] = "41";
static const char itoa_42[] = "42";
static const char itoa_43[] = "43";
static const char itoa_44[] = "44";
static const char itoa_45[] = "45";
static const char itoa_46[] = "46";
static const char itoa_47[] = "47";
static const char itoa_48[] = "48";
static const char itoa_49[] = "49";
static const char itoa_50[] = "50";
static const char itoa_51[] = "51";
static const char itoa_52[] = "52";
static const char itoa_53[] = "53";
static const char itoa_54[] = "54";
static const char itoa_55[] = "55";
static const char itoa_56[] = "56";
static const char itoa_57[] = "57";
static const char itoa_58[] = "58";
static const char itoa_59[] = "59";
static const char itoa_60[] = "60";
static const char itoa_61[] = "61";
static const char itoa_62[] = "62";
static const char itoa_63[] = "63";

static const char itoa_64[] = "64";
static const char itoa_65[] = "65";
static const char itoa_66[] = "66";
static const char itoa_67[] = "67";
static const char itoa_68[] = "68";
static const char itoa_69[] = "69";
static const char itoa_70[] = "70";
static const char itoa_71[] = "71";
static const char itoa_72[] = "72";
static const char itoa_73[] = "73";
static const char itoa_74[] = "74";
static const char itoa_75[] = "75";
static const char itoa_76[] = "76";
static const char itoa_77[] = "77";
static const char itoa_78[] = "78";
static const char itoa_79[] = "79";
static const char itoa_80[] = "80";
static const char itoa_81[] = "81";
static const char itoa_82[] = "82";
static const char itoa_83[] = "83";
static const char itoa_84[] = "84";
static const char itoa_85[] = "85";
static const char itoa_86[] = "86";
static const char itoa_87[] = "87";
static const char itoa_88[] = "88";
static const char itoa_89[] = "89";
static const char itoa_90[] = "90";
static const char itoa_91[] = "91";
static const char itoa_92[] = "92";
static const char itoa_93[] = "93";
static const char itoa_94[] = "94";
static const char itoa_95[] = "95";

static const char itoa_96[] = "96";
static const char itoa_97[] = "97";
static const char itoa_98[] = "98";
static const char itoa_99[] = "99";
static const char itoa_100[] = "100";
static const char itoa_101[] = "101";
static const char itoa_102[] = "102";
static const char itoa_103[] = "103";
static const char itoa_104[] = "104";
static const char itoa_105[] = "105";
static const char itoa_106[] = "106";
static const char itoa_107[] = "107";
static const char itoa_108[] = "108";
static const char itoa_109[] = "109";
static const char itoa_110[] = "110";
static const char itoa_111[] = "111";
static const char itoa_112[] = "112";
static const char itoa_113[] = "113";
static const char itoa_114[] = "114";
static const char itoa_115[] = "115";
static const char itoa_116[] = "116";
static const char itoa_117[] = "117";
static const char itoa_118[] = "118";
static const char itoa_119[] = "119";
static const char itoa_120[] = "120";
static const char itoa_121[] = "121";
static const char itoa_122[] = "122";
static const char itoa_123[] = "123";
static const char itoa_124[] = "124";
static const char itoa_125[] = "125";
static const char itoa_126[] = "126";
static const char itoa_127[] = "127";

static const char itoa_128[] = "128";
static const char itoa_129[] = "129";
static const char itoa_130[] = "130";
static const char itoa_131[] = "131";
static const char itoa_132[] = "132";
static const char itoa_133[] = "133";
static const char itoa_134[] = "134";
static const char itoa_135[] = "135";
static const char itoa_136[] = "136";
static const char itoa_137[] = "137";
static const char itoa_138[] = "138";
static const char itoa_139[] = "139";
static const char itoa_140[] = "140";
static const char itoa_141[] = "141";
static const char itoa_142[] = "142";
static const char itoa_143[] = "143";
static const char itoa_144[] = "144";
static const char itoa_145[] = "145";
static const char itoa_146[] = "146";
static const char itoa_147[] = "147";
static const char itoa_148[] = "148";
static const char itoa_149[] = "149";
static const char itoa_150[] = "150";
static const char itoa_151[] = "151";
static const char itoa_152[] = "152";
static const char itoa_153[] = "153";
static const char itoa_154[] = "154";
static const char itoa_155[] = "155";
static const char itoa_156[] = "156";
static const char itoa_157[] = "157";
static const char itoa_158[] = "158";
static const char itoa_159[] = "159";

static const char itoa_160[] = "160";
static const char itoa_161[] = "161";
static const char itoa_162[] = "162";
static const char itoa_163[] = "163";
static const char itoa_164[] = "164";
static const char itoa_165[] = "165";
static const char itoa_166[] = "166";
static const char itoa_167[] = "167";
static const char itoa_168[] = "168";
static const char itoa_169[] = "169";
static const char itoa_170[] = "170";
static const char itoa_171[] = "171";
static const char itoa_172[] = "172";
static const char itoa_173[] = "173";
static const char itoa_174[] = "174";
static const char itoa_175[] = "175";
static const char itoa_176[] = "176";
static const char itoa_177[] = "177";
static const char itoa_178[] = "178";
static const char itoa_179[] = "179";
static const char itoa_180[] = "180";
static const char itoa_181[] = "181";
static const char itoa_182[] = "182";
static const char itoa_183[] = "183";
static const char itoa_184[] = "184";
static const char itoa_185[] = "185";
static const char itoa_186[] = "186";
static const char itoa_187[] = "187";
static const char itoa_188[] = "188";
static const char itoa_189[] = "189";
static const char itoa_190[] = "190";
static const char itoa_191[] = "191";

static const char itoa_192[] = "192";
static const char itoa_193[] = "193";
static const char itoa_194[] = "194";
static const char itoa_195[] = "195";
static const char itoa_196[] = "196";
static const char itoa_197[] = "197";
static const char itoa_198[] = "198";
static const char itoa_199[] = "199";
static const char itoa_200[] = "200";
static const char itoa_201[] = "201";
static const char itoa_202[] = "202";
static const char itoa_203[] = "203";
static const char itoa_204[] = "204";
static const char itoa_205[] = "205";
static const char itoa_206[] = "206";
static const char itoa_207[] = "207";
static const char itoa_208[] = "208";
static const char itoa_209[] = "209";
static const char itoa_210[] = "210";
static const char itoa_211[] = "211";
static const char itoa_212[] = "212";
static const char itoa_213[] = "213";
static const char itoa_214[] = "214";
static const char itoa_215[] = "215";
static const char itoa_216[] = "216";
static const char itoa_217[] = "217";
static const char itoa_218[] = "218";
static const char itoa_219[] = "219";
static const char itoa_220[] = "220";
static const char itoa_221[] = "221";
static const char itoa_222[] = "222";
static const char itoa_223[] = "223";

static const char itoa_224[] = "224";
static const char itoa_225[] = "225";
static const char itoa_226[] = "226";
static const char itoa_227[] = "227";
static const char itoa_228[] = "228";
static const char itoa_229[] = "229";
static const char itoa_230[] = "230";
static const char itoa_231[] = "231";
static const char itoa_232[] = "232";
static const char itoa_233[] = "233";
static const char itoa_234[] = "234";
static const char itoa_235[] = "235";
static const char itoa_236[] = "236";
static const char itoa_237[] = "237";
static const char itoa_238[] = "238";
static const char itoa_239[] = "239";
static const char itoa_240[] = "240";
static const char itoa_241[] = "241";
static const char itoa_242[] = "242";
static const char itoa_243[] = "243";
static const char itoa_244[] = "244";
static const char itoa_245[] = "245";
static const char itoa_246[] = "246";
static const char itoa_247[] = "247";
static const char itoa_248[] = "248";
static const char itoa_249[] = "249";
static const char itoa_250[] = "250";
static const char itoa_251[] = "251";
static const char itoa_252[] = "252";
static const char itoa_253[] = "253";
static const char itoa_254[] = "254";
static const char itoa_255[] = "255";

static const char *const itoa_str[] = {
    itoa_00, itoa_01, itoa_02, itoa_03, itoa_04, itoa_05, itoa_06, itoa_07, itoa_08, itoa_09,
    itoa_10, itoa_11, itoa_12, itoa_13, itoa_14, itoa_15, itoa_16, itoa_17, itoa_18, itoa_19,
    itoa_20, itoa_21, itoa_22, itoa_23, itoa_24, itoa_25, itoa_26, itoa_27, itoa_28, itoa_29,
    itoa_30, itoa_31, itoa_32, itoa_33, itoa_34, itoa_35, itoa_36, itoa_37, itoa_38, itoa_39,
    itoa_40, itoa_41, itoa_42, itoa_43, itoa_44, itoa_45, itoa_46, itoa_47, itoa_48, itoa_49,
    itoa_50, itoa_51, itoa_52, itoa_53, itoa_54, itoa_55, itoa_56, itoa_57, itoa_58, itoa_59,
    itoa_60, itoa_61, itoa_62, itoa_63, itoa_64, itoa_65, itoa_66, itoa_67, itoa_68, itoa_69,
    itoa_70, itoa_71, itoa_72, itoa_73, itoa_74, itoa_75, itoa_76, itoa_77, itoa_78, itoa_79,
    itoa_80, itoa_81, itoa_82, itoa_83, itoa_84, itoa_85, itoa_86, itoa_87, itoa_88, itoa_89,
    itoa_90, itoa_91, itoa_92, itoa_93, itoa_94, itoa_95, itoa_96, itoa_97, itoa_98, itoa_99,
    itoa_100, itoa_101, itoa_102, itoa_103, itoa_104, itoa_105, itoa_106, itoa_107, itoa_108, itoa_109,
    itoa_110, itoa_111, itoa_112, itoa_113, itoa_114, itoa_115, itoa_116, itoa_117, itoa_118, itoa_119,
    itoa_120, itoa_121, itoa_122, itoa_123, itoa_124, itoa_125, itoa_126, itoa_127, itoa_128, itoa_129,
    itoa_130, itoa_131, itoa_132, itoa_133, itoa_134, itoa_135, itoa_136, itoa_137, itoa_138, itoa_139,
    itoa_140, itoa_141, itoa_142, itoa_143, itoa_144, itoa_145, itoa_146, itoa_147, itoa_148, itoa_149,
    itoa_150, itoa_151, itoa_152, itoa_153, itoa_154, itoa_155, itoa_156, itoa_157, itoa_158, itoa_159,
    itoa_160, itoa_161, itoa_162, itoa_163, itoa_164, itoa_165, itoa_166, itoa_167, itoa_168, itoa_169,
    itoa_170, itoa_171, itoa_172, itoa_173, itoa_174, itoa_175, itoa_176, itoa_177, itoa_178, itoa_179,
    itoa_180, itoa_181, itoa_182, itoa_183, itoa_184, itoa_185, itoa_186, itoa_187, itoa_188, itoa_189,
    itoa_190, itoa_191, itoa_192, itoa_193, itoa_194, itoa_195, itoa_196, itoa_197, itoa_198, itoa_199,
    itoa_200, itoa_201, itoa_202, itoa_203, itoa_204, itoa_205, itoa_206, itoa_207, itoa_208, itoa_209,
    itoa_210, itoa_211, itoa_212, itoa_213, itoa_214, itoa_215, itoa_216, itoa_217, itoa_218, itoa_219,
    itoa_220, itoa_221, itoa_222, itoa_223, itoa_224, itoa_225, itoa_226, itoa_227, itoa_228, itoa_229,
    itoa_230, itoa_231, itoa_232, itoa_233, itoa_234, itoa_235, itoa_236, itoa_237, itoa_238, itoa_239,
    itoa_240, itoa_241, itoa_242, itoa_243, itoa_244, itoa_245, itoa_246, itoa_247, itoa_248, itoa_249,
    itoa_250, itoa_251, itoa_252, itoa_253, itoa_254, itoa_255
};

static int _i2a(char *s, int n)
{
    div_t qr;
    int pos;

    if (n == 0) {
        return 0;
    }

    qr = div(n, 10);
    pos = _i2a(s, qr.quot);
    s[pos] = qr.rem + '0';
    return (pos + 1);
}

char inttoa(char *str, int n)
{
    if (n < 256) {
        strcpy(str, GET_TEXT_ITEM(itoa_str, n));
    } else {
        char *p = str;
        if (n < 0){
            *p++ = '-';
            n *= -1;
        } else if(n == 0) {
            *p++ = '0';
        }
        p[_i2a(p, n)]='\0';
        }
    return (strlen(str));
}

constexpr float round_lookup_[] = {
    0.5,          // precision 0
    0.05,         // precision 1
    0.005,        // precision 2
    0.0005,       // precision 3
    0.00005,      // precision 4
    0.000005,     // precision 5
    0.0000005,    // precision 6
    0.00000005,   // precision 7
    0.000000005,  // precision 8
    0.0000000005, // precision 9
    0.00000000005 // precision 10
};

// It's assumed that the string buffer contains at lest count_ non-\0 chars
int c_strreverse(char * const t, const int count_, char hold = 0) {
    return count_>1
    // Note: It always returns the count_, for a consistent interface.
    ? (hold=*t, *t=*(t+(count_-1)), *(t+(count_-1))=hold), c_strreverse(t+1, count_-2), count_
    : count_;
}

char floattoa(char *buffer, float in, int precision, int maxlen /*= 16*/) {
    int length_ = 0;
    char *b_ = buffer;

    if (in < 0.0) {
        *b_++ = '-';
        return floattoa(b_, -in, precision, maxlen-1) + 1;
    }

    in += round_lookup_[precision];
    int int_length_ = 0;
    int integer_part_ = (int)in;

    // do integer part
    while (integer_part_ > 0) {
        if (length_++ > maxlen) {
            *buffer = 0;
            return 0;
        }
        int t_ = integer_part_ / 10;
        *b_++ = '0' + (integer_part_ - (t_*10));
        integer_part_ = t_;
        int_length_++;
    }
    if (length_ > 0) {
        c_strreverse(buffer, int_length_);
    } else {
        *b_++ = '0';
        int_length_++;
    }

    // do fractional part
    *b_++ = '.';
    length_ = int_length_+1;

    float frac_part_ = in;
    frac_part_ -= (int)frac_part_;
    while (precision-- > 0) {
        if (length_++ > maxlen) {
            *buffer = 0;
            return 0;
        }
        frac_part_ *= 10.0;
        // if (precision==0) {
        //     t_ += 0.5;
        // }
        *b_++ = ('0' + (int)frac_part_);
        frac_part_ -= (int)frac_part_;
    }

    // right strip trailing zeroes (OPTIONAL)
#if 1
    while (*(b_-1) == '0' && length_>1) {
        *(b_--) = 0;
        length_--;
    }

    if (*(b_-1) == '.') {
        *(b_--) = 0;
        length_--;
    }
#endif
    return length_;
}
