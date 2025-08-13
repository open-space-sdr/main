/*
 * math_utils.h
 *
 * fixed-point helpers used by qec, integer-only
 * contains q31/q15 basic ops, conversions, and trig functs via lookup table (faster/better way to do this?)
 */

#ifndef INC_MATH_UTILS_H_
#define INC_MATH_UTILS_H_

#include <stdint.h>

/* q31 */

/**
 * @brief performs saturating q31 addition
 * @param a the first q31 addend
 * @param b the second q31 addend
 * @return the saturated q31 sum
 * // test: q31_add(0x40000000, 0x50000000) should return 0x7FFFFFFF
 */
static inline int32_t q31_add(int32_t a, int32_t b) {
    int64_t s = (int64_t)a + (int64_t)b;
    if (s > 0x7FFFFFFFLL) s = 0x7FFFFFFFLL;
    if (s < (int64_t)0x80000000LL) s = (int64_t)0x80000000LL;
    return (int32_t)s;
}

/**
 * @brief performs saturating q31 subtraction
 * @param a the q31 minuend
 * @param b the q31 subtrahend
 * @return the saturated q31 difference
 * // test: q31_sub(0x80000000, 0x10000000) should return 0x80000000
 */
static inline int32_t q31_sub(int32_t a, int32_t b) {
    int64_t s = (int64_t)a - (int64_t)b;
    if (s > 0x7FFFFFFFLL) s = 0x7FFFFFFFLL;
    if (s < (int64_t)0x80000000LL) s = (int64_t)0x80000000LL;
    return (int32_t)s;
}

/**
 * @brief performs saturating q31 multiplication
 * @param a the first q31 factor
 * @param b the second q31 factor
 * @return the saturated q31 product
 * // test: q31_mul(0x40000000, 0x40000000) should return 0x20000000 (0.5*0.5=0.25)
 */
static inline int32_t q31_mul(int32_t a, int32_t b) {
    int64_t p = (int64_t)a * (int64_t)b;
    p >>= 31;
    if (p > 0x7FFFFFFFLL) p = 0x7FFFFFFFLL;
    if (p < (int64_t)0x80000000LL) p = (int64_t)0x80000000LL;
    return (int32_t)p;
}

/**
 * @brief performs saturating q31 division
 * @param a the q31 numerator
 * @param b the q31 denominator
 * @return the saturated q31 quotient
 * // test: q31_div(0x20000000, 0x40000000) should return 0x40000000 (0.25/0.5=0.5)
 */
static inline int32_t q31_div(int32_t a, int32_t b) {
    if (b == 0) return (a >= 0 ? 0x7FFFFFFF : (int32_t)0x80000000);
    int64_t n = ((int64_t)a << 31);
    int64_t q = n / (int64_t)b;
    if (q > 0x7FFFFFFFLL) q = 0x7FFFFFFFLL;
    if (q < (int64_t)0x80000000LL) q = (int64_t)0x80000000LL;
    return (int32_t)q;
}


/* q15 helpers */

/**
 * @brief saturates a 32-bit integer to a 16-bit integer range.
 * @param x the 32-bit integer to saturate.
 * @return the saturated 16-bit value.
 * // test: q15_saturate(40000) should return 32767.
 */
static inline int16_t q15_saturate(int32_t x) {
    if (x > 32767)  return 32767;
    if (x < -32768) return -32768;
    return (int16_t)x;
}

/**
 * @brief performs saturating q15 multiplication.
 * @param a the first q15 factor.
 * @param b the second q15 factor.
 * @return the saturated q15 product.
 * // test: q15_mul(0x4000, 0x4000) should return 0x2000 (0.5*0.5=0.25).
 */
static inline int16_t q15_mul(int16_t a, int16_t b) {
    int32_t p = (int32_t)a * (int32_t)b;
    return q15_saturate((p + (1 << 14)) >> 15);
}

/**
 * @brief performs saturating q15 division.
 * @param a the q15 numerator.
 * @param b the q15 denominator.
 * @return the saturated q15 quotient.
 * // test: q15_div(0x2000, 0x4000) should return 0x4000 (0.25/0.5=0.5).
 */
static inline int16_t q15_div(int16_t a, int16_t b) {
    if (b == 0) return (a >= 0 ? 0x7FFF : (int16_t)0x8000);
    int32_t n = ((int32_t)a << 15);
    return q15_saturate(n / b);
}


/* conversions */

/**
 * @brief converts a q31 value to q15 with rounding and saturation.
 * @param q the q31 value to convert.
 * @return the converted q15 value.
 * // test: q31_to_q15_sat(0x7FFFFFFF) should return 0x7FFF (32767).
 */
static inline int16_t q31_to_q15_sat(int32_t q) {
    // explicitly checks for the max value before rounding to avoid overflow... (better way to do this?)
    const int32_t max_q31_for_q15 = 0x7FFF7FFF; // max Q31 value that won't overflow after rounding
    if (q >= max_q31_for_q15) {
        return 32767;
    }

    int32_t v = (q + (1 << 15)) >> 16;

    // saturation for negative vals
    if (v <= -32768) {
        return -32768;
    }
    return (int16_t)v;
}


/* sine lookup */
#define SIN_LUT_SIZE 256
static const int16_t sin_lut_q15[SIN_LUT_SIZE] = {
    0, 804, 1608, 2412, 3215, 4018, 4821, 5622, 6423, 7223, 8021, 8818,
    9614, 10408, 11200, 11990, 12778, 13564, 14347, 15128, 15906, 16681,
    17453, 18222, 18987, 19749, 20507, 21261, 22011, 22757, 23498, 24235,
    24967, 25694, 26416, 27132, 27844, 28549, 29249, 29943, 30631, 31312,
    31987, 32655, 33316, 33970, 34617, 35257, 35889, 36514, 37131, 37740,
    38341, 38934, 39518, 40094, 40662, 41221, 41771, 42312, 42845, 43368,
    43881, 44385, 44879, 45363, 45837, 46301, 46755, 47199, 47632, 48055,
    48468, 48870, 49262, 49643, 50013, 50372, 50720, 51057, 51383, 51698,
    52002, 52294, 52575, 52845, 53103, 53350, 53585, 53809, 54021, 54222,
    54411, 54588, 54754, 54908, 55050, 55181, 55300, 55407, 55502, 55586,
    55658, 55718, 55767, 55804, 55829, 55843, 55845, 55835, 55814, 55781,
    55736, 55680, 55612, 55532, 55441, 55338, 55224, 55098, 54961, 54812,
    54652, 54480, 54297, 54103, 53897, 53680, 53452, 53213, 52963, 52702,
    52430, 52147, 51854, 51549, 51234, 50908, 50571, 50224, 49866, 49498,
    49119, 48730, 48330, 47920, 47500, 47069, 46628, 46177, 45716, 45245,
    44764, 44274, 43773, 43263, 42744, 42215, 41677, 41129, 40572, 40006,
    39430, 38846, 38253, 37651, 37040, 36420, 35792, 35156, 34511, 33858,
    33197, 32528, 31851, 31166, 30474, 29774, 29067, 28352, 27630, 26901,
    26165, 25422, 24673, 23917, 23155, 22386, 21612, 20831, 20045, 19253,
    18456, 17653, 16845, 16032, 15214, 14391, 13563, 12730, 11893, 11051,
    10205, 9355, 8501, 7643, 6781, 5916, 5047, 4174, 3299, 2420,
    1538, 654, -233, -1122, -2013, -2906
};

/**
 * @brief calculates sin(x) using a lookup table
 * @param phase_q15 the input angle in q1.15 format, where the range [-32768, 32767] maps to [-pi, +pi]
 * @return the sine of the angle in q1.15 format
 * // test: sin_q15_from_rad(16384) should return ~32767 (sin(pi/2)).
 */
static inline int16_t sin_q15_from_rad(int16_t phase_q15) {
    // Map phase (pi = 32767) to index (pi/2 = 128)
    int32_t index = ((int32_t)phase_q15 * (SIN_LUT_SIZE / 2)) >> 15;
    if (index < 0) index = -index; // Handle negative phase by symmetry sin(-x) = -sin(x)
    index %= (SIN_LUT_SIZE * 2);   // Wrap around 2*pi

    if (index < SIN_LUT_SIZE) {
        return sin_lut_q15[index];
    } else {
        return -sin_lut_q15[index - SIN_LUT_SIZE];
    }
}

/**
 * @brief calculates cos(x) using a lookup table and sin(x + pi/2) identity
 * @param phase_q15 the input angle in q1.15 format, where the range [-32768, 32767] maps to [-pi, +pi]
 * @return the cosine of the angle in q1.15 format
 * // test: cos_q15_from_rad(0) should return ~32767 (cos(0)).
 */
static inline int16_t cos_q15_from_rad(int16_t phase_q15) {
    // Phase is Q1.15, so pi/2 is 16384.
    int32_t phase_plus_half_pi = (int32_t)phase_q15 + 16384;
    // Handle wrap around for the addition
    if (phase_plus_half_pi > 32767) {
        phase_plus_half_pi -= 65536;
    }
    return sin_q15_from_rad((int16_t)phase_plus_half_pi);
}

#endif /* INC_MATH_UTILS_H_ */
