#pragma once

#include <cmath>

using std::isnan;
using std::isinf;

#include <limits>
#include <stdint.h>
#include <type_traits>

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

#include "definitions.h"
#include "edc.h"
#include "location.h"
#include "matrix3.h"
#include "polygon.h"
#include "quaternion.h"
#include "rotations.h"
#include "vector2.h"
#include "vector3.h"

#if __ARM_FP == 4

//#define acos acosf
//#define asin asinf
//#define sin sinf
//#define cos cosf

__attribute((always_inline))
inline float sqrtf(float op1) {
	float result;
	asm volatile ("vsqrt.f32 %0, %1" : "=w" (result) : "w" (op1) );
	return (result);

}


namespace fpu {
__attribute((always_inline))
inline float sqrt(float op1) {
	float result;
	asm volatile ("vsqrt.f32 %0, %1" : "=w" (result) : "w" (op1) );
	return (result);

}
}

using fpu::sqrt;
//using fpu::sqrtf;
#endif
// define AP_Param types AP_Vector3f and Ap_Matrix3f
AP_PARAMDEFV(Vector3f, Vector3f, AP_PARAM_VECTOR3F);

/*
 * Check whether two floats are equal
 */
template <class FloatOne, class FloatTwo>
bool is_equal(const FloatOne v_1, const FloatTwo v_2)
{
    static_assert(std::is_arithmetic<FloatOne>::value, "template parameter not of type float or int");
    static_assert(std::is_arithmetic<FloatTwo>::value, "template parameter not of type float or int");
    return fabsf(v_1 - v_2) < std::numeric_limits<decltype(v_1 - v_2)>::epsilon();
}

/* 
 * @brief: Check whether a float is zero
 */
template <class T>
inline bool is_zero(const T fVal1) {
    static_assert(std::is_floating_point<T>::value || std::is_base_of<T,AP_Float>::value,
                  "Template parameter not of type float");
    return fabsf(static_cast<float>(fVal1)) < FLT_EPSILON ? true : false;
}

/*
 * A variant of asin() that checks the input ranges and ensures a valid angle
 * as output. If nan is given as input then zero is returned.
 */
template <class T>
float safe_asin(const T v)
{
    if (isnan(static_cast<float>(v))) {
        return 0.0f;
    }
    if (v >= 1.0f) {
        return static_cast<float>(M_PI_2);
    }
    if (v <= -1.0f) {
        return static_cast<float>(-M_PI_2);
    }
    return asinf(static_cast<float>(v));
}

/*
 * A variant of sqrt() that checks the input ranges and ensures a valid value
 * as output. If a negative number is given then 0 is returned.  The reasoning
 * is that a negative number for sqrt() in our code is usually caused by small
 * numerical rounding errors, so the real input should have been zero
 */
template <class T>
float safe_sqrt(const T v)
{
    float ret = sqrtf(static_cast<float>(v));
    if (isnan(ret)) {
        return 0;
    }
    return ret;
}

// invOut is an inverted 4x4 matrix when returns true, otherwise matrix is Singular
bool inverse3x3(float m[], float invOut[]);

// invOut is an inverted 3x3 matrix when returns true, otherwise matrix is Singular
bool inverse4x4(float m[],float invOut[]);

// matrix multiplication of two NxN matrices
float *mat_mul(float *A, float *B, uint8_t n);

// matrix algebra
bool inverse(float x[], float y[], uint16_t dim);

/*
 * Constrain an euler angle to be within the range: 0 to 360 degrees. The
 * second parameter changes the units. Default: 1 == degrees, 10 == dezi,
 * 100 == centi.
 */
template <class T>
float wrap_360(const T angle, float unit_mod = 1.0f)
{
    const float ang_360 = 360.f * unit_mod;
    float res = fmodf(static_cast<float>(angle), ang_360);
    if (res < 0) {
        res += ang_360;
    }
    return res;
}

/*
 * Constrain an angle to be within the range: -180 to 180 degrees. The second
 * parameter changes the units. Default: 1 == degrees, 10 == dezi,
 * 100 == centi.
 */
template <class T>
float wrap_180(const T angle, float unit_mod = 1.0f)
{
    auto res = wrap_360(angle, unit_mod);
    if (res > 180.f * unit_mod) {
        res -= 360.f * unit_mod;
    }
    return res;
}

/*
 * Wrap an angle in centi-degrees. See wrap_180().
 */
template <class T>
auto wrap_180_cd(const T angle) -> decltype(wrap_180(angle, 100.f))
{
    return wrap_180(angle, 100.f);
}

/*
 * Wrap an angle in centi-degrees. See wrap_360().
 */
template <class T>
auto wrap_360_cd(const T angle) -> decltype(wrap_360(angle, 100.f))
{
    return wrap_360(angle, 100.f);
}

/*
 * wrap an angle in radians to 0..2PI
 */
template <class T>
float wrap_2PI(const T radian)
{
    float res = fmodf(static_cast<float>(radian), M_2PI);
    if (res < 0) {
        res += M_2PI;
    }
    return res;
}

/*
  wrap an angle in radians to -PI ~ PI (equivalent to +- 180 degrees)
 */
template <class T>
float wrap_PI(const T radian)
{
    auto res = wrap_2PI(radian);
    if (res > M_PI) {
        res -= M_2PI;
    }
    return res;
}

/*
 * Constrain a value to be within the range: low and high
 */
template <class T>
T constrain_value(const T amt, const T low, const T high)
{
    // the check for NaN as a float prevents propagation of floating point
    // errors through any function that uses constrain_float(). The normal
    // float semantics already handle -Inf and +Inf
    if (isnan(amt)) {
        return (low + high) * 0.5f;
    }

    if (amt < low) {
        return low;
    }

    if (amt > high) {
        return high;
    }

    return amt;
}

inline float constrain_float(const float amt, const float low, const float high)
{
    return constrain_value(amt, low, high);
}

inline int16_t constrain_int16(const int16_t amt, const int16_t low, const int16_t high)
{
    return constrain_value(amt, low, high);
}

inline int32_t constrain_int32(const int32_t amt, const int32_t low, const int32_t high)
{
    return constrain_value(amt, low, high);
}

// degrees -> radians
static inline float radians(float deg)
{
    return deg * DEG_TO_RAD;
}

// radians -> degrees
static inline float degrees(float rad)
{
    return rad * RAD_TO_DEG;
}


template<class T>
inline static float sq(const T val)
{
    return powf(static_cast<float>(val), 2);
}

/*
 * Variadic template for calculating the square norm of a vector of any
 * dimension.
 */
template<class T, class... Params>
inline static float sq(const T first, const Params... parameters)
{
    return sq(first) + sq(parameters...);
}

/*
 * Variadic template for calculating the norm (pythagoras) of a vector of any
 * dimension.
 */
template<class T, class... Params>
inline static float norm(const T first, const Params... parameters)
{
    return sqrt(static_cast<float>(sq(first, parameters...)));
}

template<typename A, typename B>
static inline auto MIN(const A &one, const B &two) -> decltype(one < two ? one : two)
{
    return one < two ? one : two;
}

template<typename A, typename B>
static inline auto MAX(const A &one, const B &two) -> decltype(one > two ? one : two)
{
    return one > two ? one : two;
}

inline uint32_t hz_to_nsec(uint32_t freq)
{
    return NSEC_PER_SEC / freq;
}

inline uint32_t nsec_to_hz(uint32_t nsec)
{
    return NSEC_PER_SEC / nsec;
}

inline uint32_t usec_to_nsec(uint32_t usec)
{
    return usec * NSEC_PER_USEC;
}

inline uint32_t nsec_to_usec(uint32_t nsec)
{
    return nsec / NSEC_PER_USEC;
}

inline uint32_t hz_to_usec(uint32_t freq)
{
    return USEC_PER_SEC / freq;
}

inline uint32_t usec_to_hz(uint32_t usec)
{
    return USEC_PER_SEC / usec;
}

/*
  linear interpolation based on a variable in a range
 */
inline float linear_interpolate(float low_output, float high_output,
                         float var_value,
                         float var_low, float var_high)
{
    if (var_value <= var_low) {
        return low_output;
    }
    if (var_value >= var_high) {
        return high_output;
    }
    float p = (var_value - var_low) / (var_high - var_low);
    return low_output + p * (high_output - low_output);
}

