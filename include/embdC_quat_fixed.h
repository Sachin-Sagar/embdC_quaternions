#ifndef EMBDC_QUAT_FIXED_H
#define EMBDC_QUAT_FIXED_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "embdC_quat.h"

/**
 * @brief Fixed-point type (Q30 format: 1 sign bit, 1 integer bit, 30 fractional bits)
 * Range: [-2, 2), 1.0 is represented as (1 << 30)
 */
typedef int32_t q30_t;

#define Q30_ONE      (1 << 30)
#define Q30_HALF     (1 << 29)
#define Q30_TWO      (1 << 31)

/**
 * @brief Fixed-point 3D Vector structure
 */
typedef struct {
    q30_t x;
    q30_t y;
    q30_t z;
} vec3_fixed_t;

/**
 * @brief Fixed-point Quaternion structure
 */
typedef struct {
    q30_t w;
    q30_t x;
    q30_t y;
    q30_t z;
} quat_fixed_t;

// --- Conversions ---
q30_t float_to_q30(float f);
float q30_to_float(q30_t q);

vec3_fixed_t vec3_to_fixed(vec3_t v);
vec3_t vec3_from_fixed(vec3_fixed_t v);

quat_fixed_t quat_to_fixed(quat_t q);
quat_t quat_from_fixed(quat_fixed_t q);

// --- Basic Operations ---
quat_fixed_t quat_multiply_fixed(quat_fixed_t q1, quat_fixed_t q2);
vec3_fixed_t quat_rotate_vec3_fixed(quat_fixed_t q, vec3_fixed_t v);

// Note: Fixed-point trigonometry and norm/normalize are complex in pure C without 
// CMSIS-DSP. We'll implement the core rotation and multiplication first.

#ifdef __cplusplus
}
#endif

#endif // EMBDC_QUAT_FIXED_H
