#include "embdC_quat_fixed.h"

// Helper for Q30 multiplication: (a * b) >> 30
// Using int64_t for intermediate product to prevent overflow
static inline q30_t q30_mul(q30_t a, q30_t b) {
    return (q30_t)(((int64_t)a * b) >> 30);
}

q30_t float_to_q30(float f) {
    return (q30_t)(f * (float)Q30_ONE);
}

float q30_to_float(q30_t q) {
    return (float)q / (float)Q30_ONE;
}

vec3_fixed_t vec3_to_fixed(vec3_t v) {
    return (vec3_fixed_t){
        float_to_q30(v.x),
        float_to_q30(v.y),
        float_to_q30(v.z)
    };
}

vec3_t vec3_from_fixed(vec3_fixed_t v) {
    return (vec3_t){
        q30_to_float(v.x),
        q30_to_float(v.y),
        q30_to_float(v.z)
    };
}

quat_fixed_t quat_to_fixed(quat_t q) {
    return (quat_fixed_t){
        float_to_q30(q.w),
        float_to_q30(q.x),
        float_to_q30(q.y),
        float_to_q30(q.z)
    };
}

quat_t quat_from_fixed(quat_fixed_t q) {
    return (quat_t){
        q30_to_float(q.w),
        q30_to_float(q.x),
        q30_to_float(q.y),
        q30_to_float(q.z)
    };
}

quat_fixed_t quat_multiply_fixed(quat_fixed_t q1, quat_fixed_t q2) {
    // w = w1w2 - x1x2 - y1y2 - z1z2
    q30_t w = q30_mul(q1.w, q2.w) - q30_mul(q1.x, q2.x) - q30_mul(q1.y, q2.y) - q30_mul(q1.z, q2.z);
    // x = w1x2 + x1w2 + y1z2 - z1y2
    q30_t x = q30_mul(q1.w, q2.x) + q30_mul(q1.x, q2.w) + q30_mul(q1.y, q2.z) - q30_mul(q1.z, q2.y);
    // y = w1y2 - x1z2 + y1w2 + z1x2
    q30_t y = q30_mul(q1.w, q2.y) - q30_mul(q1.x, q2.z) + q30_mul(q1.y, q2.w) + q30_mul(q1.z, q2.x);
    // z = w1z2 + x1y2 - y1x2 + z1w2
    q30_t z = q30_mul(q1.w, q2.z) + q30_mul(q1.x, q2.y) - q30_mul(q1.y, q2.x) + q30_mul(q1.z, q2.w);
    
    return (quat_fixed_t){w, x, y, z};
}

vec3_fixed_t quat_rotate_vec3_fixed(quat_fixed_t q, vec3_fixed_t v) {
    // Using the optimized formula: v' = v + 2 * q_vec x (q_vec x v + q_w * v)
    
    // 1. t = q_vec x v
    q30_t tx = q30_mul(q.y, v.z) - q30_mul(q.z, v.y);
    q30_t ty = q30_mul(q.z, v.x) - q30_mul(q.x, v.z);
    q30_t tz = q30_mul(q.x, v.y) - q30_mul(q.y, v.x);
    
    // 2. t = t + q_w * v
    tx += q30_mul(q.w, v.x);
    ty += q30_mul(q.w, v.y);
    tz += q30_mul(q.w, v.z);
    
    // 3. result = v + 2 * (q_vec x t)
    // q_vec x t
    q30_t r_x = q30_mul(q.y, tz) - q30_mul(q.z, ty);
    q30_t r_y = q30_mul(q.z, tx) - q30_mul(q.x, tz);
    q30_t r_z = q30_mul(q.x, ty) - q30_mul(q.y, tx);
    
    // v + 2 * result
    vec3_fixed_t res;
    res.x = v.x + (r_x << 1);
    res.y = v.y + (r_y << 1);
    res.z = v.z + (r_z << 1);
    
    return res;
}
