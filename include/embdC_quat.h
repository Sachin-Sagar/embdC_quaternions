#ifndef EMBDC_QUAT_H
#define EMBDC_QUAT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>
#include <stdbool.h>

/**
 * @brief 3D Vector structure
 */
typedef struct {
    float x;
    float y;
    float z;
} vec3_t;

/**
 * @brief Quaternion structure (w + xi + yj + zk)
 */
typedef struct {
    float w;
    float x;
    float y;
    float z;
} quat_t;

/**
 * @brief Euler angles structure (in radians)
 */
typedef struct {
    float roll;  // Rotation around X axis
    float pitch; // Rotation around Y axis
    float yaw;   // Rotation around Z axis
} euler_t;

// --- Basic Vector Operations ---
vec3_t vec3_add(vec3_t a, vec3_t b);
vec3_t vec3_sub(vec3_t a, vec3_t b);
float vec3_dot(vec3_t a, vec3_t b);
vec3_t vec3_cross(vec3_t a, vec3_t b);
float vec3_norm(vec3_t v);
vec3_t vec3_normalize(vec3_t v);

// --- Basic Quaternion Operations ---
quat_t quat_identity(void);
quat_t quat_multiply(quat_t q1, quat_t q2);
quat_t quat_conjugate(quat_t q);
quat_t quat_inverse(quat_t q);
float quat_norm(quat_t q);
quat_t quat_normalize(quat_t q);

// --- Rotations & Transformations ---
/**
 * @brief Rotates a vector using a quaternion: v' = q * v * q^-1
 */
vec3_t quat_rotate_vec3(quat_t q, vec3_t v);

/**
 * @brief Transforms a vector from sensor frame to body frame.
 * 
 * @param v_sensor Vector in sensor frame
 * @param q_sensor_to_body Quaternion representing the rotation from sensor to body frame
 * @return vec3_t Vector in body frame
 */
vec3_t transform_sensor_to_body(vec3_t v_sensor, quat_t q_sensor_to_body);

// --- Conversions ---
quat_t quat_from_euler(euler_t e);
euler_t quat_to_euler(quat_t q);

// Note: Additional conversions (Rotation Matrix, etc.) can be added as needed.

#ifdef __cplusplus
}
#endif

#endif // EMBDC_QUAT_H
