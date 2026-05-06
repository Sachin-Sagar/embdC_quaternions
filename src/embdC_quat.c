#include "embdC_quat.h"
#include <math.h>

// --- Basic Vector Operations ---

vec3_t vec3_add(vec3_t a, vec3_t b) {
    return (vec3_t){a.x + b.x, a.y + b.y, a.z + b.z};
}

vec3_t vec3_sub(vec3_t a, vec3_t b) {
    return (vec3_t){a.x - b.x, a.y - b.y, a.z - b.z};
}

float vec3_dot(vec3_t a, vec3_t b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

vec3_t vec3_cross(vec3_t a, vec3_t b) {
    return (vec3_t){
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

float vec3_norm(vec3_t v) {
    return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

vec3_t vec3_normalize(vec3_t v) {
    float n = vec3_norm(v);
    if (n < 1e-6f) return (vec3_t){0, 0, 0};
    return (vec3_t){v.x / n, v.y / n, v.z / n};
}

// --- Basic Quaternion Operations ---

quat_t quat_identity(void) {
    return (quat_t){1.0f, 0.0f, 0.0f, 0.0f};
}

quat_t quat_multiply(quat_t q1, quat_t q2) {
    return (quat_t){
        q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z,
        q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y,
        q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x,
        q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w
    };
}

quat_t quat_conjugate(quat_t q) {
    return (quat_t){q.w, -q.x, -q.y, -q.z};
}

float quat_norm(quat_t q) {
    return sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
}

quat_t quat_normalize(quat_t q) {
    float n = quat_norm(q);
    if (n < 1e-6f) return quat_identity();
    return (quat_t){q.w / n, q.x / n, q.y / n, q.z / n};
}

quat_t quat_inverse(quat_t q) {
    float n2 = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
    if (n2 < 1e-6f) return quat_identity();
    quat_t conj = quat_conjugate(q);
    return (quat_t){conj.w / n2, conj.x / n2, conj.y / n2, conj.z / n2};
}

// --- Rotations & Transformations ---

vec3_t quat_rotate_vec3(quat_t q, vec3_t v) {
    // v' = q * v * q^-1
    // For unit quaternions, q^-1 = q_conjugate
    quat_t v_quat = {0.0f, v.x, v.y, v.z};
    quat_t q_inv = quat_inverse(q);
    quat_t temp = quat_multiply(q, v_quat);
    quat_t result_quat = quat_multiply(temp, q_inv);
    return (vec3_t){result_quat.x, result_quat.y, result_quat.z};
}

vec3_t transform_sensor_to_body(vec3_t v_sensor, quat_t q_sensor_to_body) {
    return quat_rotate_vec3(q_sensor_to_body, v_sensor);
}

// --- Conversions ---

quat_t quat_from_euler(euler_t e) {
    // Using ZYX convention (yaw, then pitch, then roll)
    float cy = cosf(e.yaw * 0.5f);
    float sy = sinf(e.yaw * 0.5f);
    float cp = cosf(e.pitch * 0.5f);
    float sp = sinf(e.pitch * 0.5f);
    float cr = cosf(e.roll * 0.5f);
    float sr = sinf(e.roll * 0.5f);

    return (quat_t){
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy
    };
}

euler_t quat_to_euler(quat_t q) {
    euler_t e;

    // roll (x-axis rotation)
    float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    e.roll = atan2f(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    float sinp = 2.0f * (q.w * q.y - q.z * q.x);
    if (fabsf(sinp) >= 1.0f)
        e.pitch = copysignf(M_PI / 2.0f, sinp); // use 90 degrees if out of range
    else
        e.pitch = asinf(sinp);

    // yaw (z-axis rotation)
    float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    e.yaw = atan2f(siny_cosp, cosy_cosp);

    return e;
}
