#include "embdC_quat.h"
#include <math.h>

// --- Basic Vector Operations ---

vec3_t vec3_add(vec3_t a, vec3_t b) {
    float x = a.x + b.x;
    float y = a.y + b.y;
    float z = a.z + b.z;
    return (vec3_t){x, y, z};
}

vec3_t vec3_sub(vec3_t a, vec3_t b) {
    float x = a.x - b.x;
    float y = a.y - b.y;
    float z = a.z - b.z;
    return (vec3_t){x, y, z};
}

float vec3_dot(vec3_t a, vec3_t b) {
    float dot = a.x * b.x + a.y * b.y + a.z * b.z;
    return dot;
}

vec3_t vec3_cross(vec3_t a, vec3_t b) {
    float x = a.y * b.z - a.z * b.y;
    float y = a.z * b.x - a.x * b.z;
    float z = a.x * b.y - a.y * b.x;
    return (vec3_t){x, y, z};
}

float vec3_norm(vec3_t v) {
    float mag_sq = v.x * v.x + v.y * v.y + v.z * v.z;
    return sqrtf(mag_sq);
}

vec3_t vec3_normalize(vec3_t v) {
    float n = vec3_norm(v);
    if (n < 1e-6f) {
        return (vec3_t){0.0f, 0.0f, 0.0f};
    }
    float inv_n = 1.0f / n;
    return (vec3_t){v.x * inv_n, v.y * inv_n, v.z * inv_n};
}

// --- Basic Quaternion Operations ---

quat_t quat_identity(void) {
    return (quat_t){1.0f, 0.0f, 0.0f, 0.0f};
}

quat_t quat_multiply(quat_t q1, quat_t q2) {
    float w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
    float x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
    float y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
    float z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
    return (quat_t){w, x, y, z};
}

quat_t quat_conjugate(quat_t q) {
    return (quat_t){q.w, -q.x, -q.y, -q.z};
}

float quat_norm(quat_t q) {
    float mag_sq = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
    return sqrtf(mag_sq);
}

quat_t quat_normalize(quat_t q) {
    float n = quat_norm(q);
    if (n < 1e-6f) {
        return quat_identity();
    }
    float inv_n = 1.0f / n;
    return (quat_t){q.w * inv_n, q.x * inv_n, q.y * inv_n, q.z * inv_n};
}

quat_t quat_inverse(quat_t q) {
    float n2 = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
    if (n2 < 1e-6f) {
        return quat_identity();
    }
    float inv_n2 = 1.0f / n2;
    return (quat_t){
        q.w * inv_n2,
        -q.x * inv_n2,
        -q.y * inv_n2,
        -q.z * inv_n2
    };
}

// --- Rotations & Transformations ---

/**
 * Optimized vector rotation using quaternion.
 * v' = v + 2 * q_vec x (q_vec x v + q_w * v)
 * This is faster than v' = q * v * q^-1 as it avoids full quat multiplication.
 */
vec3_t quat_rotate_vec3(quat_t q, vec3_t v) {
    vec3_t q_vec = {q.x, q.y, q.z};
    
    // t = 2 * (q_vec x v)
    vec3_t q_x_v = vec3_cross(q_vec, v);
    vec3_t t = {
        2.0f * q_x_v.x,
        2.0f * q_x_v.y,
        2.0f * q_x_v.z
    };
    
    // v' = v + q_w * t + q_vec x t
    vec3_t qw_t = {
        q.w * t.x,
        q.w * t.y,
        q.w * t.z
    };
    vec3_t q_x_t = vec3_cross(q_vec, t);
    
    vec3_t result = {
        v.x + qw_t.x + q_x_t.x,
        v.y + qw_t.y + q_x_t.y,
        v.z + qw_t.z + q_x_t.z
    };
    
    return result;
}

vec3_t transform_sensor_to_body(vec3_t v_sensor, quat_t q_sensor_to_body) {
    return quat_rotate_vec3(q_sensor_to_body, v_sensor);
}

// --- Conversions ---

quat_t quat_from_euler(euler_t e) {
    // Using ZYX convention (yaw, then pitch, then roll)
    float half_yaw = e.yaw * 0.5f;
    float half_pitch = e.pitch * 0.5f;
    float half_roll = e.roll * 0.5f;
    
    float cy = cosf(half_yaw);
    float sy = sinf(half_yaw);
    float cp = cosf(half_pitch);
    float sp = sinf(half_pitch);
    float cr = cosf(half_roll);
    float sr = sinf(half_roll);

    float w = cr * cp * cy + sr * sp * sy;
    float x = sr * cp * cy - cr * sp * sy;
    float y = cr * sp * cy + sr * cp * sy;
    float z = cr * cp * sy - sr * sp * cy;
    
    return (quat_t){w, x, y, z};
}

euler_t quat_to_euler(quat_t q) {
    euler_t e;

    // roll (x-axis rotation)
    float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    e.roll = atan2f(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    float sinp = 2.0f * (q.w * q.y - q.z * q.x);
    if (fabsf(sinp) >= 1.0f) {
        // use 90 degrees if out of range
        e.pitch = copysignf(1.57079632679f, sinp); 
    } else {
        e.pitch = asinf(sinp);
    }

    // yaw (z-axis rotation)
    float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    e.yaw = atan2f(siny_cosp, cosy_cosp);

    return e;
}
