#include <iostream>
#include <cmath>
#include <cassert>
#include "embdC_quat.h"
#include "embdC_quat_fixed.h"

#define EXPECT_NEAR(a, b, eps) \
    if (std::abs((a) - (b)) > (eps)) { \
        std::cerr << "FAILED: " << #a << " (" << (a) << ") != " << #b << " (" << (b) << ") within " << (eps) << std::endl; \
        exit(1); \
    }

void test_fixed_conversions() {
    std::cout << "Running test_fixed_conversions..." << std::endl;
    float f = 0.5f;
    q30_t q = float_to_q30(f);
    float f2 = q30_to_float(q);
    EXPECT_NEAR(f, f2, 1e-7);
    
    vec3_t v = { 1.0f, -0.5f, 0.25f };
    vec3_fixed_t vf = vec3_to_fixed(v);
    vec3_t v2 = vec3_from_fixed(vf);
    EXPECT_NEAR(v.x, v2.x, 1e-7);
    EXPECT_NEAR(v.y, v2.y, 1e-7);
    EXPECT_NEAR(v.z, v2.z, 1e-7);
}

void test_fixed_multiply() {
    std::cout << "Running test_fixed_multiply..." << std::endl;
    quat_t q1 = { 0.7071f, 0.7071f, 0.0f, 0.0f }; // 90 deg X
    quat_t q2 = { 0.7071f, 0.0f, 0.7071f, 0.0f }; // 90 deg Y
    
    quat_t res_float = quat_multiply(q1, q2);
    
    quat_fixed_t q1f = quat_to_fixed(q1);
    quat_fixed_t q2f = quat_to_fixed(q2);
    quat_fixed_t res_fixed = quat_multiply_fixed(q1f, q2f);
    
    quat_t res_back = quat_from_fixed(res_fixed);
    
    EXPECT_NEAR(res_float.w, res_back.w, 1e-4);
    EXPECT_NEAR(res_float.x, res_back.x, 1e-4);
    EXPECT_NEAR(res_float.y, res_back.y, 1e-4);
    EXPECT_NEAR(res_float.z, res_back.z, 1e-4);
}

void test_fixed_rotation() {
    std::cout << "Running test_fixed_rotation..." << std::endl;
    // Rotate [1, 2, 3] by 90 deg around Z
    quat_t q = { 0.70710678f, 0.0f, 0.0f, 0.70710678f };
    vec3_t v = { 1.0f, 0.0f, 0.0f };
    
    vec3_t v_rot_float = quat_rotate_vec3(q, v);
    
    quat_fixed_t qf = quat_to_fixed(q);
    vec3_fixed_t vf = vec3_to_fixed(v);
    vec3_fixed_t v_rot_fixed = quat_rotate_vec3_fixed(qf, vf);
    
    vec3_t v_rot_back = vec3_from_fixed(v_rot_fixed);
    
    EXPECT_NEAR(v_rot_float.x, v_rot_back.x, 1e-4);
    EXPECT_NEAR(v_rot_float.y, v_rot_back.y, 1e-4);
    EXPECT_NEAR(v_rot_float.z, v_rot_back.z, 1e-4);
    
    // Check specific expected value [0, 1, 0]
    EXPECT_NEAR(v_rot_back.x, 0.0f, 1e-4);
    EXPECT_NEAR(v_rot_back.y, 1.0f, 1e-4);
    EXPECT_NEAR(v_rot_back.z, 0.0f, 1e-4);
}

int main() {
    test_fixed_conversions();
    test_fixed_multiply();
    test_fixed_rotation();
    std::cout << "FIXED POINT TESTS PASSED!" << std::endl;
    return 0;
}
