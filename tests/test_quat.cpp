#include <iostream>
#include <cmath>
#include <cassert>
#include "embdC_quat.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define EXPECT_NEAR(a, b, eps) \
    if (std::abs((a) - (b)) > (eps)) { \
        std::cerr << "FAILED: " << #a << " (" << (a) << ") != " << #b << " (" << (b) << ") within " << (eps) << std::endl; \
        exit(1); \
    }

#define EXPECT_FLOAT_EQ(a, b) EXPECT_NEAR(a, b, 1e-6)

void test_identity() {
    std::cout << "Running test_identity..." << std::endl;
    quat_t q = quat_identity();
    EXPECT_FLOAT_EQ(q.w, 1.0f);
    EXPECT_FLOAT_EQ(q.x, 0.0f);
    EXPECT_FLOAT_EQ(q.y, 0.0f);
    EXPECT_FLOAT_EQ(q.z, 0.0f);
}

void test_multiply_identity() {
    std::cout << "Running test_multiply_identity..." << std::endl;
    quat_t q1 = {1, 2, 3, 4};
    quat_t id = quat_identity();
    quat_t res = quat_multiply(q1, id);
    EXPECT_FLOAT_EQ(res.w, 1.0f);
    EXPECT_FLOAT_EQ(res.x, 2.0f);
    EXPECT_FLOAT_EQ(res.y, 3.0f);
    EXPECT_FLOAT_EQ(res.z, 4.0f);
}

void test_rotation_x() {
    std::cout << "Running test_rotation_x..." << std::endl;
    // Rotate [0, 1, 0] by 90 degrees around X axis -> [0, 0, 1]
    euler_t e = { (float)(M_PI / 2.0), 0, 0 };
    quat_t q = quat_from_euler(e);
    vec3_t v = { 0, 1, 0 };
    vec3_t v_rot = quat_rotate_vec3(q, v);
    
    EXPECT_NEAR(v_rot.x, 0.0f, 1e-6);
    EXPECT_NEAR(v_rot.y, 0.0f, 1e-6);
    EXPECT_NEAR(v_rot.z, 1.0f, 1e-6);
}

void test_euler_conversion() {
    std::cout << "Running test_euler_conversion..." << std::endl;
    euler_t e1 = { 0.1f, 0.2f, 0.3f };
    quat_t q = quat_from_euler(e1);
    euler_t e2 = quat_to_euler(q);
    
    EXPECT_NEAR(e1.roll, e2.roll, 1e-5);
    EXPECT_NEAR(e1.pitch, e2.pitch, 1e-5);
    EXPECT_NEAR(e1.yaw, e2.yaw, 1e-5);
}

void test_sensor_to_body() {
    std::cout << "Running test_sensor_to_body..." << std::endl;
    // Sensor is mounted 90 deg rotated around Z (yaw)
    // Sensor X points to Body Y, Sensor Y points to -Body X
    euler_t mount_euler = { 0, 0, (float)(M_PI / 2.0) };
    quat_t q_mount = quat_from_euler(mount_euler);
    
    vec3_t v_sensor = { 1, 0, 0 }; // Sensor X
    vec3_t v_body = transform_sensor_to_body(v_sensor, q_mount);
    
    EXPECT_NEAR(v_body.x, 0.0f, 1e-6);
    EXPECT_NEAR(v_body.y, 1.0f, 1e-6);
    EXPECT_NEAR(v_body.z, 0.0f, 1e-6);
}

int main() {
    test_identity();
    test_multiply_identity();
    test_rotation_x();
    test_euler_conversion();
    test_sensor_to_body();
    std::cout << "ALL TESTS PASSED!" << std::endl;
    return 0;
}
