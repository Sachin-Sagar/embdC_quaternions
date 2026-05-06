# Implementation Plan: EmbdC Quaternions

## Objective
Develop a pure C library (`embdC_quat`) for quaternion-based coordinate transformations (specifically sensor frame to body frame). Create a C++ simulation and visualization tool using Raylib to demonstrate the transformations, simulate test cases, and compare quaternions against Euler angles (e.g., highlighting gimbal lock).

## Project Structure
```text
embdC_quaternions/
├── PROJECT_PLAN.md             # This plan
├── CMakeLists.txt              # Root CMake configuration
├── include/
│   └── embdC_quat.h            # Pure C API declarations
├── src/
│   ├── CMakeLists.txt          # Library build config
│   └── embdC_quat.c            # Pure C implementation
├── tests/
│   ├── CMakeLists.txt          # Tests build config
│   └── test_quat.cpp           # Google Test test cases
└── sim/
    ├── CMakeLists.txt          # Simulation app build config
    └── main.cpp                # C++ Raylib visualization
```

## Technology Stack
*   **Math Library:** Pure C (C99/C11 compatible)
*   **Simulation/Viz:** C++17, Raylib
*   **Testing:** Google Test (C++)
*   **Build System:** CMake

## Implementation Phases

### Phase 1: Pure C Math Library
*   **Data Structures:**
    *   `quat_t`: struct with `w, x, y, z`.
    *   `vec3_t`: struct with `x, y, z`.
    *   `euler_t`: struct with `roll, pitch, yaw`.
*   **Core Math Operations:**
    *   Quaternion multiplication, normalization, conjugate, inverse.
    *   Vector rotation using quaternions ($v' = q \cdot v \cdot q^{-1}$).
*   **Conversions & Transformations:**
    *   Quaternion to Euler angles and vice versa.
    *   Quaternion to Rotation Matrix.
    *   Coordinate transformation function: transforming a vector from a sensor frame to the body frame given the sensor's mounting orientation (as a quaternion).

### Phase 2: Unit Testing (Google Test)
*   Set up Google Test via CMake FetchContent.
*   Write unit tests to validate:
    *   Basic arithmetic properties of quaternions.
    *   Conversion accuracy (Quat -> Euler -> Quat).
    *   Specific transformations to ensure axes align correctly.
    *   Edge cases (e.g., Euler angle singularities / gimbal lock).

### Phase 3: Visualization Tool (Raylib)
*   Set up a Raylib window with a 3D orbit/free camera.
*   **Visual Elements:**
    *   Draw the World Frame (origin).
    *   Draw two identical 3D objects (e.g., an airplane model or distinct directional shapes) side-by-side.
        *   **Object A:** Rotated using Euler angles.
        *   **Object B:** Rotated using Quaternions.
    *   Draw coordinate axes (X, Y, Z) attached to the bodies to clearly show orientation.
*   **Simulation Loop:**
    *   Implement an input mechanism (keyboard or on-screen sliders using Raygui if available, or simple text overlays) to apply rotations.
    *   Create a "Gimbal Lock Demo" mode: continuously rotate along axes to visually demonstrate how the Euler representation loses a degree of freedom compared to the Quaternion representation.
*   **Information Display:**
    *   Overlay text showing current Euler angles, Quaternion components, and whether Gimbal Lock is near.

## Verification & Iteration
*   Run tests (`ctest`) to ensure the math library is rock solid.
*   Verify the visualization builds and runs smoothly, correctly displaying the mathematical operations.
