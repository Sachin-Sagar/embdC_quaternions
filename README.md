# EmbdC Quaternions

A lightweight, high-performance, pure C library for quaternion-based coordinate transformations, featuring both floating-point and fixed-point implementations. This project is specifically optimized for embedded systems (e.g., STM32) where sensor-to-body frame transformations are critical.

## Features
- **Optimized Floating-Point Implementation (`embdC_quat`):**
    - Uses an optimized "sandwich product" for vector rotation: $v' = v + 2q_{xyz} \times (q_{xyz} \times v + w v)$, reducing operations by >50% compared to standard $q v q^{-1}$.
    - Single-precision optimized for hardware FPUs (Cortex-M4F/M7).
    - ZYX Euler angle conversions (Roll, Pitch, Yaw).
- **Q30 Fixed-Point Implementation (`embdC_quat_fixed`):**
    - High-precision `int32_t` arithmetic (Q30 format).
    - Deterministic performance, ideal for systems without an FPU (Cortex-M0/M3).
    - Minimal memory footprint.
- **Sensor-to-Body Transformation:** Simplified API for transforming vectors from a sensor mounting frame to the vehicle body frame.
- **C++ Simulation Tool:**
    - Real-time 3D visualization using Raylib.
    - Z-Up Orientation following aerospace and robotics conventions.
    - Interactive HUD showing real-time Euler and Quaternion data.
- **Testing:** Comprehensive unit test suite covering both floating and fixed-point math.

## Project Structure
- `include/`: C API headers.
- `src/`: Core library implementation (C).
- `sim/`: Simulation tool source (C++).
- `tests/`: Unit test suite.
- `CMakeLists.txt`: Build configuration.

## Getting Started

### Prerequisites
- **Compiler:** GCC (C11) and G++ (C++17).
- **Build Tool:** CMake.
- **Dependencies:** [Raylib](https://www.raylib.com/) (Required for simulation only).

### Building the Project
```bash
mkdir build && cd build
cmake ..
make

# Run the unit tests
./tests/test_quat
./tests/test_quat_fixed
```

## Implementation Details

### Floating-Point Optimization
The library is designed to prevent implicit `double` promotion. All constants use the `f` suffix, and we avoid heavy standard library calls where possible. The vector rotation is implemented as:
```c
vec3_t q_x_v = vec3_cross(q_vec, v);
vec3_t t = { 2.0f * q_x_v.x, 2.0f * q_x_v.y, 2.0f * q_x_v.z };
v_prime = v + q.w * t + (q_vec x t);
```

### Fixed-Point (Q30)
The fixed-point version uses a **Q30** format:
- **1.0** is represented as `1 << 30`.
- **Range:** $[-2, 2)$, perfect for unit quaternions.
- **Precision:** $\approx 9.3 \times 10^{-10}$.
- **Intermediate Math:** Uses `int64_t` for products to ensure zero overflow before the shift.

## Simulation Usage
Run the visualization tool:
```bash
./sim/simulation
```

The simulation features a **720p 3D Viewport** on the left and an **Interactive Sidebar** on the right.

### Controls & Interaction
- **3D Navigation:** Use the keyboard shortcuts listed in the sidebar to rotate the Body or move/rotate the IMU.
- **Manual Input:** Click on any coordinate or angle box in the sidebar to type values directly. Press Backspace to edit. Click outside the box to deselect.
- **Handedness Guarantee:** All transformations are quaternion-based, ensuring the resulting mapping is always right-handed.
- **Export:** Press **P** to save the configuration and the 3x3 Rotation Matrix ($R_{s2b}$) to `imu_config.txt`.

### Customization (Source Code)
You can customize the simulation by modifying the configuration structures at the top of `sim/main.cpp`:

#### 1. Graphics & Resolution (`GraphicsConfig`)
- `renderWidth` / `renderHeight`: Change the size of the 3D viewport.
- `msaaSamples`: Set to 0, 2, 4, or 8 to adjust Anti-Aliasing quality (default is 4).
- `targetFPS`: Limit the frame rate.

#### 2. Key Mappings (`ControlConfig`)
- All keyboard shortcuts (e.g., `bodyPitchUp`, `imuPosXInc`, `saveConfig`) can be reassigned to any `KEY_` constant defined by Raylib.

### IMU Calibration Workflow
1.  **Mounting Alignment:** Align the orange virtual sensor with your physical mounting orientation using keyboard shortcuts or manual input.
2.  **Matrix Export:** Press **P** to save.
3.  **Implementation:** Copy the matrix from `imu_config.txt` into your firmware to transform raw data: $v_{body} = R_{s2b} \cdot v_{sensor}$.



## License
MIT License
