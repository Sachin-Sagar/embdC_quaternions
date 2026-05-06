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

### Controls
| Key | Action |
|-----|--------|
| **W / S** | Pitch Up / Down |
| **A / D** | Yaw Left / Right |
| **Q / E** | Roll Left / Right |
| **R** | Reset Rotation |
| **ESC** | Exit |

## License
MIT License
