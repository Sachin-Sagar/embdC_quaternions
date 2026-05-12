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
- **Sensor-to-Body Transformation:** Simplified API for transforming vectors from a sensor mounting frame to the vehicle body frame.
- **C++ Simulation Tool:**
    - Real-time 3D visualization using Raylib (Native OpenGL 2.1/ES support).
    - **Dynamic UI Scaling:** UI automatically scales based on window resolution.
    - Interactive HUD for real-time Euler and Quaternion data.
- **Testing:** Comprehensive unit test suite using Google Test.

## Project Structure
- `include/`: C API headers.
- `src/`: Core library implementation (C).
- `sim/`: Simulation tool source (C++).
- `tests/`: Unit test suite.

## Getting Started

### Prerequisites
- **Compiler:** GCC (C11) and G++ (C++17).
- **Build Tool:** CMake (>= 3.10).
- **Dependencies (Linux/Debian):**
  ```bash
  sudo apt update
  sudo apt install build-essential cmake git mesa-utils
  # Required for simulation tool:
  sudo apt install libasound2-dev libx11-dev libxrandr-dev libxi-dev libxcursor-dev libxinerama-dev libgl1-mesa-dev libglu1-mesa-dev
  ```

### Building the Project
By default, the simulation is disabled to ensure fast builds on low-power devices (like Raspberry Pi).

```bash
mkdir build && cd build
cmake ..
make
```

To build with the **3D Simulation tool**:
```bash
cmake .. -DBUILD_SIMULATION=ON
make
```

### Running the Unit Tests
```bash
./tests/test_quat        # Floating-point tests
./tests/test_quat_fixed  # Fixed-point tests
```

## Simulation Usage

### Raspberry Pi 4 Optimization
For the best experience on a Raspberry Pi 4, it is highly recommended to install Raylib globally using **OpenGL 2.1**:

```bash
git clone https://github.com/raysan5/raylib.git
cd raylib/build
cmake -DPLATFORM=Desktop -DGRAPHICS=GRAPHICS_API_OPENGL_21 -DBUILD_EXAMPLES=OFF ..
make -j$(nproc)
sudo make install
```

### Running the Tool
```bash
./sim/simulation
```
*If running over SSH on a Pi with a monitor attached: `DISPLAY=:0 ./sim/simulation`*

### Troubleshooting (Raspberry Pi)
If you see `GLXBadFBConfig`:
1. Ensure **Full KMS** is enabled in `sudo raspi-config` (Advanced Options -> GL Driver).
2. Increase GPU Memory to at least 128MB in `/boot/config.txt`.
3. Force OpenGL version: `MESA_GL_VERSION_OVERRIDE=2.1 ./sim/simulation`

## Implementation Details

### Floating-Point Optimization
Vector rotation is implemented using an optimized formula that avoids full quaternion multiplication:
```c
vec3_t q_x_v = vec3_cross(q_vec, v);
vec3_t t = { 2.0f * q_x_v.x, 2.0f * q_x_v.y, 2.0f * q_x_v.z };
v_prime = v + q.w * t + (q_vec x t);
```

### Fixed-Point (Q30)
- **1.0** is represented as `1 << 30`.
- **Intermediate Math:** Uses `int64_t` for products to ensure zero overflow before the shift.

## License
MIT License
