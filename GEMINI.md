# EmbdC Quaternions - Project Context

## Project Overview
EmbdC Quaternions is a specialized, high-performance C library designed for quaternion-based coordinate transformations in embedded systems. It is particularly optimized for sensor-to-body frame mappings, common in robotics, aerospace, and navigation systems (e.g., IMU data transformation).

### Key Features
- **Dual Implementations:**
    - **Floating-Point (`embdC_quat`):** Optimized for hardware FPUs (Cortex-M4F/M7), using single-precision `float` to prevent implicit `double` promotion.
    - **Fixed-Point (`embdC_quat_fixed`):** Uses **Q30** format (`int32_t`) for deterministic performance on systems without an FPU (Cortex-M0/M3).
- **Optimization:** Implements an optimized vector rotation formula ($v' = v + 2q_{xyz} \times (q_{xyz} \times v + w v)$) that reduces operations by over 50% compared to standard quaternion multiplication.
- **Simulation Tool:** A C++ Raylib-based application for 3D visualization of transformations, gimbal lock demonstration, and IMU calibration.
- **Testing:** Comprehensive unit tests using Google Test.

## Technical Stack
- **Language:** C11 (Core Library), C++17 (Simulation & Tests).
- **Build System:** CMake.
- **Graphics Library:** Raylib (for `sim/`).
- **Test Framework:** Google Test.

## Directory Structure
- `include/`: Public C API headers (`embdC_quat.h`, `embdC_quat_fixed.h`).
- `src/`: Library implementation logic.
- `sim/`: Source code for the 3D visualization tool.
- `tests/`: Unit test suite.
- `imu_config.txt`: Generated configuration file from the simulation tool.
- `math_explanation.html`: Detailed documentation on the mathematical background.

## Building and Running

### Building the Project
```bash
mkdir build && cd build
cmake ..
make
```

### Running Tests
```bash
./tests/test_quat        # Floating-point tests
./tests/test_quat_fixed  # Fixed-point tests
```

### Running Simulation
```bash
./sim/simulation
```

## Development Conventions

### Coding Style
- **Pure C for Core:** The core library must remain pure C (C99/C11) to ensure compatibility with embedded compilers.
- **Floating-Point Precision:** Always use the `f` suffix for float literals (e.g., `1.0f`) to avoid promotion to `double`.
- **Naming:** Follow the existing prefix-based naming convention (e.g., `quat_`, `vec3_`, `euler_`).
- **Fixed-Point Arithmetic:** Use `int64_t` for intermediate products in Q30 calculations to prevent overflow before the right-shift.

### Mathematics
- **Quaternion Convention:** $w + xi + yj + zk$.
- **Euler Angles:** ZYX convention (Roll $\to$ X, Pitch $\to$ Y, Yaw $\to$ Z).
- **Coordinate System:** Z-Up orientation (standard in aerospace/robotics).

### Testing
- All new math features must be accompanied by Google Test cases in the `tests/` directory.
- Fixed-point implementations should be validated against their floating-point counterparts for precision metrics.

## Key Files
- `include/embdC_quat.h`: Main floating-point API.
- `include/embdC_quat_fixed.h`: Fixed-point (Q30) API.
- `src/embdC_quat.c`: Implementation of optimized rotation and Euler conversions.
- `sim/main.cpp`: Interactive 3D simulation and calibration tool.
- `PROJECT_PLAN.md`: Original roadmap and implementation phases.
