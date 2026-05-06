# EmbdC Quaternions

A lightweight, pure C library for quaternion-based coordinate transformations, featuring a 3D C++ simulation tool built with Raylib. This project is designed for embedded systems where sensor-to-body frame transformations (e.g., IMU mounting) are critical.

## Features
- **Pure C Math Library (`embdC_quat`):**
    - High-performance Vector and Quaternion arithmetic.
    - ZYX Euler angle conversions (Roll, Pitch, Yaw).
    - Sensor-to-Body coordinate transformation.
    - Right-Handed Coordinate System support.
- **C++ Simulation Tool:**
    - Real-time 3D visualization using Raylib.
    - **Z-Up Orientation:** Follows standard aerospace and robotics conventions.
    - Interactive HUD showing real-time Euler and Quaternion data.
    - Visual axis markers for World, Body, and Sensor frames.
- **Testing:**
    - Custom unit test suite to verify mathematical correctness.

## Project Structure
- `include/`: C API headers.
- `src/`: Core library implementation (C).
- `sim/`: Simulation tool source (C++).
- `tests/`: Unit test suite.
- `Makefile`: Unified build system.

## Getting Started

### Prerequisites
- **Compiler:** GCC (C11) and G++ (C++17).
- **Build Tool:** Make.
- **Dependencies:** [Raylib](https://www.raylib.com/) (Required for simulation only).

### Installation (Raylib on Linux)
If Raylib is not in your package manager, build it from source:
```bash
sudo apt install libasound2-dev libx11-dev libxrandr-dev libxi-dev libgl1-mesa-dev libglu1-mesa-dev libxcursor-dev libxinerama-dev
git clone --depth 1 https://github.com/raysan5/raylib.git
cd raylib/src && make PLATFORM=PLATFORM_DESKTOP
sudo make install
```

### Building the Project
```bash
# Build the library, tests, and simulation
make

# Run the math unit tests
make test
```

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

### Visual Guide
- **World Frame:** Static grid at $Z=0$.
    - **Red:** X | **Green:** Y | **Blue:** Z (UP)
- **Body Frame:** Large gray cuboid representing the vehicle.
- **Sensor Frame:** Small orange box representing the IMU/Sensor.

## Math Reference
The library uses the **Hamilton product** for quaternion multiplication and performs vector rotation using:
$$v' = q \cdot v \cdot q^{-1}$$
For unit quaternions, this simplifies to using the conjugate for the inverse.
