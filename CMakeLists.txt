cmake_minimum_required(VERSION 3.10)
project(embdC_quaternions VERSION 0.1.0 LANGUAGES C CXX)
message(STATUS "Starting configuration for ${PROJECT_NAME}...")

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Add Library
message(STATUS "Configuring core library...")
add_subdirectory(src)

# Add Tests
option(BUILD_TESTS "Build tests" ON)
if(BUILD_TESTS)
    message(STATUS "Tests enabled. Configuring GoogleTest...")
    include(FetchContent)
    FetchContent_Declare(
      googletest
      GIT_REPOSITORY https://github.com/google/googletest.git
      GIT_TAG        v1.14.0 # Use a stable tag
      GIT_PROGRESS   TRUE
    )
    # For Windows: Prevent overriding the parent project's compiler/linker settings
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    
    message(STATUS "Fetching GoogleTest (this may take a while on slow connections)...")
    FetchContent_MakeAvailable(googletest)
    message(STATUS "GoogleTest is ready.")
    
    add_subdirectory(tests)
endif()

# Add Simulation
option(BUILD_SIMULATION "Build simulation tool (requires Raylib)" OFF)
if(BUILD_SIMULATION)
    message(STATUS "Configuring simulation tool...")
    add_subdirectory(sim)
else()
    message(STATUS "Simulation tool is disabled (BUILD_SIMULATION=OFF).")
endif()
message(STATUS "Configuration complete.")
