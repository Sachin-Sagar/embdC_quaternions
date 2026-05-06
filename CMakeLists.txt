cmake_minimum_required(VERSION 3.10)
project(embdC_quaternions VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Add Library
add_subdirectory(src)

# Add Tests
option(BUILD_TESTS "Build tests" ON)
if(BUILD_TESTS)
    include(FetchContent)
    FetchContent_Declare(
      googletest
      URL https://github.com/google/googletest/archive/03597a01ee9d7da4044a0e7cd82c16d25f273b4e.zip
    )
    # For Windows: Prevent overriding the parent project's compiler/linker settings
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
    
    add_subdirectory(tests)
endif()

# Add Simulation (if Raylib is found or as a subproject)
# For now, we'll just set up the subdirectory, implementation will follow
# add_subdirectory(sim)
