# 1. Try to find raylib using find_package (modern way)
find_package(raylib QUIET)

if(raylib_FOUND)
    message(STATUS "Found Raylib via find_package")
    if(TARGET raylib)
        set(RAYLIB_LIB raylib)
    else()
        set(RAYLIB_LIB ${raylib_LIBRARIES})
        set(RAYLIB_INC ${raylib_INCLUDE_DIRS})
    endif()
else()
    # 2. Manual search if find_package fails
    message(STATUS "Raylib find_package failed, searching manually...")
    find_path(RAYLIB_INCLUDE_DIR NAMES raylib.h PATHS /usr/local/include /usr/include)
    find_library(RAYLIB_LIBRARY NAMES raylib PATHS /usr/local/lib /usr/lib)
    
    if(RAYLIB_INCLUDE_DIR AND RAYLIB_LIBRARY)
        set(raylib_FOUND TRUE)
        set(RAYLIB_LIB ${RAYLIB_LIBRARY})
        set(RAYLIB_INC ${RAYLIB_INCLUDE_DIR})
        message(STATUS "Found system Raylib at ${RAYLIB_LIBRARY}")
    endif()
endif()

# 3. Fallback to FetchContent if nothing found
if(NOT raylib_FOUND)
    message(STATUS "Raylib not found on system. Falling back to FetchContent...")
    include(FetchContent)

    FetchContent_Declare(
      raylib
      GIT_REPOSITORY https://github.com/raysan5/raylib.git
      GIT_TAG        5.0
      GIT_PROGRESS   TRUE
    )

    set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(BUILD_GAMES OFF CACHE BOOL "" FORCE)

    FetchContent_MakeAvailable(raylib)
    set(RAYLIB_LIB raylib)
    set(RAYLIB_INC "") # Headers handled by target
endif()

add_executable(simulation main.cpp)

if(RAYLIB_INC)
    target_include_directories(simulation PRIVATE ${RAYLIB_INC})
endif()
target_include_directories(simulation PRIVATE ../include)

# Link everything. Order matters for static libraries.
target_link_libraries(simulation PRIVATE ${RAYLIB_LIB} embdC_quat)

# Essential system dependencies for Raylib on Linux
if(UNIX AND NOT APPLE)
    target_link_libraries(simulation PRIVATE 
        m 
        pthread 
        dl 
        rt 
        X11 
        GL 
    )
endif()

set_target_properties(simulation PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/sim")
