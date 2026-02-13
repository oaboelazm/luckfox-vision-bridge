cmake_minimum_required(VERSION 3.16)
project(luckfox_vision_bridge_native LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(LVB_USE_OPENCV "Enable OpenCV-based AprilTag detection" ON)

set(LVB_OPENCV_FOUND OFF)
if(LVB_USE_OPENCV)
    # Try standard CMake package first.
    find_package(OpenCV QUIET COMPONENTS core imgproc videoio aruco calib3d)
    if(OpenCV_FOUND)
        set(LVB_OPENCV_FOUND ON)
        set(LVB_OPENCV_INCLUDE_DIRS ${OpenCV_INCLUDE_DIRS})
        set(LVB_OPENCV_LIBS ${OpenCV_LIBS})
        add_compile_definitions(LVB_HAVE_OPENCV=1)
    else()
        # Buildroot often exposes OpenCV through pkg-config only.
        find_package(PkgConfig QUIET)
        if(PkgConfig_FOUND)
            pkg_check_modules(OPENCV4 QUIET opencv4)
            if(OPENCV4_FOUND)
                set(LVB_OPENCV_FOUND ON)
                set(LVB_OPENCV_INCLUDE_DIRS ${OPENCV4_INCLUDE_DIRS})
                set(LVB_OPENCV_LIBS ${OPENCV4_LIBRARIES})
                add_compile_definitions(LVB_HAVE_OPENCV=1)
                message(STATUS "Using OpenCV via pkg-config (opencv4).")
            endif()
        endif()
    endif()

    if(NOT LVB_OPENCV_FOUND)
        message(WARNING "OpenCV not found (CMake package nor pkg-config): building without AprilTag detection support.")
    endif()
endif()

add_executable(luckfox_bridge
    src/main.cpp
)

if(LVB_OPENCV_FOUND)
    target_include_directories(luckfox_bridge PRIVATE ${LVB_OPENCV_INCLUDE_DIRS})
    target_link_libraries(luckfox_bridge PRIVATE ${LVB_OPENCV_LIBS})
endif()

if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(luckfox_bridge PRIVATE -O3 -Wall -Wextra -Wpedantic)
endif()
