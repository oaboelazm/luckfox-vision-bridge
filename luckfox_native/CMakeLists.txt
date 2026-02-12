cmake_minimum_required(VERSION 3.16)
project(luckfox_vision_bridge_native LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(LVB_USE_OPENCV "Enable OpenCV-based AprilTag detection" ON)

if(LVB_USE_OPENCV)
    find_package(OpenCV COMPONENTS core imgproc videoio aruco calib3d)
    if(OpenCV_FOUND)
        add_compile_definitions(LVB_HAVE_OPENCV=1)
    else()
        message(WARNING "OpenCV not found: building without AprilTag detection support.")
    endif()
endif()

add_executable(luckfox_bridge
    src/main.cpp
)

if(OpenCV_FOUND)
    target_link_libraries(luckfox_bridge PRIVATE ${OpenCV_LIBS})
endif()

if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(luckfox_bridge PRIVATE -O3 -Wall -Wextra -Wpedantic)
endif()
