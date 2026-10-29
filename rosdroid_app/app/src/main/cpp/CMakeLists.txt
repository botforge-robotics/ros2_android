cmake_minimum_required(VERSION 3.22.1)

project("rosdroid_native")

# 1. Native bridge for Madgwick sensor fusion and JNI hooks
add_library(
    native_bridge
    SHARED
    native_bridge.cpp
    madgwick.cpp
)

find_library(
    log-lib
    log
)

target_link_libraries(
    native_bridge
    ${log-lib}
    m
)
