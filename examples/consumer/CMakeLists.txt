cmake_minimum_required(VERSION 3.24)
project(robotics_control_consumer LANGUAGES CXX)

find_package(robotics_control_core 0.1 CONFIG REQUIRED)

add_executable(robotics_control_consumer main.cpp)
target_link_libraries(robotics_control_consumer PRIVATE robotics::control)
target_compile_features(robotics_control_consumer PRIVATE cxx_std_20)
