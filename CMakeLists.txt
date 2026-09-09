cmake_minimum_required(VERSION 3.24)
project(robotics_control_core VERSION 0.1.0 LANGUAGES CXX)

option(RCC_BUILD_TESTS "Build unit tests" ON)
option(RCC_ENABLE_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" ON)
option(RCC_ENABLE_SANITIZERS "Enable ASan/UBSan where supported" OFF)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(robotics_control_core
    src/a_star.cpp
    src/differential_drive.cpp
    src/map_inflation.cpp
    src/odometry.cpp
    src/path_follower.cpp
    src/path_smoother.cpp
    src/pid.cpp
    src/pure_pursuit.cpp
)
add_library(robotics::control ALIAS robotics_control_core)
set_target_properties(robotics_control_core PROPERTIES EXPORT_NAME control)

target_include_directories(robotics_control_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_compile_features(robotics_control_core PUBLIC cxx_std_20)

function(rcc_enable_project_warnings target)
    if(MSVC)
        target_compile_options(${target} PRIVATE /W4 /permissive-)
        if(RCC_ENABLE_WARNINGS_AS_ERRORS)
            target_compile_options(${target} PRIVATE /WX)
        endif()
    else()
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
        if(RCC_ENABLE_WARNINGS_AS_ERRORS)
            target_compile_options(${target} PRIVATE -Werror)
        endif()
    endif()
endfunction()

rcc_enable_project_warnings(robotics_control_core)

if(RCC_ENABLE_SANITIZERS AND NOT MSVC)
    target_compile_options(robotics_control_core PUBLIC -fsanitize=address,undefined -fno-omit-frame-pointer)
    target_link_options(robotics_control_core PUBLIC -fsanitize=address,undefined)
endif()

add_executable(robotics_control_demo app/demo.cpp)
target_link_libraries(robotics_control_demo PRIVATE robotics::control)
rcc_enable_project_warnings(robotics_control_demo)

add_executable(robotics_control_benchmark app/benchmark.cpp)
target_link_libraries(robotics_control_benchmark PRIVATE robotics::control)
rcc_enable_project_warnings(robotics_control_benchmark)

if(RCC_BUILD_TESTS)
    include(CTest)
    enable_testing()
    add_executable(robotics_control_tests tests/test_main.cpp)
    target_link_libraries(robotics_control_tests PRIVATE robotics::control)
    rcc_enable_project_warnings(robotics_control_tests)

    add_executable(robotics_control_v02_tests tests/test_v02.cpp)
    target_link_libraries(robotics_control_v02_tests PRIVATE robotics::control)
    rcc_enable_project_warnings(robotics_control_v02_tests)

    add_test(NAME robotics_control_tests COMMAND robotics_control_tests)
    add_test(NAME robotics_control_v02_tests COMMAND robotics_control_v02_tests)
    add_test(NAME robotics_control_benchmark COMMAND robotics_control_benchmark)
endif()

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

install(
    TARGETS robotics_control_core
    EXPORT robotics_control_coreTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(
    EXPORT robotics_control_coreTargets
    FILE robotics_control_coreTargets.cmake
    NAMESPACE robotics::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/robotics_control_core
)

configure_package_config_file(
    cmake/robotics_control_coreConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/robotics_control_coreConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/robotics_control_core
)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/robotics_control_coreConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)
install(
    FILES
        ${CMAKE_CURRENT_BINARY_DIR}/robotics_control_coreConfig.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/robotics_control_coreConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/robotics_control_core
)
