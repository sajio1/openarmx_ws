# Copyright 2025 成都长数机器人有限公司 (Chengdu Changsu Robot Co., Ltd.)
# Website: https://openarmx.com/
# Contact: Mr Wang
# Phone & WeChat: +86-17746530375
# Email: openarmrobot@gmail.com
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author Information:
# Company: 成都长数机器人有限公司 (Chengdu Changsu Robot Co., Ltd.)
# Website: https://openarmx.com/
# Contact Person: Mr Wang
# Phone & WeChat: +86-17746530375
# Email: openarmrobot@gmail.com

cmake_minimum_required(VERSION 3.22)
project(openarmx_can VERSION 1.0.0)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ament_cmake for ROS2 package support
find_package(ament_cmake REQUIRED)

# Testing support
include(CTest)

include(GNUInstallDirs)

# Create the main library
add_library(
  openarmx_can
  src/openarm/can/socket/arm_component.cpp
  src/openarm/can/socket/gripper_component.cpp
  src/openarm/can/socket/openarm.cpp
  src/openarm/canbus/can_device_collection.cpp
  src/openarm/canbus/can_socket.cpp
  src/openarm/robstride_motor/rs_motor.cpp
  src/openarm/robstride_motor/rs_motor_control.cpp
  src/openarm/robstride_motor/rs_motor_device.cpp
  src/openarm/robstride_motor/rs_motor_device_collection.cpp)

set_target_properties(openarmx_can PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(
  openarmx_can PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                              $<INSTALL_INTERFACE:include>)

install(TARGETS openarmx_can EXPORT openarmx_can_export)

# CMake package
set(INSTALL_CMAKE_DIR ${CMAKE_INSTALL_LIBDIR}/cmake/OpenArmCANOpenArmXCom)
install(
  EXPORT openarmx_can_export
  DESTINATION ${INSTALL_CMAKE_DIR}
  NAMESPACE OpenArmCANOpenArmXCom::
  FILE OpenArmCANOpenArmXComTargets.cmake)

include(CMakePackageConfigHelpers)
configure_package_config_file(
  OpenArmXCANConfig.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/OpenArmXCANConfig.cmake
  INSTALL_DESTINATION ${INSTALL_CMAKE_DIR})

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/OpenArmXCANVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/OpenArmXCANConfig.cmake
              ${CMAKE_CURRENT_BINARY_DIR}/OpenArmXCANVersion.cmake
        DESTINATION ${INSTALL_CMAKE_DIR})

# pkg-config package
if(IS_ABSOLUTE "${CMAKE_INSTALL_INCLUDEDIR}")
  set(PKG_CONFIG_INCLUDEDIR "${CMAKE_INSTALL_INCLUDEDIR}")
else()
  set(PKG_CONFIG_INCLUDEDIR "\${prefix}/${CMAKE_INSTALL_INCLUDEDIR}")
endif()

if(IS_ABSOLUTE "${CMAKE_INSTALL_LIBDIR}")
  set(PKG_CONFIG_LIBDIR "${CMAKE_INSTALL_LIBDIR}")
else()
  set(PKG_CONFIG_LIBDIR "\${prefix}/${CMAKE_INSTALL_LIBDIR}")
endif()

configure_file(openarmx-can.pc.in ${CMAKE_CURRENT_BINARY_DIR}/openarmx-can.pc
               @ONLY)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/openarmx-can.pc
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig)

# Add Robstride demo executable
add_executable(robstride-demo examples/robstride_demo.cpp)
target_link_libraries(robstride-demo openarmx_can)

# Add Python-style test executable
add_executable(python-style-test examples/python_style_test.cpp)
target_link_libraries(python-style-test openarmx_can)

# Add Robstride motion test executable (commented out - file not found)
# add_executable(robstride-motion-test examples/robstride_motion_test.cpp)
# target_link_libraries(robstride-motion-test openarm_can_openarmx)

# Add Robstride full demo executable (7 arm motors + 1 gripper)
add_executable(robstride-demo-full examples/robstride_demo_full.cpp)
target_link_libraries(robstride-demo-full openarmx_can)

# Add Robstride gripper demo executable (gripper only)
add_executable(robstride-demo-gripper examples/robstride_demo_gripper.cpp)
target_link_libraries(robstride-demo-gripper openarmx_can)

# Add motor check executable for Robstride setup
add_executable(motor-check setup/motor_check.cpp)
target_link_libraries(motor-check openarmx_can)

# Add tests
if(BUILD_TESTING)
  # add_subdirectory(test)
endif()

# Install headers
install(DIRECTORY include/
        DESTINATION include)

# Install libraries
install(TARGETS openarmx_can
        EXPORT openarmx_canTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include)

# Install executables
install(TARGETS robstride-demo python-style-test robstride-demo-full robstride-demo-gripper motor-check
        DESTINATION lib/${PROJECT_NAME})

# Export the targets
install(EXPORT openarmx_canTargets
        FILE openarmx_canTargets.cmake
        NAMESPACE openarmx_can::
        DESTINATION lib/cmake/openarmx_can)

# Configure and install the config file
include(CMakePackageConfigHelpers)
configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/openarmx_canConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/openarmx_canConfig.cmake
  INSTALL_DESTINATION lib/cmake/openarmx_can
)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/openarmx_canConfig.cmake
        DESTINATION lib/cmake/openarmx_can)

# Export for ament
ament_export_targets(openarmx_canTargets HAS_LIBRARY_TARGET)
ament_export_dependencies()

ament_package()
