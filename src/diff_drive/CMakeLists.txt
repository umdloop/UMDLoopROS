cmake_minimum_required(VERSION 3.8)
project(diff_drive)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)


if (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "arm-linux-gnueabihf") # Jetson TK1 / Pi
    set(PHOENIX_LIBS_DIR ${CMAKE_SOURCE_DIR}/hardware/lib/arm32)
elseif (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "aarch64-linux-gnu") # Jetson TX2
    set(PHOENIX_LIBS_DIR ${CMAKE_SOURCE_DIR}/hardware/lib/arm64)
else()
    set(PHOENIX_LIBS_DIR ${CMAKE_SOURCE_DIR}/hardware/lib/x86-64)
endif()

add_library(
    diff_drive
    SHARED
    hardware/src/drive.cpp
    hardware/include/diff_drive/drive.hpp
)
target_include_directories(diff_drive PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:hardware/include/hardware>
$<INSTALL_INTERFACE:hardware/include/ctre>
)

pluginlib_export_plugin_description_file(hardware_interface diff_drive.xml)

TARGET_LINK_LIBRARIES(diff_drive ${PHOENIX_LIBS_DIR}/libCTRE_PhoenixTools.so ${PHOENIX_LIBS_DIR}/libCTRE_PhoenixCCI.so ${PHOENIX_LIBS_DIR}/libCTRE_Phoenix.so)


ament_target_dependencies(diff_drive
    hardware_interface
    rclcpp
    pluginlib
)
set(DIRECTORIES launch ros2_control urdf rviz config)
foreach(DIR ${DIRECTORIES})
  install(DIRECTORY ${DIR} DESTINATION share/diff_drive)
endforeach()

install(DIRECTORY hardware/include DESTINATION include/hardware)

install(TARGETS diff_drive
  EXPORT export_diff_drive
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
ament_export_targets(export_diff_drive
HAS_LIBRARY_TARGET)

ament_export_dependencies(    
  hardware_interface
  rclcpp
  pluginlib
)

ament_package()