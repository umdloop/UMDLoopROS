cmake_minimum_required(VERSION 3.8)
project(diff_drive)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
set(THREADS_PREFER_PTHREAD_FLAG ON)

find_package (Threads REQUIRED)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_library(
    diff_drive
    SHARED
    hardware/src/drive.cpp
    hardware/include/hardware/drive.hpp
)
target_include_directories(diff_drive PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:hardware/include/hardware>
$<INSTALL_INTERFACE:hardware/include/ctre>
)

if (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "arm-linux-gnueabihf") # Jetson TK1 / Pi
    set(PHOENIX_LIBS_DIR ${CMAKE_SOURCE_DIR}/hardware/lib/arm32)
elseif (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "aarch64-linux-gnu") # Jetson TX2
    set(PHOENIX_LIBS_DIR ${CMAKE_SOURCE_DIR}/hardware/lib/arm64)
else()
    set(PHOENIX_LIBS_DIR ${CMAKE_SOURCE_DIR}/hardware/lib/x86-64)
endif()

pluginlib_export_plugin_description_file(hardware_interface hardware_int.xml)


install(FILES 
${PHOENIX_LIBS_DIR}/libCTRE_PhoenixTools.so
${PHOENIX_LIBS_DIR}/libCTRE_PhoenixCCI.so
${PHOENIX_LIBS_DIR}/libCTRE_Phoenix.so
DESTINATION lib/${PROJECT_NAME}
)

ament_target_dependencies(diff_drive
    "hardware_interface"
    "rclcpp"
    "pluginlib"
)

install(
  DIRECTORY hardware/include
  DESTINATION include/hardware
)
install(
  DIRECTORY launch
  DESTINATION share/diff_drive
)
install(
  DIRECTORY ros2_control
  DESTINATION share/diff_drive
)
install(
  DIRECTORY urdf
  DESTINATION share/diff_drive
)
install(
  DIRECTORY rviz
  DESTINATION share/diff_drive
)
install(
  DIRECTORY config
  DESTINATION share/diff_drive
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

ament_package()
