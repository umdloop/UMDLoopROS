cmake_minimum_required(VERSION 3.8)
project(loop_hw)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(myactuator_rmd REQUIRED)

add_library(
    loop_hw
    SHARED

    src/hardware_interfaces/rover_drivetrain.cpp
    include/hardware_interfaces/rover_drivetrain.hpp
)


target_compile_features(loop_hw PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(loop_hw PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
    loop_hw
    "hardware_interface"
    "rclcpp"
    myactuator_rmd
    "pluginlib"
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(loop_hw PRIVATE "urc_hw_BUILDING_LIBRARY")
pluginlib_export_plugin_description_file(hardware_interface loop_hw.xml)
install(
    DIRECTORY include/
    DESTINATION include
)
install(
    TARGETS loop_hw
    EXPORT export_${PROJECT_NAME}
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

ament_export_include_directories(
    include
)
ament_export_libraries(
    loop_hw
)
ament_export_targets(
    export_${PROJECT_NAME}
)

ament_package()