cmake_minimum_required(VERSION 3.8)
project(science)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)


if (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "arm-linux-gnueabihf") # Jetson TK1 / Pi
    set(PHOENIX_LIBS_DIR ${CMAKE_SOURCE_DIR}/lib/arm32)
elseif (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "aarch64-linux-gnu") # Jetson TX2
    set(PHOENIX_LIBS_DIR ${CMAKE_SOURCE_DIR}/lib/arm64)
else()
    set(PHOENIX_LIBS_DIR ${CMAKE_SOURCE_DIR}/lib/x86-64)
endif()

add_executable(science_node src/science_node.cpp)
target_include_directories(science_node PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/science>
$<INSTALL_INTERFACE:include/ctre>
)
target_compile_features(science_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_link_libraries(science_node 
  ${PHOENIX_LIBS_DIR}/libCTRE_PhoenixTools.so 
  ${PHOENIX_LIBS_DIR}/libCTRE_PhoenixCCI.so 
  ${PHOENIX_LIBS_DIR}/libCTRE_Phoenix.so
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  )

ament_target_dependencies(science_node
    rclcpp
)

set(DIRECTORIES launch)
foreach(DIR ${DIRECTORIES})
  install(DIRECTORY ${DIR} DESTINATION share/science)
endforeach()
ament_export_dependencies(    
  rclcpp
)
install(TARGETS science_node
  DESTINATION lib/${PROJECT_NAME})

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
