cmake_minimum_required(VERSION 3.8)
project(geodude_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)

## libbarrett
find_package(Barrett REQUIRED)
include_directories(${BARRETT_INCLUDE_DIRS})
message(STATUS "BARRETT_INCLUDE_DIRS: ${BARRETT_INCLUDE_DIRS}")
link_directories(${BARRETT_LIBRARY_DIRS})
message(STATUS "BARRETT_LIBRARY_DIRS: ${BARRETT_LIBRARY_DIRS}")
add_definitions(${BARRETT_DEFINITIONS})
message(STATUS "BARRETT_DEFINITIONS: ${BARRETT_DEFINITIONS}")

add_library(geodude_hardware SHARED
  src/barrett.cpp
  src/vention_api.cpp
  src/vention.cpp)

target_include_directories(geodude_hardware PUBLIC include)

target_link_libraries(geodude_hardware ${BARRETT_LIBRARIES})

ament_target_dependencies(geodude_hardware rclcpp rclcpp_lifecycle pluginlib hardware_interface std_srvs sensor_msgs)

pluginlib_export_plugin_description_file(hardware_interface geodude_hardware.xml)

# Zero-Barrett Executable
add_executable(zero_barrett src/zero_barrett.cpp)
target_link_libraries(zero_barrett ${BARRETT_LIBRARIES})
ament_target_dependencies(zero_barrett ament_index_cpp rclcpp)

# Time Vention Executable
add_executable(time_vention src/time_vention.cpp)
add_executable(zero_vention src/zero_vention.cpp)
target_link_libraries(time_vention geodude_hardware)
target_link_libraries(zero_vention geodude_hardware)

# Install (required for pluginlib)
install(
  DIRECTORY include/
  DESTINATION include/geodude_hardware
)
install(TARGETS geodude_hardware
  EXPORT export_geodude_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  zero_barrett
  time_vention
  zero_vention
  DESTINATION lib/${PROJECT_NAME})


install(
  DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  ### Linting
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)
  find_package(ament_cmake_copyright REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ### GMock
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)
  ament_add_gmock(test_barrett test/test_barrett.cpp)
  ament_add_gmock(test_vention test/test_vention.cpp)
  ament_target_dependencies(test_barrett
    pluginlib
    ros2_control_test_assets
    rclcpp
    rclcpp_lifecycle
    hardware_interface
  )
  ament_target_dependencies(test_vention
    pluginlib
    ros2_control_test_assets
    rclcpp
    rclcpp_lifecycle
    hardware_interface
  )
endif()

ament_export_dependencies(rclcpp rclcpp_lifecycle pluginlib hardware_interface)
ament_package()
