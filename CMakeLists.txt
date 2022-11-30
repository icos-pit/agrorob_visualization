cmake_minimum_required(VERSION 3.8)
project(agrorob_visualization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(agrorob_msgs REQUIRED)
find_package(sensor_msgs  REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros  REQUIRED)
find_package(nav_msgs  REQUIRED)

# set(library_name ${PROJECT_NAME})
# add_library(${library_name} SHARED
#   src/kinematics.cpp
#   src/agrorob_state_publisher.cpp
# )

add_executable(agrorob_state_publisher src/main.cpp src/agrorob_state_publisher.cpp src/kinematics.cpp)

target_include_directories(agrorob_state_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(agrorob_state_publisher PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  agrorob_state_publisher
  "rclpy"
  "rclcpp"
  "agrorob_msgs"
  "sensor_msgs"
  "tf2"
  "tf2_ros"
  "geometry_msgs"
  "nav_msgs"
)

install(DIRECTORY
  launch
  urdf
  meshes
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  meshes
  DESTINATION meshes/${PROJECT_NAME}/
)

install(PROGRAMS
  agrorob_visualization/state_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS agrorob_state_publisher 
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()