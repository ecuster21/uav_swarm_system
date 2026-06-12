cmake_minimum_required(VERSION 3.8)
project(formation_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(swarm_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(formation_controller src/formation_controller_node.cpp)
ament_target_dependencies(
  formation_controller
  geometry_msgs
  rclcpp
  swarm_msgs
)
target_link_libraries(formation_controller yaml-cpp)
target_compile_features(formation_controller PUBLIC cxx_std_17)

add_executable(local_follower_controller src/local_follower_controller_node.cpp)
ament_target_dependencies(
  local_follower_controller
  geometry_msgs
  rclcpp
  swarm_msgs
)
target_link_libraries(local_follower_controller yaml-cpp)
target_compile_features(local_follower_controller PUBLIC cxx_std_17)

install(TARGETS formation_controller local_follower_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
