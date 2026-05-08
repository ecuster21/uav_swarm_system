cmake_minimum_required(VERSION 3.8)
project(px4_bridge_uxrce)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(swarm_msgs REQUIRED)
find_package(px4_msgs QUIET)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(px4_msgs_FOUND)
  add_executable(px4_uxrce_bridge src/px4_uxrce_bridge_node.cpp)
  target_compile_features(px4_uxrce_bridge PUBLIC cxx_std_17)
  ament_target_dependencies(
    px4_uxrce_bridge
    rclcpp
    geometry_msgs
    std_srvs
    swarm_msgs
    px4_msgs
  )

  install(
    TARGETS px4_uxrce_bridge
    DESTINATION lib/${PROJECT_NAME}
  )
else()
  message(WARNING "px4_msgs was not found. px4_bridge_uxrce will install launch files only and skip the C++ executable.")
endif()

ament_package()
