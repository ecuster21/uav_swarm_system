cmake_minimum_required(VERSION 3.8)
project(swarm_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  ../../../config/swarm.yaml
  ../../../config/formations.yaml
  ../../../config/waypoints.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
