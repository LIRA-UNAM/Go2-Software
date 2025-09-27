cmake_minimum_required(VERSION 3.14)
project(dogbase)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_api REQUIRED)

file(GLOB SRC_WALK
  ${PROJECT_SOURCE_DIR}/*.cpp
)

add_executable(dogbase
  ${SRC_WALK}
)

target_link_libraries(dogbase
  sportclient
)


ament_target_dependencies(dogbase
  ${TARGET_DEPENDENCIES}
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
)

install(TARGETS
  dogbase
  DESTINATION ${INSTALL_DIR}
)
