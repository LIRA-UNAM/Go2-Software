cmake_minimum_required(VERSION 3.14)
project(walk)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_api REQUIRED)

file(GLOB SRC_WALK
  ${PROJECT_SOURCE_DIR}/*.cpp
)

add_executable(walk_demo
  ${SRC_WALK}
)

target_link_libraries(walk_demo
  sportclient
)

ament_target_dependencies(walk_demo
  ${TARGET_DEPENDENCIES}
)

install(TARGETS
  walk_demo
  DESTINATION ${INSTALL_DIR}
)

# ament_package()
