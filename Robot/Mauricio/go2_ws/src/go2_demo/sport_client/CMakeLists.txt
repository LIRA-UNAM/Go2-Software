cmake_minimum_required(VERSION 3.14)
project(sportclient)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# find_package(unitree_go REQUIRED)
find_package(unitree_api REQUIRED)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
)

file(GLOB SRC_SPORTCLI
  ${PROJECT_SOURCE_DIR}/*.cpp
)

add_library(sportclient SHARED
  ${SRC_SPORTCLI}
)

target_include_directories(sportclient
  PUBLIC
  ${PROJECT_SOURCE_DIR}/include
  PRIVATE
  ${PROJECT_SOURCE_DIR}/include/sport_client
)

target_link_libraries(sportclient
  nlohmann_json::nlohmann_json
)

ament_target_dependencies(sportclient
  ${TARGET_DEPENDENCIES}
)

install(TARGETS
  sportclient
)

# ament_package()
