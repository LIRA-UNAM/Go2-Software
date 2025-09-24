cmake_minimum_required(VERSION 3.14)
project(go2_demo)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set (CMAKE_CXX_STANDARD 14)
endif()

include(nlohmannJSON_import.cmake)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_api REQUIRED)

set(SPORT_CLIENT_HEADERS "")

set(TARGET_DEPENDENCIES
  unitree_go
  unitree_api
  rclcpp
  std_msgs
)

set(INSTALL_DIR lib/${PROJECT_NAME})

add_subdirectory(sport_client)
add_subdirectory(standsit_demo)
add_subdirectory(walk_demo)
add_subdirectory(dog_base)


# ament_target_dependencies(standsit ${TARGET_DEPENDENCIES})

ament_package()
