cmake_minimum_required(VERSION 3.14)
project(go2_demo)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(nlohmann_json 3.2 REQUIRED)
# Sin internet en el robot: instalar con: sudo apt install nlohmann-json3-dev

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(unitree_api REQUIRED)

set(INSTALL_DIR lib/${PROJECT_NAME})

add_subdirectory(sport_client)
add_subdirectory(dog_base)

ament_package()
