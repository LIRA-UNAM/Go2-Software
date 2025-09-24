cmake_minimum_required(VERSION 3.14)
project(standsit)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_api REQUIRED)

file(GLOB SRC_STANDSIT
  ${PROJECT_SOURCE_DIR}/*.cpp
)

add_executable(standsit_demo
  ${SRC_STANDSIT}
)


# target_include_directories(standsit_demo
#   PUBLIC
#   ${SPORT_CLIENT_HEADERS}
# )

target_link_libraries(standsit_demo
  sportclient
)


ament_target_dependencies(standsit_demo
  ${TARGET_DEPENDENCIES}
)

install(TARGETS
  standsit_demo
  DESTINATION ${INSTALL_DIR}
)

# ament_package()
