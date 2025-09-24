cmake_minimum_required(VERSION 3.8)
project(object_classification)

# Opcional: estándares y warnings (puedes dejarlos si quieres)
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencias
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Generar interfaces (.srv)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ClassifyDino.srv"
  DEPENDENCIES std_msgs sensor_msgs
)

# (Opcional) Lint en testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export para runtime de las interfaces
ament_export_dependencies(rosidl_default_runtime)

ament_package()
