cmake_minimum_required(VERSION 3.8)
project(triple_pendulum_description)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# macro expansion with xacro
add_custom_target(urdf ALL
  COMMAND xacro model.urdf.xacro > model.urdf
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/urdf
)

install(
  DIRECTORY launch urdf rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
