cmake_minimum_required(VERSION 2.8.3)
project(path_planning)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  ras_arduino_msgs
  robot_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(path_planning src/path_planning.cpp)
add_dependencies(path_planning ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_planning ${catkin_LIBRARIES})
