cmake_minimum_required(VERSION 2.8.3)
project(path_planning)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  ras_arduino_msgs
  robot_msgs
  rosbag
)

catkin_package(
 CATKIN_DEPENDS roscpp
    robot_msgs
    rosbag
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(path_planning_main src/path_planning_main.cpp)
add_dependencies(path_planning_main ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_planning_main ${catkin_LIBRARIES})

add_executable(path_planning src/path_planning.cpp)
add_dependencies(path_planning ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_planning ${catkin_LIBRARIES})

add_executable(path_planning_new src/path_planning_new.cpp)
add_dependencies(path_planning_new ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_planning_new ${catkin_LIBRARIES})

add_executable(path_navigation src/path_navigation.cpp)
add_dependencies(path_navigation ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_navigation ${catkin_LIBRARIES})


add_executable(goal_finding src/goal_finding.cpp)
add_dependencies(goal_finding ${catkin_EXPORTED_TARGETS})
target_link_libraries(goal_finding ${catkin_LIBRARIES})

