cmake_minimum_required(VERSION 3.0.2)
project(pend)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg
)

generate_messages(DEPENDENCIES std_msgs)


include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(pend src/pend.cpp)
target_link_libraries(pend ${catkin_LIBRARIES})
add_dependencies(pend ${catkin_EXPORTED_TARGETS})

