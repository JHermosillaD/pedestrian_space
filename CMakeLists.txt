cmake_minimum_required(VERSION 3.0.2)
project(pedestrian_space)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs  
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(display_marker src/main.cpp)

target_link_libraries(display_marker
  ${catkin_LIBRARIES} ${LIBS}
)
