cmake_minimum_required(VERSION 3.0.2)
project(dps)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs  
  visualization_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(marker_node src/main.cpp)

target_link_libraries(marker_node
  ${catkin_LIBRARIES} ${LIBS}
)
