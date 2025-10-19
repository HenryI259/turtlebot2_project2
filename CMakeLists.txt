cmake_minimum_required(VERSION 3.0.2)
project(turtlebot2_project2)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
 roscpp 
 geometry_msgs
 nav_msgs
 kobuki_msgs
 sensor_msgs
 std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES turtlebot2_project2
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
 ${catkin_INCLUDE_DIRS}
)

add_executable(navigate src/navigate.cpp)
target_link_libraries(navigate ${catkin_LIBRARIES})
add_dependencies(navigate ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
