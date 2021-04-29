cmake_minimum_required(VERSION 3.0.2)
project(magut_eye_on_base)

add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  roscpp
  tf
  tf_conversions
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_core
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES magut_eye_on_base
  CATKIN_DEPENDS eigen_conversions roscpp tf tf_conversions moveit_ros_planning moveit_core moveit_ros_planning_interface
  DEPENDS
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/magut_eye_on_base_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
