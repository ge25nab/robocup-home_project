cmake_minimum_required(VERSION 2.8.3)
project(grasp)
add_compile_options(-std=c++11)
# Catkin
find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  std_msgs
  sensor_msgs
  message_filters
  cv_bridge
  tf
  image_geometry
  perception_msgs
  geometry_msgs
  std_srvs
  tf_lookup
  moveit_ros_planning_interface
#>>>>TODO: Add all the dependencies you need. Don't forget to update also the package manifest (package.xml).

)

#find_package(OpenCV REQUIRED)
#find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
# INCLUDE_DIRS include
LIBRARIES grasp
CATKIN_DEPENDS message_runtime
)

include_directories(
	include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

#>>>>TODO: Create a library for the class declared in the files.

#>>>>TODO: Link the executable with the library.

#add_library(grasp src/grasp.cpp)
#target_link_libraries(grasp
#  ${catkin_LIBRARIES}
#  ${OpenCV_LIBRARIES}
#  ${PCL_LIBRARIES}
#)

add_library(tiago_grasp src/grasp_class.cpp)
target_link_libraries(tiago_grasp  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} ${EIGEN3_LIBRARIES})
add_dependencies(tiago_grasp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(grasp src/grasp.cpp)
target_link_libraries(grasp
  
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  tiago_grasp
)
add_dependencies(grasp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_executable(tftransform src/tf2.cpp)
target_link_libraries(tftransform
  
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  tiago_grasp
)
add_dependencies(tftransform ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#install(TARGETS node_2Dto3D
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

