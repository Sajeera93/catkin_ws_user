cmake_minimum_required(VERSION 2.8.3)
project(camera_pose)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs
  image_transport
)


catkin_package(
LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS std_msgs sensor_msgs geometry_msgs
	DEPENDS Boost
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)

add_executable(camera_pose_node src/camera_pose.cpp)
add_dependencies(camera_pose_node ${PROJECT_NAME})
target_link_libraries(camera_pose_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})

install(TARGETS camera_pose_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

