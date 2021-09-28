cmake_minimum_required(VERSION 3.0.2)
project(avoidance)

#set(CMAKE_BUILD_TYPE "Debug" )

add_compile_options(-std=c++14)

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  tf
  message_generation
  custom_messages
)

## Generate messages in the 'msg' folder
#  add_message_files(
#    FILES
#    ForceData.msg
#    basic_local.msg
#    local.msg
#  )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#     DEPENDENCIES
#     std_msgs
#  )


#find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)

include_directories(
  include
	${catkin_INCLUDE_DIRS} 
	${OpenCV_INCLUDE_DIR}
	${PCL_INCLUDE_DIRS}
)

link_directories(
  include
  ${PCL_LIBRSRY_DIRS}
  ${OpenCV_LIBRSRY_DIRS}
)

add_message_files(
  FILES
  VehicleStatus.msg
)

generate_messages(
 DEPENDENCIES
 geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime
  DEPENDS EIGEN3 PCL OpenCV
  INCLUDE_DIRS include
)

add_executable(extractObstacle src/extractObstacle.cpp)
target_link_libraries(extractObstacle  ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRSRY_DIRS})
add_dependencies(extractObstacle custom_messages_generate_messages_cpp)

add_executable(extractObstacle2 src/extractObstacle2.cpp)
target_link_libraries(extractObstacle2  ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRSRY_DIRS})
add_dependencies(extractObstacle custom_messages_generate_messages_cpp)

add_executable(pathSave src/pathSave.cpp)
target_link_libraries(pathSave  ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRSRY_DIRS})
add_dependencies(extractObstacle custom_messages_generate_messages_cpp)

