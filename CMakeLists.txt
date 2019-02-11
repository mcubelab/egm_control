cmake_minimum_required(VERSION 2.8.3)
project(egm_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  kdl_parser
  urdf
  trac_ik_lib
)
find_package(Protobuf REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES egm_control
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${PROTOBUF_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_executable(EGMControl src/EGMControl.cpp src/ROSHelper.cpp src/RobotController.cpp src/helpers/EGMHelper.cpp src/helpers/PracticalSocket.cpp src/helpers/egm.pb.cc)
target_link_libraries(EGMControl ${catkin_LIBRARIES} ${PROTOBUF_LIBRARY})
# add_dependencies(EGMControl)
