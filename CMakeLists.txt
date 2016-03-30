cmake_minimum_required(VERSION 2.8.3)
project(masterslave)

set(CMAKE_BUILD_TYPE Debug)


add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
  eigen_conversions
  cmake_modules
  tf
  tf_conversions
  dynamic_reconfigure
)

include_directories(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)

set(OpenIGTLink_DIR "/usr/local/lib/igtl/")
find_package(OpenIGTLink REQUIRED)
include(${OpenIGTLink_USE_FILE})



find_package(Eigen REQUIRED)
## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)




################################################
## Declare ROS messages, services and actions ##
################################################
add_service_files(FILES
    OpenIGTLStateService.srv
    NumericKinematicRCM.srv
    NumericKinematicInverseKinematics.srv
    NumericKinematicDirectKinematics.srv
    GeometricKinematicRCM.srv
    GeometricKinematicInverseKinematics.srv
    GeometricKinematicDirectKinematics.srv
    Manipulation.srv
  )


## Generate messages in the 'msg' folder
add_message_files(
   FILES
   Button.msg
   #Message2.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
   geometry_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

generate_dynamic_reconfigure_options(
  cfg/BoundingBox.cfg
  cfg/MasterSlave.cfg
  cfg/ControlDevice.cfg
  cfg/NumericKinematic.cfg
  cfg/RosOpenIGTLBridge.cfg
  cfg/TrajectoryGenerator.cfg
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES masterslave
  CATKIN_DEPENDS roscpp std_msgs message_runtime
  DEPENDS Eigen
#  DEPENDS system_lib
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  include
  include/masterslave
  include/masterslave/kinematic
  include/masterslave/optimization
)


## Declare a C++ executable
add_executable(ControlDevice src/ControlDevice.cpp include/masterslave/ControlDevice.h)
add_dependencies(ControlDevice ControlDevice_gencfg)

add_executable(RosOpenIGTLBridge src/RosOpenIGTLBridge.cpp include/masterslave/RosOpenIGTLBridge.h)
add_dependencies(RosOpenIGTLBridge RosOpenITGLBridge_gencfg)
add_dependencies(RosOpenIGTLBridge masterslave_gencfg)

add_executable(NumericKinematic src/kinematic/NumericKinematic.cpp include/masterslave/kinematic/NumericKinematic.h include/masterslave/kinematic/IKinematic.h src/kinematic/IKinematic.cpp)
add_dependencies(NumericKinematic masterslave_gencfg)
add_dependencies(NumericKinematic NumericKinematic_gencfg)

add_executable(GeometricKinematic src/kinematic/GeometricKinematic.cpp include/masterslave/kinematic/GeometricKinematic.h include/masterslave/kinematic/IKinematic.h src/kinematic/IKinematic.cpp)
add_dependencies(GeometricKinematic GeometricKinematic_gencfg)

add_executable(MasterSlaveManipulation src/manipulation/MasterSlaveManipulation.cpp include/masterslave/manipulation/MasterSlaveManipulation.h)
add_dependencies(MasterSlaveManipulation MasterSlaveManipulation_gencfg)
add_dependencies(MasterSlaveManipulation masterslave_gencfg)


add_executable(TrajectoryGenerator src/manipulation/TrajectoryGenerator.cpp include/masterslave/manipulation/TrajectoryGenerator.h include/masterslave/manipulation/trajectory/PTPTrajectory.h src/manipulation/trajectory/PTPTrajectory.cpp include/masterslave/manipulation/trajectory/CircleTrajectory.h src/manipulation/trajectory/CircleTrajectory.cpp include/masterslave/manipulation/trajectory/ITrajectory.h include/masterslave/manipulation/trajectory/LineTrajectory.h src/manipulation/trajectory/LineTrajectory.cpp)
add_dependencies(TrajectoryGenerator TrajectoryGenerator_gencfg)
add_dependencies(TrajectoryGenerator masterslave_gencfg)

add_executable(NumericKinematicCommander src/commander/NumericKinematicCommander.cpp include/masterslave/commander/NumericKinematicCommander.h src/commander/ICommander.cpp include/masterslave/commander/ICommander.h include/masterslave/commander/BoundingBox.h src/commander/BoundingBox.cpp)
add_dependencies(NumericKinematicCommander masterslave_gencfg)

add_executable(GeometricKinematicCommander src/commander/GeometricKinematicCommander.cpp include/masterslave/commander/GeometricKinematicCommander.h src/commander/ICommander.cpp include/masterslave/commander/ICommander.h include/masterslave/commander/BoundingBox.h src/commander/BoundingBox.cpp)
add_dependencies(GeometricKinematicCommander masterslave_gencfg)

add_executable(VisualServoing src/manipulation/VisualServoing.cpp include/masterslave/manipulation/VisualServoing.h)

## Add cmake target dependencies of the executable
add_dependencies(ControlDevice ControlDevice_generate_messages_cpp)
add_dependencies(ControlDevice masterslave_generate_messages_cpp)
add_dependencies(RosOpenIGTLBridge rosopenigtlbridge_generate_messages_cpp)
add_dependencies(RosOpenIGTLBridge masterslave_generate_messages_cpp)
add_dependencies(GeometricKinematic GeometricKinematic_generate_messages_cpp)
add_dependencies(GeometricKinematic masterslave_generate_messages_cpp)
add_dependencies(MasterSlaveManipulation MasterSlaveManipulation_generate_messages_cpp)
add_dependencies(MasterSlaveManipulation masterslave_generate_messages_cpp)
add_dependencies(NumericKinematic NumericKinematic_generate_messages_cpp)
add_dependencies(NumericKinematic masterslave_generate_messages_cpp)
add_dependencies(TrajectoryGenerator TrajectoryGenerator_generate_messages_cpp)
add_dependencies(TrajectoryGenerator masterslave_generate_messages_cpp)
add_dependencies(NumericKinematicCommander NumericKinematicCommander_generate_messages_cpp)
add_dependencies(NumericKinematicCommander masterslave_generate_messages_cpp)
add_dependencies(GeometricKinematicCommander GeometricKinematicCommander_generate_messages_cpp)
add_dependencies(GeometricKinematicCommander masterslave_generate_messages_cpp)
add_dependencies(VisualServoing masterslave_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(ControlDevice
   ${catkin_LIBRARIES}
   ${Eigen_LIBRARIES}
)

target_link_libraries(RosOpenIGTLBridge
    ${catkin_LIBRARIES}
    ${Eigen_LIBRARIES}
    OpenIGTLink
)

target_link_libraries(NumericKinematic
    ${catkin_LIBRARIES}
    ${Eigen_LIBRARIES}
)

target_link_libraries(GeometricKinematic
    ${catkin_LIBRARIES}
    ${Eigen_LIBRARIES}
)

target_link_libraries(MasterSlaveManipulation
    ${catkin_LIBRARIES}
    ${Eigen_LIBRARIES}
)

target_link_libraries(TrajectoryGenerator
    ${catkin_LIBRARIES}
    ${Eigen_LIBRARIES}
)

target_link_libraries(NumericKinematicCommander
    ${catkin_LIBRARIES}
    ${Eigen_LIBRARIES}
)

target_link_libraries(GeometricKinematicCommander
    ${catkin_LIBRARIES}
    ${Eigen_LIBRARIES}
)

target_link_libraries(VisualServoing
    ${catkin_LIBRARIES}
    ${Eigen_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS masterslave masterslave_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_masterslave.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
