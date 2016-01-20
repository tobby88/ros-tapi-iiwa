cmake_minimum_required(VERSION 2.8.3)
project(masterslave)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  eigen_conversions
  cmake_modules
  tf
  tf_conversions
  dynamic_reconfigure
)

set(OpenIGTLink_DIR "/usr/local/lib/igtl/")
find_package(OpenIGTLink REQUIRED)
include(${OpenIGTLink_USE_FILE})



find_package(Eigen REQUIRED)
## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)




################################################
## Declare ROS messages, services and actions ##
################################################


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
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

generate_dynamic_reconfigure_options(
  cfg/masterslave.cfg
  cfg/controldevice.cfg
  cfg/rosopenigtlbridge.cfg
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
)


## Declare a C++ executable
add_executable(controldevice src/controldevice.cpp include/masterslave/controldevice.h)
add_dependencies(controldevice controldevice_gencfg)

add_executable(masterslave src/masterslave.cpp include/masterslave/masterslave.h src/kinematic/laparoscope.cpp include/masterslave/kinematic/laparoscope.h include/masterslave/kinematic/kinematics.h src/kinematic/kinematics.cpp include/masterslave/kinematic/ursulakinematics.h src/kinematic/ursulakinematics.cpp src/task/task.cpp include/masterslave/task/task.h src/task/laparoscopetask.cpp include/masterslave/task/laparoscopetask.h)
add_dependencies(masterslave $(PROJECT_NAME)_gencfg)

add_executable(rosopenigtlbridge src/rosopenigtlbridge.cpp include/masterslave/rosopenigtlbridge.h)
add_dependencies(rosopenigtlbridge rosopenigtlbridge_gencfg)


## Add cmake target dependencies of the executable

add_dependencies(masterslave masterslave_generate_messages_cpp)
add_dependencies(controldevice controldevice_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(controldevice
   ${catkin_LIBRARIES}
   ${Eigen_LIBRARIES}
)

target_link_libraries(masterslave
    ${catkin_LIBRARIES}
    ${Eigen_LIBRARIES}
)

target_link_libraries(rosopenigtlbridge
    ${catkin_LIBRARIES}
    ${Eigen_LIBRARIES}
    OpenIGTLink
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
