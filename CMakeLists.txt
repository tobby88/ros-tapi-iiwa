cmake_minimum_required(VERSION 2.8.3)
project(tapi_iiwa)

# Use specified C++ standard
set(CMAKE_CXX_STANDARD 11)
# ...and for older compilers
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  geometry_msgs
  message_generation
  roscpp
  sensor_msgs
  std_msgs
  tapi_lib
  tf
  tf_conversions
)

include_directories(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)

set(OpenIGTLink_DIR "/usr/local/lib/igtl/")
find_package(OpenIGTLink REQUIRED)
include(${OpenIGTLink_USE_FILE})

find_package(Eigen3 REQUIRED)
## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

# Qt
# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

################################################
## Declare ROS messages, services and actions ##
################################################
add_service_files(DIRECTORY srv
  FILES
  OpenIGTLStateService.srv
)


## Generate messages in the 'msg' folder
#add_message_files(DIRECTORY msg
#   FILES
#   Button.msg
#   Message2.msg
#)

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

#generate_dynamic_reconfigure_options(
#  cfg/Tapi_iiwa.cfg
#)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES masterslave
  CATKIN_DEPENDS eigen_conversions geometry_msgs message_runtime roscpp sensor_msgs std_msgs tapi_lib tf tf_conversions
  DEPENDS Eigen3
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
)

## Declare a C++ library
# add_library(tapi_iiwa
#   src/${PROJECT_NAME}/tapi_iiwa.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(tapi_iiwa ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(tapi_iiwa
  src/main.cpp
  src/tapi_iiwa.cpp
)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(tapi_iiwa ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(tapi_iiwa
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
