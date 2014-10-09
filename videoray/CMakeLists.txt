cmake_minimum_required(VERSION 2.8.3)
project(videoray)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
    auv_msgs 
    roscpp
    dynamic_reconfigure 
    diagnostic_msgs
    snippets)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system serialization)
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

## Uncomment this if the package has a setup.py. This macro ensures
## modules and scripts declared therein get installed
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   auv_msgs
# )

###################################################
## Dynamic reconfigure
###################################################
#generate_dynamic_reconfigure_options(cfg/ThrusterMapping.cfg cfg/DPSettings.cfg)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#   INCLUDE_DIRS include
#  LIBRARIES pladypos
   CATKIN_DEPENDS auv_msgs message_runtime roscpp dynamic_reconfigure
#  DEPENDS system_lib
)

###########
## Build ##
###########
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a cpp library
# add_library(pladypos
#   src/${PROJECT_NAME}/pladypos.cpp
# )

## Declare a cpp executable

add_executable(videoray_node src/videoray_node.cpp)
add_executable(bmotor_node src/brushless/BMotor.cpp)
add_executable(bmotor_node2 src/brushless/BMotor2.cpp)

## Add dependencies to the executable
#add_dependencies(pladypos_node ${PROJECT_NAME}_gencpp)
if (DEFINED catkin_EXPORTED_TARGETS)
 add_dependencies(videoray_node ${catkin_EXPORTED_TARGETS})
endif()

## Specify libraries to link a library or executable target against

#target_link_libraries(pladypos_node ${catkin_LIBRARIES} ${Boost_SYSTEM_LIBRARY})
#target_link_libraries(pladypos_node_v2 ${catkin_LIBRARIES})
target_link_libraries(videoray_node ${catkin_LIBRARIES})
target_link_libraries(bmotor_node ${catkin_LIBRARIES} ${Boost_SERIALIZATION_LIBRARY})
target_link_libraries(bmotor_node2 ${catkin_LIBRARIES} ${Boost_SERIALIZATION_LIBRARY})

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## not required for python when using catkin_python_setup()
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS pladypos pladypos_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_pladypos.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
