cmake_minimum_required(VERSION 2.8.3)
project(deep_learning_crazyflie)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -DMAVLINK_DIALECT=common")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  sensor_msgs
  cmake_modules
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
  message_filters
  image_transport
  cv_bridge
  message_filters
)

find_package(OpenCV REQUIRED)
find_package(Eigen REQUIRED)

add_subdirectory( ../klt_feature_tracker klt_feature_tracker)
include_directories( ../klt_feature_tracker/include)
set(klt_feature_tracker_LIBS "klt_feature_tracker")

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder

add_service_files(
	DIRECTORY srv
	FILES TunePID.srv Status.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   sensor_msgs
   std_msgs
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES deep_learning_crazyflie
 CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs message_runtime image_transport
 DEPENDS system_lib Eigen
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(deep_learning_crazyflie
#   src/${PROJECT_NAME}/deep_learning_crazyflie.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(deep_learning_crazyflie ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
 add_executable(deep_learning_crazyflie_node src/ground_speed.cpp)
 add_executable(crazyflie_teleop_node src/keyboard_controller_crazyflie.cpp)
 add_executable(crazyflie_camera_node src/CameraPublisher.cpp)
 add_executable(crazyflie_opticalflow_node src/CalcOpticalFlow.cpp)
 add_executable(crazyflie_imu_node src/ImuPublisher.cpp)
 add_executable(crazyflie_position_control_node src/position_controller_crazyflie.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(deep_learning_crazyflie_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(crazyflie_teleop_node deep_learning_crazyflie_generate_messages_cpp)
add_dependencies(deep_learning_crazyflie_node deep_learning_crazyflie_generate_messages_cpp)
add_dependencies(crazyflie_camera_node ${PROJECT_NAME}_EXPORTED_TARGETS ${catkin_EXPORTED_TARGETS})
add_dependencies(crazyflie_opticalflow_node ${PROJECT_NAME}_EXPORTED_TARGETS ${catkin_EXPORTED_TARGETS} ${Eigen_LIBRARIES} )
add_dependencies(crazyflie_imu_node ${PROJECT_NAME}_EXPORTED_TARGETS ${catkin_EXPORTED_TARGETS} ${Eigen_LIBRARIES} )
add_dependencies(crazyflie_position_control_node deep_learning_crazflie_gencpp ${PROJECT_NAME}_EXPORTED_TARGETS ${catkin_EXPORTED_TARGETS} )
## Specify libraries to link a library or executable target against
# target_link_libraries(deep_learning_crazyflie_node
#   ${catkin_LIBRARIES}
# )
target_link_libraries(deep_learning_crazyflie_node ${catkin_LIBRARIES}  ${klt_feature_tracker_LIBS} )
target_link_libraries(crazyflie_teleop_node  ${catkin_LIBRARIES})
target_link_libraries(crazyflie_camera_node  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} )
target_link_libraries(crazyflie_opticalflow_node  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${klt_feature_tracker_LIBS} )
target_link_libraries(crazyflie_imu_node  ${catkin_LIBRARIES})
target_link_libraries(crazyflie_position_control_node  ${catkin_LIBRARIES})

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
# install(TARGETS deep_learning_crazyflie deep_learning_crazyflie_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_deep_learning_crazyflie.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
