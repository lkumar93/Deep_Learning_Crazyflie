# Install script for directory: /home/avinash/Deep_Learning_Crazyflie/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/avinash/Deep_Learning_Crazyflie/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/avinash/Deep_Learning_Crazyflie/install/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/avinash/Deep_Learning_Crazyflie/install" TYPE PROGRAM FILES "/home/avinash/Deep_Learning_Crazyflie/build/catkin_generated/installspace/_setup_util.py")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/avinash/Deep_Learning_Crazyflie/install/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/avinash/Deep_Learning_Crazyflie/install" TYPE PROGRAM FILES "/home/avinash/Deep_Learning_Crazyflie/build/catkin_generated/installspace/env.sh")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/avinash/Deep_Learning_Crazyflie/install/setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/avinash/Deep_Learning_Crazyflie/install" TYPE FILE FILES "/home/avinash/Deep_Learning_Crazyflie/build/catkin_generated/installspace/setup.bash")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/avinash/Deep_Learning_Crazyflie/install/setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/avinash/Deep_Learning_Crazyflie/install" TYPE FILE FILES "/home/avinash/Deep_Learning_Crazyflie/build/catkin_generated/installspace/setup.sh")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/avinash/Deep_Learning_Crazyflie/install/setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/avinash/Deep_Learning_Crazyflie/install" TYPE FILE FILES "/home/avinash/Deep_Learning_Crazyflie/build/catkin_generated/installspace/setup.zsh")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/avinash/Deep_Learning_Crazyflie/install/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/avinash/Deep_Learning_Crazyflie/install" TYPE FILE FILES "/home/avinash/Deep_Learning_Crazyflie/build/catkin_generated/installspace/.rosinstall")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/avinash/Deep_Learning_Crazyflie/build/gtest/cmake_install.cmake")
  include("/home/avinash/Deep_Learning_Crazyflie/build/crazyflie_ros_pkg/crazyflie/cmake_install.cmake")
  include("/home/avinash/Deep_Learning_Crazyflie/build/crazyflie_ros_pkg/crazyflie_controller/cmake_install.cmake")
  include("/home/avinash/Deep_Learning_Crazyflie/build/crazyflie_ros_pkg/crazyflie_cpp/cmake_install.cmake")
  include("/home/avinash/Deep_Learning_Crazyflie/build/crazyflie_ros_pkg/crazyflie_demo/cmake_install.cmake")
  include("/home/avinash/Deep_Learning_Crazyflie/build/crazyflie_ros_pkg/crazyflie_description/cmake_install.cmake")
  include("/home/avinash/Deep_Learning_Crazyflie/build/crazyflie_ros_pkg/crazyflie_tools/cmake_install.cmake")
  include("/home/avinash/Deep_Learning_Crazyflie/build/hector_quadrotor/hector_quadrotor/cmake_install.cmake")
  include("/home/avinash/Deep_Learning_Crazyflie/build/hector_quadrotor/hector_quadrotor_demo/cmake_install.cmake")
  include("/home/avinash/Deep_Learning_Crazyflie/build/hector_quadrotor/hector_quadrotor_description/cmake_install.cmake")
  include("/home/avinash/Deep_Learning_Crazyflie/build/vision_opencv/opencv_tests/cmake_install.cmake")
  include("/home/avinash/Deep_Learning_Crazyflie/build/hector_quadrotor/hector_uav_msgs/cmake_install.cmake")
  include("/home/avinash/Deep_Learning_Crazyflie/build/vision_opencv/image_geometry/cmake_install.cmake")
  include("/home/avinash/Deep_Learning_Crazyflie/build/vision_opencv/vision_opencv/cmake_install.cmake")
  include("/home/avinash/Deep_Learning_Crazyflie/build/hector_quadrotor/hector_quadrotor_controller/cmake_install.cmake")
  include("/home/avinash/Deep_Learning_Crazyflie/build/hector_quadrotor/hector_quadrotor_model/cmake_install.cmake")
  include("/home/avinash/Deep_Learning_Crazyflie/build/hector_quadrotor/hector_quadrotor_teleop/cmake_install.cmake")
  include("/home/avinash/Deep_Learning_Crazyflie/build/vision_opencv/cv_bridge/cmake_install.cmake")
  include("/home/avinash/Deep_Learning_Crazyflie/build/deep_learning_crazyflie/cmake_install.cmake")
  include("/home/avinash/Deep_Learning_Crazyflie/build/vision_opencv/opencv_apps/cmake_install.cmake")
  include("/home/avinash/Deep_Learning_Crazyflie/build/crazyflie_ros_pkg/crazyflie_driver/cmake_install.cmake")
  include("/home/avinash/Deep_Learning_Crazyflie/build/hector_quadrotor/hector_quadrotor_gazebo_plugins/cmake_install.cmake")
  include("/home/avinash/Deep_Learning_Crazyflie/build/hector_quadrotor/hector_quadrotor_pose_estimation/cmake_install.cmake")
  include("/home/avinash/Deep_Learning_Crazyflie/build/hector_quadrotor/hector_quadrotor_controller_gazebo/cmake_install.cmake")
  include("/home/avinash/Deep_Learning_Crazyflie/build/hector_quadrotor/hector_quadrotor_gazebo/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/avinash/Deep_Learning_Crazyflie/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
