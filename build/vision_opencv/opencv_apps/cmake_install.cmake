# Install script for directory: /home/avinash/Deep_Learning_Crazyflie/src/vision_opencv/opencv_apps

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/opencv_apps" TYPE FILE FILES "/home/avinash/Deep_Learning_Crazyflie/devel/include/opencv_apps/EdgeDetectionConfig.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/opencv_apps" TYPE FILE FILES "/home/avinash/Deep_Learning_Crazyflie/devel/include/opencv_apps/HoughLinesConfig.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/opencv_apps" TYPE FILE FILES "/home/avinash/Deep_Learning_Crazyflie/devel/include/opencv_apps/HoughCirclesConfig.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/opencv_apps" TYPE FILE FILES "/home/avinash/Deep_Learning_Crazyflie/devel/include/opencv_apps/FindContoursConfig.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/opencv_apps" TYPE FILE FILES "/home/avinash/Deep_Learning_Crazyflie/devel/include/opencv_apps/ConvexHullConfig.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/opencv_apps" TYPE FILE FILES "/home/avinash/Deep_Learning_Crazyflie/devel/include/opencv_apps/GeneralContoursConfig.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/opencv_apps" TYPE FILE FILES "/home/avinash/Deep_Learning_Crazyflie/devel/include/opencv_apps/ContourMomentsConfig.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/opencv_apps" TYPE FILE FILES "/home/avinash/Deep_Learning_Crazyflie/devel/include/opencv_apps/FaceDetectionConfig.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/opencv_apps" TYPE FILE FILES "/home/avinash/Deep_Learning_Crazyflie/devel/include/opencv_apps/GoodfeatureTrackConfig.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/opencv_apps" TYPE FILE FILES "/home/avinash/Deep_Learning_Crazyflie/devel/include/opencv_apps/CamShiftConfig.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/opencv_apps" TYPE FILE FILES "/home/avinash/Deep_Learning_Crazyflie/devel/include/opencv_apps/FBackFlowConfig.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/opencv_apps" TYPE FILE FILES "/home/avinash/Deep_Learning_Crazyflie/devel/include/opencv_apps/LKFlowConfig.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/opencv_apps" TYPE FILE FILES "/home/avinash/Deep_Learning_Crazyflie/devel/include/opencv_apps/PeopleDetectConfig.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/opencv_apps" TYPE FILE FILES "/home/avinash/Deep_Learning_Crazyflie/devel/include/opencv_apps/PhaseCorrConfig.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/opencv_apps" TYPE FILE FILES "/home/avinash/Deep_Learning_Crazyflie/devel/include/opencv_apps/SegmentObjectsConfig.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/opencv_apps" TYPE FILE FILES "/home/avinash/Deep_Learning_Crazyflie/devel/include/opencv_apps/SimpleFlowConfig.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/opencv_apps" TYPE FILE FILES "/home/avinash/Deep_Learning_Crazyflie/devel/include/opencv_apps/WatershedSegmentationConfig.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages/opencv_apps" TYPE FILE FILES "/home/avinash/Deep_Learning_Crazyflie/devel/lib/python2.7/dist-packages/opencv_apps/__init__.py")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/avinash/Deep_Learning_Crazyflie/devel/lib/python2.7/dist-packages/opencv_apps/cfg")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages/opencv_apps" TYPE DIRECTORY FILES "/home/avinash/Deep_Learning_Crazyflie/devel/lib/python2.7/dist-packages/opencv_apps/cfg")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/opencv_apps/msg" TYPE FILE FILES
    "/home/avinash/Deep_Learning_Crazyflie/src/vision_opencv/opencv_apps/msg/Point2D.msg"
    "/home/avinash/Deep_Learning_Crazyflie/src/vision_opencv/opencv_apps/msg/Point2DStamped.msg"
    "/home/avinash/Deep_Learning_Crazyflie/src/vision_opencv/opencv_apps/msg/Point2DArray.msg"
    "/home/avinash/Deep_Learning_Crazyflie/src/vision_opencv/opencv_apps/msg/Point2DArrayStamped.msg"
    "/home/avinash/Deep_Learning_Crazyflie/src/vision_opencv/opencv_apps/msg/Rect.msg"
    "/home/avinash/Deep_Learning_Crazyflie/src/vision_opencv/opencv_apps/msg/RectArray.msg"
    "/home/avinash/Deep_Learning_Crazyflie/src/vision_opencv/opencv_apps/msg/RectArrayStamped.msg"
    "/home/avinash/Deep_Learning_Crazyflie/src/vision_opencv/opencv_apps/msg/Flow.msg"
    "/home/avinash/Deep_Learning_Crazyflie/src/vision_opencv/opencv_apps/msg/FlowStamped.msg"
    "/home/avinash/Deep_Learning_Crazyflie/src/vision_opencv/opencv_apps/msg/FlowArray.msg"
    "/home/avinash/Deep_Learning_Crazyflie/src/vision_opencv/opencv_apps/msg/FlowArrayStamped.msg"
    "/home/avinash/Deep_Learning_Crazyflie/src/vision_opencv/opencv_apps/msg/RectArrayStamped.msg"
    "/home/avinash/Deep_Learning_Crazyflie/src/vision_opencv/opencv_apps/msg/Size.msg"
    "/home/avinash/Deep_Learning_Crazyflie/src/vision_opencv/opencv_apps/msg/Face.msg"
    "/home/avinash/Deep_Learning_Crazyflie/src/vision_opencv/opencv_apps/msg/FaceArray.msg"
    "/home/avinash/Deep_Learning_Crazyflie/src/vision_opencv/opencv_apps/msg/FaceArrayStamped.msg"
    "/home/avinash/Deep_Learning_Crazyflie/src/vision_opencv/opencv_apps/msg/Line.msg"
    "/home/avinash/Deep_Learning_Crazyflie/src/vision_opencv/opencv_apps/msg/LineArray.msg"
    "/home/avinash/Deep_Learning_Crazyflie/src/vision_opencv/opencv_apps/msg/LineArrayStamped.msg"
    "/home/avinash/Deep_Learning_Crazyflie/src/vision_opencv/opencv_apps/msg/RotatedRect.msg"
    "/home/avinash/Deep_Learning_Crazyflie/src/vision_opencv/opencv_apps/msg/RotatedRectStamped.msg"
    "/home/avinash/Deep_Learning_Crazyflie/src/vision_opencv/opencv_apps/msg/RotatedRectArray.msg"
    "/home/avinash/Deep_Learning_Crazyflie/src/vision_opencv/opencv_apps/msg/RotatedRectArrayStamped.msg"
    "/home/avinash/Deep_Learning_Crazyflie/src/vision_opencv/opencv_apps/msg/Circle.msg"
    "/home/avinash/Deep_Learning_Crazyflie/src/vision_opencv/opencv_apps/msg/CircleArray.msg"
    "/home/avinash/Deep_Learning_Crazyflie/src/vision_opencv/opencv_apps/msg/CircleArrayStamped.msg"
    "/home/avinash/Deep_Learning_Crazyflie/src/vision_opencv/opencv_apps/msg/Moment.msg"
    "/home/avinash/Deep_Learning_Crazyflie/src/vision_opencv/opencv_apps/msg/MomentArray.msg"
    "/home/avinash/Deep_Learning_Crazyflie/src/vision_opencv/opencv_apps/msg/MomentArrayStamped.msg"
    "/home/avinash/Deep_Learning_Crazyflie/src/vision_opencv/opencv_apps/msg/Contour.msg"
    "/home/avinash/Deep_Learning_Crazyflie/src/vision_opencv/opencv_apps/msg/ContourArray.msg"
    "/home/avinash/Deep_Learning_Crazyflie/src/vision_opencv/opencv_apps/msg/ContourArrayStamped.msg"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/opencv_apps/cmake" TYPE FILE FILES "/home/avinash/Deep_Learning_Crazyflie/build/vision_opencv/opencv_apps/catkin_generated/installspace/opencv_apps-msg-paths.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/avinash/Deep_Learning_Crazyflie/devel/include/opencv_apps")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/avinash/Deep_Learning_Crazyflie/devel/share/common-lisp/ros/opencv_apps")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/avinash/Deep_Learning_Crazyflie/devel/lib/python2.7/dist-packages/opencv_apps")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/avinash/Deep_Learning_Crazyflie/devel/lib/python2.7/dist-packages/opencv_apps")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/avinash/Deep_Learning_Crazyflie/build/vision_opencv/opencv_apps/catkin_generated/installspace/opencv_apps.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/opencv_apps/cmake" TYPE FILE FILES "/home/avinash/Deep_Learning_Crazyflie/build/vision_opencv/opencv_apps/catkin_generated/installspace/opencv_apps-msg-extras.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/opencv_apps/cmake" TYPE FILE FILES
    "/home/avinash/Deep_Learning_Crazyflie/build/vision_opencv/opencv_apps/catkin_generated/installspace/opencv_appsConfig.cmake"
    "/home/avinash/Deep_Learning_Crazyflie/build/vision_opencv/opencv_apps/catkin_generated/installspace/opencv_appsConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/opencv_apps" TYPE FILE FILES "/home/avinash/Deep_Learning_Crazyflie/src/vision_opencv/opencv_apps/package.xml")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libopencv_apps.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libopencv_apps.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libopencv_apps.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/avinash/Deep_Learning_Crazyflie/devel/lib/libopencv_apps.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libopencv_apps.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libopencv_apps.so")
    file(RPATH_REMOVE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libopencv_apps.so")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libopencv_apps.so")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/opencv_apps" TYPE DIRECTORY FILES "/home/avinash/Deep_Learning_Crazyflie/src/vision_opencv/opencv_apps/include/opencv_apps/")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/edge_detection" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/edge_detection")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/edge_detection"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/opencv_apps" TYPE EXECUTABLE FILES "/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/edge_detection")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/edge_detection" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/edge_detection")
    file(RPATH_REMOVE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/edge_detection")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/edge_detection")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/hough_lines" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/hough_lines")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/hough_lines"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/opencv_apps" TYPE EXECUTABLE FILES "/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/hough_lines")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/hough_lines" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/hough_lines")
    file(RPATH_REMOVE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/hough_lines")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/hough_lines")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/hough_circles" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/hough_circles")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/hough_circles"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/opencv_apps" TYPE EXECUTABLE FILES "/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/hough_circles")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/hough_circles" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/hough_circles")
    file(RPATH_REMOVE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/hough_circles")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/hough_circles")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/face_detection" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/face_detection")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/face_detection"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/opencv_apps" TYPE EXECUTABLE FILES "/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/face_detection" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/face_detection")
    file(RPATH_REMOVE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/face_detection")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/face_detection")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/goodfeature_track" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/goodfeature_track")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/goodfeature_track"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/opencv_apps" TYPE EXECUTABLE FILES "/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/goodfeature_track")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/goodfeature_track" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/goodfeature_track")
    file(RPATH_REMOVE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/goodfeature_track")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/goodfeature_track")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/camshift" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/camshift")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/camshift"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/opencv_apps" TYPE EXECUTABLE FILES "/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/camshift")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/camshift" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/camshift")
    file(RPATH_REMOVE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/camshift")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/camshift")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/simple_example" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/simple_example")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/simple_example"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/opencv_apps" TYPE EXECUTABLE FILES "/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/simple_example")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/simple_example" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/simple_example")
    file(RPATH_REMOVE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/simple_example")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/simple_example")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/simple_compressed_example" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/simple_compressed_example")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/simple_compressed_example"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/opencv_apps" TYPE EXECUTABLE FILES "/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/simple_compressed_example")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/simple_compressed_example" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/simple_compressed_example")
    file(RPATH_REMOVE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/simple_compressed_example")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/simple_compressed_example")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/fback_flow" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/fback_flow")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/fback_flow"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/opencv_apps" TYPE EXECUTABLE FILES "/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/fback_flow")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/fback_flow" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/fback_flow")
    file(RPATH_REMOVE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/fback_flow")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/fback_flow")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/lk_flow" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/lk_flow")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/lk_flow"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/opencv_apps" TYPE EXECUTABLE FILES "/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/lk_flow")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/lk_flow" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/lk_flow")
    file(RPATH_REMOVE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/lk_flow")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/lk_flow")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/people_detect" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/people_detect")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/people_detect"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/opencv_apps" TYPE EXECUTABLE FILES "/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/people_detect")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/people_detect" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/people_detect")
    file(RPATH_REMOVE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/people_detect")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/people_detect")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/phase_corr" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/phase_corr")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/phase_corr"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/opencv_apps" TYPE EXECUTABLE FILES "/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/phase_corr")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/phase_corr" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/phase_corr")
    file(RPATH_REMOVE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/phase_corr")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/phase_corr")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/segment_objects" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/segment_objects")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/segment_objects"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/opencv_apps" TYPE EXECUTABLE FILES "/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/segment_objects")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/segment_objects" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/segment_objects")
    file(RPATH_REMOVE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/segment_objects")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/segment_objects")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/watershed_segmentation" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/watershed_segmentation")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/watershed_segmentation"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/opencv_apps" TYPE EXECUTABLE FILES "/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/watershed_segmentation")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/watershed_segmentation" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/watershed_segmentation")
    file(RPATH_REMOVE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/watershed_segmentation")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/opencv_apps/watershed_segmentation")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/opencv_apps" TYPE FILE FILES "/home/avinash/Deep_Learning_Crazyflie/src/vision_opencv/opencv_apps/nodelet_plugins.xml")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/avinash/Deep_Learning_Crazyflie/build/vision_opencv/opencv_apps/test/cmake_install.cmake")

endif()

