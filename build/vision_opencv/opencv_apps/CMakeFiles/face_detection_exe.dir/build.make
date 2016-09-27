# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.2

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

# Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/avinash/Deep_Learning_Crazyflie/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/avinash/Deep_Learning_Crazyflie/build

# Include any dependencies generated for this target.
include vision_opencv/opencv_apps/CMakeFiles/face_detection_exe.dir/depend.make

# Include the progress variables for this target.
include vision_opencv/opencv_apps/CMakeFiles/face_detection_exe.dir/progress.make

# Include the compile flags for this target's objects.
include vision_opencv/opencv_apps/CMakeFiles/face_detection_exe.dir/flags.make

vision_opencv/opencv_apps/CMakeFiles/face_detection_exe.dir/src/node/face_detection.cpp.o: vision_opencv/opencv_apps/CMakeFiles/face_detection_exe.dir/flags.make
vision_opencv/opencv_apps/CMakeFiles/face_detection_exe.dir/src/node/face_detection.cpp.o: /home/avinash/Deep_Learning_Crazyflie/src/vision_opencv/opencv_apps/src/node/face_detection.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/avinash/Deep_Learning_Crazyflie/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object vision_opencv/opencv_apps/CMakeFiles/face_detection_exe.dir/src/node/face_detection.cpp.o"
	cd /home/avinash/Deep_Learning_Crazyflie/build/vision_opencv/opencv_apps && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/face_detection_exe.dir/src/node/face_detection.cpp.o -c /home/avinash/Deep_Learning_Crazyflie/src/vision_opencv/opencv_apps/src/node/face_detection.cpp

vision_opencv/opencv_apps/CMakeFiles/face_detection_exe.dir/src/node/face_detection.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/face_detection_exe.dir/src/node/face_detection.cpp.i"
	cd /home/avinash/Deep_Learning_Crazyflie/build/vision_opencv/opencv_apps && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/avinash/Deep_Learning_Crazyflie/src/vision_opencv/opencv_apps/src/node/face_detection.cpp > CMakeFiles/face_detection_exe.dir/src/node/face_detection.cpp.i

vision_opencv/opencv_apps/CMakeFiles/face_detection_exe.dir/src/node/face_detection.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/face_detection_exe.dir/src/node/face_detection.cpp.s"
	cd /home/avinash/Deep_Learning_Crazyflie/build/vision_opencv/opencv_apps && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/avinash/Deep_Learning_Crazyflie/src/vision_opencv/opencv_apps/src/node/face_detection.cpp -o CMakeFiles/face_detection_exe.dir/src/node/face_detection.cpp.s

vision_opencv/opencv_apps/CMakeFiles/face_detection_exe.dir/src/node/face_detection.cpp.o.requires:
.PHONY : vision_opencv/opencv_apps/CMakeFiles/face_detection_exe.dir/src/node/face_detection.cpp.o.requires

vision_opencv/opencv_apps/CMakeFiles/face_detection_exe.dir/src/node/face_detection.cpp.o.provides: vision_opencv/opencv_apps/CMakeFiles/face_detection_exe.dir/src/node/face_detection.cpp.o.requires
	$(MAKE) -f vision_opencv/opencv_apps/CMakeFiles/face_detection_exe.dir/build.make vision_opencv/opencv_apps/CMakeFiles/face_detection_exe.dir/src/node/face_detection.cpp.o.provides.build
.PHONY : vision_opencv/opencv_apps/CMakeFiles/face_detection_exe.dir/src/node/face_detection.cpp.o.provides

vision_opencv/opencv_apps/CMakeFiles/face_detection_exe.dir/src/node/face_detection.cpp.o.provides.build: vision_opencv/opencv_apps/CMakeFiles/face_detection_exe.dir/src/node/face_detection.cpp.o

# Object files for target face_detection_exe
face_detection_exe_OBJECTS = \
"CMakeFiles/face_detection_exe.dir/src/node/face_detection.cpp.o"

# External object files for target face_detection_exe
face_detection_exe_EXTERNAL_OBJECTS =

/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection: vision_opencv/opencv_apps/CMakeFiles/face_detection_exe.dir/src/node/face_detection.cpp.o
/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection: vision_opencv/opencv_apps/CMakeFiles/face_detection_exe.dir/build.make
/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection: /home/avinash/Deep_Learning_Crazyflie/devel/lib/libcv_bridge.so
/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection: /usr/lib/i386-linux-gnu/libopencv_videostab.so.2.4.8
/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection: /usr/lib/i386-linux-gnu/libopencv_video.so.2.4.8
/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection: /usr/lib/i386-linux-gnu/libopencv_superres.so.2.4.8
/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection: /usr/lib/i386-linux-gnu/libopencv_stitching.so.2.4.8
/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection: /usr/lib/i386-linux-gnu/libopencv_photo.so.2.4.8
/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection: /usr/lib/i386-linux-gnu/libopencv_ocl.so.2.4.8
/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection: /usr/lib/i386-linux-gnu/libopencv_objdetect.so.2.4.8
/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection: /usr/lib/i386-linux-gnu/libopencv_ml.so.2.4.8
/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection: /usr/lib/i386-linux-gnu/libopencv_legacy.so.2.4.8
/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection: /usr/lib/i386-linux-gnu/libopencv_imgproc.so.2.4.8
/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection: /usr/lib/i386-linux-gnu/libopencv_highgui.so.2.4.8
/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection: /usr/lib/i386-linux-gnu/libopencv_gpu.so.2.4.8
/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection: /usr/lib/i386-linux-gnu/libopencv_flann.so.2.4.8
/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection: /usr/lib/i386-linux-gnu/libopencv_features2d.so.2.4.8
/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection: /usr/lib/i386-linux-gnu/libopencv_core.so.2.4.8
/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection: /usr/lib/i386-linux-gnu/libopencv_contrib.so.2.4.8
/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection: /usr/lib/i386-linux-gnu/libopencv_calib3d.so.2.4.8
/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection: /opt/ros/indigo/lib/libdynamic_reconfigure_config_init_mutex.so
/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection: /opt/ros/indigo/lib/libimage_transport.so
/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection: /opt/ros/indigo/lib/libmessage_filters.so
/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection: /opt/ros/indigo/lib/libnodeletlib.so
/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection: /opt/ros/indigo/lib/libbondcpp.so
/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection: /usr/lib/i386-linux-gnu/libuuid.so
/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection: /usr/lib/i386-linux-gnu/libtinyxml.so
/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection: /opt/ros/indigo/lib/libclass_loader.so
/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection: /usr/lib/libPocoFoundation.so
/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection: /usr/lib/i386-linux-gnu/libdl.so
/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection: /opt/ros/indigo/lib/libroslib.so
/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection: /opt/ros/indigo/lib/libroscpp.so
/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection: /usr/lib/i386-linux-gnu/libboost_signals.so
/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection: /usr/lib/i386-linux-gnu/libboost_filesystem.so
/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection: /opt/ros/indigo/lib/librosconsole.so
/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection: /opt/ros/indigo/lib/librosconsole_log4cxx.so
/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection: /opt/ros/indigo/lib/librosconsole_backend_interface.so
/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection: /usr/lib/liblog4cxx.so
/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection: /usr/lib/i386-linux-gnu/libboost_regex.so
/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection: /opt/ros/indigo/lib/libroscpp_serialization.so
/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection: /opt/ros/indigo/lib/librostime.so
/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection: /usr/lib/i386-linux-gnu/libboost_date_time.so
/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection: /opt/ros/indigo/lib/libxmlrpcpp.so
/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection: /opt/ros/indigo/lib/libcpp_common.so
/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection: /usr/lib/i386-linux-gnu/libboost_system.so
/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection: /usr/lib/i386-linux-gnu/libboost_thread.so
/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection: /usr/lib/i386-linux-gnu/libpthread.so
/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection: /usr/lib/i386-linux-gnu/libconsole_bridge.so
/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection: /usr/lib/i386-linux-gnu/libopencv_videostab.so.2.4.8
/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection: /usr/lib/i386-linux-gnu/libopencv_superres.so.2.4.8
/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection: /usr/lib/i386-linux-gnu/libopencv_stitching.so.2.4.8
/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection: /usr/lib/i386-linux-gnu/libopencv_ocl.so.2.4.8
/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection: /usr/lib/i386-linux-gnu/libopencv_gpu.so.2.4.8
/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection: /usr/lib/i386-linux-gnu/libopencv_contrib.so.2.4.8
/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection: /usr/lib/i386-linux-gnu/libopencv_photo.so.2.4.8
/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection: /usr/lib/i386-linux-gnu/libopencv_legacy.so.2.4.8
/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection: /usr/lib/i386-linux-gnu/libopencv_video.so.2.4.8
/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection: /usr/lib/i386-linux-gnu/libopencv_objdetect.so.2.4.8
/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection: /usr/lib/i386-linux-gnu/libopencv_ml.so.2.4.8
/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection: /usr/lib/i386-linux-gnu/libopencv_calib3d.so.2.4.8
/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection: /usr/lib/i386-linux-gnu/libopencv_features2d.so.2.4.8
/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection: /usr/lib/i386-linux-gnu/libopencv_highgui.so.2.4.8
/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection: /usr/lib/i386-linux-gnu/libopencv_imgproc.so.2.4.8
/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection: /usr/lib/i386-linux-gnu/libopencv_flann.so.2.4.8
/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection: /usr/lib/i386-linux-gnu/libopencv_core.so.2.4.8
/home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection: vision_opencv/opencv_apps/CMakeFiles/face_detection_exe.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable /home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection"
	cd /home/avinash/Deep_Learning_Crazyflie/build/vision_opencv/opencv_apps && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/face_detection_exe.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
vision_opencv/opencv_apps/CMakeFiles/face_detection_exe.dir/build: /home/avinash/Deep_Learning_Crazyflie/devel/lib/opencv_apps/face_detection
.PHONY : vision_opencv/opencv_apps/CMakeFiles/face_detection_exe.dir/build

vision_opencv/opencv_apps/CMakeFiles/face_detection_exe.dir/requires: vision_opencv/opencv_apps/CMakeFiles/face_detection_exe.dir/src/node/face_detection.cpp.o.requires
.PHONY : vision_opencv/opencv_apps/CMakeFiles/face_detection_exe.dir/requires

vision_opencv/opencv_apps/CMakeFiles/face_detection_exe.dir/clean:
	cd /home/avinash/Deep_Learning_Crazyflie/build/vision_opencv/opencv_apps && $(CMAKE_COMMAND) -P CMakeFiles/face_detection_exe.dir/cmake_clean.cmake
.PHONY : vision_opencv/opencv_apps/CMakeFiles/face_detection_exe.dir/clean

vision_opencv/opencv_apps/CMakeFiles/face_detection_exe.dir/depend:
	cd /home/avinash/Deep_Learning_Crazyflie/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/avinash/Deep_Learning_Crazyflie/src /home/avinash/Deep_Learning_Crazyflie/src/vision_opencv/opencv_apps /home/avinash/Deep_Learning_Crazyflie/build /home/avinash/Deep_Learning_Crazyflie/build/vision_opencv/opencv_apps /home/avinash/Deep_Learning_Crazyflie/build/vision_opencv/opencv_apps/CMakeFiles/face_detection_exe.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : vision_opencv/opencv_apps/CMakeFiles/face_detection_exe.dir/depend

