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
include vision_opencv/image_geometry/test/CMakeFiles/image_geometry-utest.dir/depend.make

# Include the progress variables for this target.
include vision_opencv/image_geometry/test/CMakeFiles/image_geometry-utest.dir/progress.make

# Include the compile flags for this target's objects.
include vision_opencv/image_geometry/test/CMakeFiles/image_geometry-utest.dir/flags.make

vision_opencv/image_geometry/test/CMakeFiles/image_geometry-utest.dir/utest.cpp.o: vision_opencv/image_geometry/test/CMakeFiles/image_geometry-utest.dir/flags.make
vision_opencv/image_geometry/test/CMakeFiles/image_geometry-utest.dir/utest.cpp.o: /home/avinash/Deep_Learning_Crazyflie/src/vision_opencv/image_geometry/test/utest.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/avinash/Deep_Learning_Crazyflie/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object vision_opencv/image_geometry/test/CMakeFiles/image_geometry-utest.dir/utest.cpp.o"
	cd /home/avinash/Deep_Learning_Crazyflie/build/vision_opencv/image_geometry/test && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/image_geometry-utest.dir/utest.cpp.o -c /home/avinash/Deep_Learning_Crazyflie/src/vision_opencv/image_geometry/test/utest.cpp

vision_opencv/image_geometry/test/CMakeFiles/image_geometry-utest.dir/utest.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/image_geometry-utest.dir/utest.cpp.i"
	cd /home/avinash/Deep_Learning_Crazyflie/build/vision_opencv/image_geometry/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/avinash/Deep_Learning_Crazyflie/src/vision_opencv/image_geometry/test/utest.cpp > CMakeFiles/image_geometry-utest.dir/utest.cpp.i

vision_opencv/image_geometry/test/CMakeFiles/image_geometry-utest.dir/utest.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/image_geometry-utest.dir/utest.cpp.s"
	cd /home/avinash/Deep_Learning_Crazyflie/build/vision_opencv/image_geometry/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/avinash/Deep_Learning_Crazyflie/src/vision_opencv/image_geometry/test/utest.cpp -o CMakeFiles/image_geometry-utest.dir/utest.cpp.s

vision_opencv/image_geometry/test/CMakeFiles/image_geometry-utest.dir/utest.cpp.o.requires:
.PHONY : vision_opencv/image_geometry/test/CMakeFiles/image_geometry-utest.dir/utest.cpp.o.requires

vision_opencv/image_geometry/test/CMakeFiles/image_geometry-utest.dir/utest.cpp.o.provides: vision_opencv/image_geometry/test/CMakeFiles/image_geometry-utest.dir/utest.cpp.o.requires
	$(MAKE) -f vision_opencv/image_geometry/test/CMakeFiles/image_geometry-utest.dir/build.make vision_opencv/image_geometry/test/CMakeFiles/image_geometry-utest.dir/utest.cpp.o.provides.build
.PHONY : vision_opencv/image_geometry/test/CMakeFiles/image_geometry-utest.dir/utest.cpp.o.provides

vision_opencv/image_geometry/test/CMakeFiles/image_geometry-utest.dir/utest.cpp.o.provides.build: vision_opencv/image_geometry/test/CMakeFiles/image_geometry-utest.dir/utest.cpp.o

# Object files for target image_geometry-utest
image_geometry__utest_OBJECTS = \
"CMakeFiles/image_geometry-utest.dir/utest.cpp.o"

# External object files for target image_geometry-utest
image_geometry__utest_EXTERNAL_OBJECTS =

/home/avinash/Deep_Learning_Crazyflie/devel/lib/image_geometry/image_geometry-utest: vision_opencv/image_geometry/test/CMakeFiles/image_geometry-utest.dir/utest.cpp.o
/home/avinash/Deep_Learning_Crazyflie/devel/lib/image_geometry/image_geometry-utest: vision_opencv/image_geometry/test/CMakeFiles/image_geometry-utest.dir/build.make
/home/avinash/Deep_Learning_Crazyflie/devel/lib/image_geometry/image_geometry-utest: gtest/libgtest.so
/home/avinash/Deep_Learning_Crazyflie/devel/lib/image_geometry/image_geometry-utest: /home/avinash/Deep_Learning_Crazyflie/devel/lib/libimage_geometry.so
/home/avinash/Deep_Learning_Crazyflie/devel/lib/image_geometry/image_geometry-utest: /usr/lib/i386-linux-gnu/libopencv_videostab.so.2.4.8
/home/avinash/Deep_Learning_Crazyflie/devel/lib/image_geometry/image_geometry-utest: /usr/lib/i386-linux-gnu/libopencv_superres.so.2.4.8
/home/avinash/Deep_Learning_Crazyflie/devel/lib/image_geometry/image_geometry-utest: /usr/lib/i386-linux-gnu/libopencv_stitching.so.2.4.8
/home/avinash/Deep_Learning_Crazyflie/devel/lib/image_geometry/image_geometry-utest: /usr/lib/i386-linux-gnu/libopencv_ocl.so.2.4.8
/home/avinash/Deep_Learning_Crazyflie/devel/lib/image_geometry/image_geometry-utest: /usr/lib/i386-linux-gnu/libopencv_gpu.so.2.4.8
/home/avinash/Deep_Learning_Crazyflie/devel/lib/image_geometry/image_geometry-utest: /usr/lib/i386-linux-gnu/libopencv_contrib.so.2.4.8
/home/avinash/Deep_Learning_Crazyflie/devel/lib/image_geometry/image_geometry-utest: /usr/lib/i386-linux-gnu/libopencv_photo.so.2.4.8
/home/avinash/Deep_Learning_Crazyflie/devel/lib/image_geometry/image_geometry-utest: /usr/lib/i386-linux-gnu/libopencv_legacy.so.2.4.8
/home/avinash/Deep_Learning_Crazyflie/devel/lib/image_geometry/image_geometry-utest: /usr/lib/i386-linux-gnu/libopencv_video.so.2.4.8
/home/avinash/Deep_Learning_Crazyflie/devel/lib/image_geometry/image_geometry-utest: /usr/lib/i386-linux-gnu/libopencv_objdetect.so.2.4.8
/home/avinash/Deep_Learning_Crazyflie/devel/lib/image_geometry/image_geometry-utest: /usr/lib/i386-linux-gnu/libopencv_ml.so.2.4.8
/home/avinash/Deep_Learning_Crazyflie/devel/lib/image_geometry/image_geometry-utest: /usr/lib/i386-linux-gnu/libopencv_calib3d.so.2.4.8
/home/avinash/Deep_Learning_Crazyflie/devel/lib/image_geometry/image_geometry-utest: /usr/lib/i386-linux-gnu/libopencv_features2d.so.2.4.8
/home/avinash/Deep_Learning_Crazyflie/devel/lib/image_geometry/image_geometry-utest: /usr/lib/i386-linux-gnu/libopencv_highgui.so.2.4.8
/home/avinash/Deep_Learning_Crazyflie/devel/lib/image_geometry/image_geometry-utest: /usr/lib/i386-linux-gnu/libopencv_imgproc.so.2.4.8
/home/avinash/Deep_Learning_Crazyflie/devel/lib/image_geometry/image_geometry-utest: /usr/lib/i386-linux-gnu/libopencv_flann.so.2.4.8
/home/avinash/Deep_Learning_Crazyflie/devel/lib/image_geometry/image_geometry-utest: /usr/lib/i386-linux-gnu/libopencv_core.so.2.4.8
/home/avinash/Deep_Learning_Crazyflie/devel/lib/image_geometry/image_geometry-utest: vision_opencv/image_geometry/test/CMakeFiles/image_geometry-utest.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable /home/avinash/Deep_Learning_Crazyflie/devel/lib/image_geometry/image_geometry-utest"
	cd /home/avinash/Deep_Learning_Crazyflie/build/vision_opencv/image_geometry/test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/image_geometry-utest.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
vision_opencv/image_geometry/test/CMakeFiles/image_geometry-utest.dir/build: /home/avinash/Deep_Learning_Crazyflie/devel/lib/image_geometry/image_geometry-utest
.PHONY : vision_opencv/image_geometry/test/CMakeFiles/image_geometry-utest.dir/build

vision_opencv/image_geometry/test/CMakeFiles/image_geometry-utest.dir/requires: vision_opencv/image_geometry/test/CMakeFiles/image_geometry-utest.dir/utest.cpp.o.requires
.PHONY : vision_opencv/image_geometry/test/CMakeFiles/image_geometry-utest.dir/requires

vision_opencv/image_geometry/test/CMakeFiles/image_geometry-utest.dir/clean:
	cd /home/avinash/Deep_Learning_Crazyflie/build/vision_opencv/image_geometry/test && $(CMAKE_COMMAND) -P CMakeFiles/image_geometry-utest.dir/cmake_clean.cmake
.PHONY : vision_opencv/image_geometry/test/CMakeFiles/image_geometry-utest.dir/clean

vision_opencv/image_geometry/test/CMakeFiles/image_geometry-utest.dir/depend:
	cd /home/avinash/Deep_Learning_Crazyflie/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/avinash/Deep_Learning_Crazyflie/src /home/avinash/Deep_Learning_Crazyflie/src/vision_opencv/image_geometry/test /home/avinash/Deep_Learning_Crazyflie/build /home/avinash/Deep_Learning_Crazyflie/build/vision_opencv/image_geometry/test /home/avinash/Deep_Learning_Crazyflie/build/vision_opencv/image_geometry/test/CMakeFiles/image_geometry-utest.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : vision_opencv/image_geometry/test/CMakeFiles/image_geometry-utest.dir/depend

