# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.7

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/xdf/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/xdf/catkin_ws/build

# Include any dependencies generated for this target.
include darknet_ros/sigverse_open_manipulator/CMakeFiles/finding_auto.dir/depend.make

# Include the progress variables for this target.
include darknet_ros/sigverse_open_manipulator/CMakeFiles/finding_auto.dir/progress.make

# Include the compile flags for this target's objects.
include darknet_ros/sigverse_open_manipulator/CMakeFiles/finding_auto.dir/flags.make

darknet_ros/sigverse_open_manipulator/CMakeFiles/finding_auto.dir/src/finding_auto.cpp.o: darknet_ros/sigverse_open_manipulator/CMakeFiles/finding_auto.dir/flags.make
darknet_ros/sigverse_open_manipulator/CMakeFiles/finding_auto.dir/src/finding_auto.cpp.o: /home/xdf/catkin_ws/src/darknet_ros/sigverse_open_manipulator/src/finding_auto.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/xdf/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object darknet_ros/sigverse_open_manipulator/CMakeFiles/finding_auto.dir/src/finding_auto.cpp.o"
	cd /home/xdf/catkin_ws/build/darknet_ros/sigverse_open_manipulator && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/finding_auto.dir/src/finding_auto.cpp.o -c /home/xdf/catkin_ws/src/darknet_ros/sigverse_open_manipulator/src/finding_auto.cpp

darknet_ros/sigverse_open_manipulator/CMakeFiles/finding_auto.dir/src/finding_auto.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/finding_auto.dir/src/finding_auto.cpp.i"
	cd /home/xdf/catkin_ws/build/darknet_ros/sigverse_open_manipulator && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/xdf/catkin_ws/src/darknet_ros/sigverse_open_manipulator/src/finding_auto.cpp > CMakeFiles/finding_auto.dir/src/finding_auto.cpp.i

darknet_ros/sigverse_open_manipulator/CMakeFiles/finding_auto.dir/src/finding_auto.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/finding_auto.dir/src/finding_auto.cpp.s"
	cd /home/xdf/catkin_ws/build/darknet_ros/sigverse_open_manipulator && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/xdf/catkin_ws/src/darknet_ros/sigverse_open_manipulator/src/finding_auto.cpp -o CMakeFiles/finding_auto.dir/src/finding_auto.cpp.s

darknet_ros/sigverse_open_manipulator/CMakeFiles/finding_auto.dir/src/finding_auto.cpp.o.requires:

.PHONY : darknet_ros/sigverse_open_manipulator/CMakeFiles/finding_auto.dir/src/finding_auto.cpp.o.requires

darknet_ros/sigverse_open_manipulator/CMakeFiles/finding_auto.dir/src/finding_auto.cpp.o.provides: darknet_ros/sigverse_open_manipulator/CMakeFiles/finding_auto.dir/src/finding_auto.cpp.o.requires
	$(MAKE) -f darknet_ros/sigverse_open_manipulator/CMakeFiles/finding_auto.dir/build.make darknet_ros/sigverse_open_manipulator/CMakeFiles/finding_auto.dir/src/finding_auto.cpp.o.provides.build
.PHONY : darknet_ros/sigverse_open_manipulator/CMakeFiles/finding_auto.dir/src/finding_auto.cpp.o.provides

darknet_ros/sigverse_open_manipulator/CMakeFiles/finding_auto.dir/src/finding_auto.cpp.o.provides.build: darknet_ros/sigverse_open_manipulator/CMakeFiles/finding_auto.dir/src/finding_auto.cpp.o


# Object files for target finding_auto
finding_auto_OBJECTS = \
"CMakeFiles/finding_auto.dir/src/finding_auto.cpp.o"

# External object files for target finding_auto
finding_auto_EXTERNAL_OBJECTS =

/home/xdf/catkin_ws/devel/lib/sigverse_open_manipulator/finding_auto: darknet_ros/sigverse_open_manipulator/CMakeFiles/finding_auto.dir/src/finding_auto.cpp.o
/home/xdf/catkin_ws/devel/lib/sigverse_open_manipulator/finding_auto: darknet_ros/sigverse_open_manipulator/CMakeFiles/finding_auto.dir/build.make
/home/xdf/catkin_ws/devel/lib/sigverse_open_manipulator/finding_auto: /opt/ros/kinetic/lib/libtf.so
/home/xdf/catkin_ws/devel/lib/sigverse_open_manipulator/finding_auto: /opt/ros/kinetic/lib/libtf2_ros.so
/home/xdf/catkin_ws/devel/lib/sigverse_open_manipulator/finding_auto: /opt/ros/kinetic/lib/libactionlib.so
/home/xdf/catkin_ws/devel/lib/sigverse_open_manipulator/finding_auto: /opt/ros/kinetic/lib/libmessage_filters.so
/home/xdf/catkin_ws/devel/lib/sigverse_open_manipulator/finding_auto: /opt/ros/kinetic/lib/libroscpp.so
/home/xdf/catkin_ws/devel/lib/sigverse_open_manipulator/finding_auto: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/xdf/catkin_ws/devel/lib/sigverse_open_manipulator/finding_auto: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/xdf/catkin_ws/devel/lib/sigverse_open_manipulator/finding_auto: /opt/ros/kinetic/lib/libxmlrpcpp.so
/home/xdf/catkin_ws/devel/lib/sigverse_open_manipulator/finding_auto: /opt/ros/kinetic/lib/libtf2.so
/home/xdf/catkin_ws/devel/lib/sigverse_open_manipulator/finding_auto: /opt/ros/kinetic/lib/libcv_bridge.so
/home/xdf/catkin_ws/devel/lib/sigverse_open_manipulator/finding_auto: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_core3.so.3.3.1
/home/xdf/catkin_ws/devel/lib/sigverse_open_manipulator/finding_auto: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgproc3.so.3.3.1
/home/xdf/catkin_ws/devel/lib/sigverse_open_manipulator/finding_auto: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgcodecs3.so.3.3.1
/home/xdf/catkin_ws/devel/lib/sigverse_open_manipulator/finding_auto: /opt/ros/kinetic/lib/librosconsole.so
/home/xdf/catkin_ws/devel/lib/sigverse_open_manipulator/finding_auto: /opt/ros/kinetic/lib/librosconsole_log4cxx.so
/home/xdf/catkin_ws/devel/lib/sigverse_open_manipulator/finding_auto: /opt/ros/kinetic/lib/librosconsole_backend_interface.so
/home/xdf/catkin_ws/devel/lib/sigverse_open_manipulator/finding_auto: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/xdf/catkin_ws/devel/lib/sigverse_open_manipulator/finding_auto: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/xdf/catkin_ws/devel/lib/sigverse_open_manipulator/finding_auto: /opt/ros/kinetic/lib/libroscpp_serialization.so
/home/xdf/catkin_ws/devel/lib/sigverse_open_manipulator/finding_auto: /opt/ros/kinetic/lib/librostime.so
/home/xdf/catkin_ws/devel/lib/sigverse_open_manipulator/finding_auto: /opt/ros/kinetic/lib/libcpp_common.so
/home/xdf/catkin_ws/devel/lib/sigverse_open_manipulator/finding_auto: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/xdf/catkin_ws/devel/lib/sigverse_open_manipulator/finding_auto: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/xdf/catkin_ws/devel/lib/sigverse_open_manipulator/finding_auto: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/xdf/catkin_ws/devel/lib/sigverse_open_manipulator/finding_auto: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/xdf/catkin_ws/devel/lib/sigverse_open_manipulator/finding_auto: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/xdf/catkin_ws/devel/lib/sigverse_open_manipulator/finding_auto: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/xdf/catkin_ws/devel/lib/sigverse_open_manipulator/finding_auto: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/xdf/catkin_ws/devel/lib/sigverse_open_manipulator/finding_auto: /opt/ros/kinetic/lib/libroscpp_serialization.so
/home/xdf/catkin_ws/devel/lib/sigverse_open_manipulator/finding_auto: /opt/ros/kinetic/lib/librostime.so
/home/xdf/catkin_ws/devel/lib/sigverse_open_manipulator/finding_auto: /opt/ros/kinetic/lib/libcpp_common.so
/home/xdf/catkin_ws/devel/lib/sigverse_open_manipulator/finding_auto: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/xdf/catkin_ws/devel/lib/sigverse_open_manipulator/finding_auto: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/xdf/catkin_ws/devel/lib/sigverse_open_manipulator/finding_auto: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/xdf/catkin_ws/devel/lib/sigverse_open_manipulator/finding_auto: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/xdf/catkin_ws/devel/lib/sigverse_open_manipulator/finding_auto: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/xdf/catkin_ws/devel/lib/sigverse_open_manipulator/finding_auto: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/xdf/catkin_ws/devel/lib/sigverse_open_manipulator/finding_auto: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/xdf/catkin_ws/devel/lib/sigverse_open_manipulator/finding_auto: darknet_ros/sigverse_open_manipulator/CMakeFiles/finding_auto.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/xdf/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/xdf/catkin_ws/devel/lib/sigverse_open_manipulator/finding_auto"
	cd /home/xdf/catkin_ws/build/darknet_ros/sigverse_open_manipulator && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/finding_auto.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
darknet_ros/sigverse_open_manipulator/CMakeFiles/finding_auto.dir/build: /home/xdf/catkin_ws/devel/lib/sigverse_open_manipulator/finding_auto

.PHONY : darknet_ros/sigverse_open_manipulator/CMakeFiles/finding_auto.dir/build

darknet_ros/sigverse_open_manipulator/CMakeFiles/finding_auto.dir/requires: darknet_ros/sigverse_open_manipulator/CMakeFiles/finding_auto.dir/src/finding_auto.cpp.o.requires

.PHONY : darknet_ros/sigverse_open_manipulator/CMakeFiles/finding_auto.dir/requires

darknet_ros/sigverse_open_manipulator/CMakeFiles/finding_auto.dir/clean:
	cd /home/xdf/catkin_ws/build/darknet_ros/sigverse_open_manipulator && $(CMAKE_COMMAND) -P CMakeFiles/finding_auto.dir/cmake_clean.cmake
.PHONY : darknet_ros/sigverse_open_manipulator/CMakeFiles/finding_auto.dir/clean

darknet_ros/sigverse_open_manipulator/CMakeFiles/finding_auto.dir/depend:
	cd /home/xdf/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/xdf/catkin_ws/src /home/xdf/catkin_ws/src/darknet_ros/sigverse_open_manipulator /home/xdf/catkin_ws/build /home/xdf/catkin_ws/build/darknet_ros/sigverse_open_manipulator /home/xdf/catkin_ws/build/darknet_ros/sigverse_open_manipulator/CMakeFiles/finding_auto.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : darknet_ros/sigverse_open_manipulator/CMakeFiles/finding_auto.dir/depend
