# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/lyu/Documents/catkin_evis/lie-spline-experiments

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/lyu/Documents/catkin_evis/lie-spline-experiments/build

# Include any dependencies generated for this target.
include test/CMakeFiles/test_ceres_spline_helper_old.dir/depend.make

# Include the progress variables for this target.
include test/CMakeFiles/test_ceres_spline_helper_old.dir/progress.make

# Include the compile flags for this target's objects.
include test/CMakeFiles/test_ceres_spline_helper_old.dir/flags.make

test/CMakeFiles/test_ceres_spline_helper_old.dir/src/test_ceres_spline_helper_old.cpp.o: test/CMakeFiles/test_ceres_spline_helper_old.dir/flags.make
test/CMakeFiles/test_ceres_spline_helper_old.dir/src/test_ceres_spline_helper_old.cpp.o: ../test/src/test_ceres_spline_helper_old.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lyu/Documents/catkin_evis/lie-spline-experiments/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object test/CMakeFiles/test_ceres_spline_helper_old.dir/src/test_ceres_spline_helper_old.cpp.o"
	cd /home/lyu/Documents/catkin_evis/lie-spline-experiments/build/test && /usr/bin/ccache /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_ceres_spline_helper_old.dir/src/test_ceres_spline_helper_old.cpp.o -c /home/lyu/Documents/catkin_evis/lie-spline-experiments/test/src/test_ceres_spline_helper_old.cpp

test/CMakeFiles/test_ceres_spline_helper_old.dir/src/test_ceres_spline_helper_old.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_ceres_spline_helper_old.dir/src/test_ceres_spline_helper_old.cpp.i"
	cd /home/lyu/Documents/catkin_evis/lie-spline-experiments/build/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lyu/Documents/catkin_evis/lie-spline-experiments/test/src/test_ceres_spline_helper_old.cpp > CMakeFiles/test_ceres_spline_helper_old.dir/src/test_ceres_spline_helper_old.cpp.i

test/CMakeFiles/test_ceres_spline_helper_old.dir/src/test_ceres_spline_helper_old.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_ceres_spline_helper_old.dir/src/test_ceres_spline_helper_old.cpp.s"
	cd /home/lyu/Documents/catkin_evis/lie-spline-experiments/build/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lyu/Documents/catkin_evis/lie-spline-experiments/test/src/test_ceres_spline_helper_old.cpp -o CMakeFiles/test_ceres_spline_helper_old.dir/src/test_ceres_spline_helper_old.cpp.s

test/CMakeFiles/test_ceres_spline_helper_old.dir/src/test_ceres_spline_helper_old.cpp.o.requires:

.PHONY : test/CMakeFiles/test_ceres_spline_helper_old.dir/src/test_ceres_spline_helper_old.cpp.o.requires

test/CMakeFiles/test_ceres_spline_helper_old.dir/src/test_ceres_spline_helper_old.cpp.o.provides: test/CMakeFiles/test_ceres_spline_helper_old.dir/src/test_ceres_spline_helper_old.cpp.o.requires
	$(MAKE) -f test/CMakeFiles/test_ceres_spline_helper_old.dir/build.make test/CMakeFiles/test_ceres_spline_helper_old.dir/src/test_ceres_spline_helper_old.cpp.o.provides.build
.PHONY : test/CMakeFiles/test_ceres_spline_helper_old.dir/src/test_ceres_spline_helper_old.cpp.o.provides

test/CMakeFiles/test_ceres_spline_helper_old.dir/src/test_ceres_spline_helper_old.cpp.o.provides.build: test/CMakeFiles/test_ceres_spline_helper_old.dir/src/test_ceres_spline_helper_old.cpp.o


# Object files for target test_ceres_spline_helper_old
test_ceres_spline_helper_old_OBJECTS = \
"CMakeFiles/test_ceres_spline_helper_old.dir/src/test_ceres_spline_helper_old.cpp.o"

# External object files for target test_ceres_spline_helper_old
test_ceres_spline_helper_old_EXTERNAL_OBJECTS =

test/test_ceres_spline_helper_old: test/CMakeFiles/test_ceres_spline_helper_old.dir/src/test_ceres_spline_helper_old.cpp.o
test/test_ceres_spline_helper_old: test/CMakeFiles/test_ceres_spline_helper_old.dir/build.make
test/test_ceres_spline_helper_old: lib/libgtest.a
test/test_ceres_spline_helper_old: lib/libgtest_main.a
test/test_ceres_spline_helper_old: lib/libgtest.a
test/test_ceres_spline_helper_old: test/CMakeFiles/test_ceres_spline_helper_old.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/lyu/Documents/catkin_evis/lie-spline-experiments/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable test_ceres_spline_helper_old"
	cd /home/lyu/Documents/catkin_evis/lie-spline-experiments/build/test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_ceres_spline_helper_old.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/CMakeFiles/test_ceres_spline_helper_old.dir/build: test/test_ceres_spline_helper_old

.PHONY : test/CMakeFiles/test_ceres_spline_helper_old.dir/build

test/CMakeFiles/test_ceres_spline_helper_old.dir/requires: test/CMakeFiles/test_ceres_spline_helper_old.dir/src/test_ceres_spline_helper_old.cpp.o.requires

.PHONY : test/CMakeFiles/test_ceres_spline_helper_old.dir/requires

test/CMakeFiles/test_ceres_spline_helper_old.dir/clean:
	cd /home/lyu/Documents/catkin_evis/lie-spline-experiments/build/test && $(CMAKE_COMMAND) -P CMakeFiles/test_ceres_spline_helper_old.dir/cmake_clean.cmake
.PHONY : test/CMakeFiles/test_ceres_spline_helper_old.dir/clean

test/CMakeFiles/test_ceres_spline_helper_old.dir/depend:
	cd /home/lyu/Documents/catkin_evis/lie-spline-experiments/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lyu/Documents/catkin_evis/lie-spline-experiments /home/lyu/Documents/catkin_evis/lie-spline-experiments/test /home/lyu/Documents/catkin_evis/lie-spline-experiments/build /home/lyu/Documents/catkin_evis/lie-spline-experiments/build/test /home/lyu/Documents/catkin_evis/lie-spline-experiments/build/test/CMakeFiles/test_ceres_spline_helper_old.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/CMakeFiles/test_ceres_spline_helper_old.dir/depend
