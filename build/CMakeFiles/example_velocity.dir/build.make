# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/tianyu/github/Robotic-Dog-Interface-AIMS-Lab

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/tianyu/github/Robotic-Dog-Interface-AIMS-Lab/build

# Include any dependencies generated for this target.
include CMakeFiles/example_velocity.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/example_velocity.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/example_velocity.dir/flags.make

CMakeFiles/example_velocity.dir/externals/unitree_legged_sdk/example/example_velocity.cpp.o: CMakeFiles/example_velocity.dir/flags.make
CMakeFiles/example_velocity.dir/externals/unitree_legged_sdk/example/example_velocity.cpp.o: ../externals/unitree_legged_sdk/example/example_velocity.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tianyu/github/Robotic-Dog-Interface-AIMS-Lab/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/example_velocity.dir/externals/unitree_legged_sdk/example/example_velocity.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/example_velocity.dir/externals/unitree_legged_sdk/example/example_velocity.cpp.o -c /home/tianyu/github/Robotic-Dog-Interface-AIMS-Lab/externals/unitree_legged_sdk/example/example_velocity.cpp

CMakeFiles/example_velocity.dir/externals/unitree_legged_sdk/example/example_velocity.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/example_velocity.dir/externals/unitree_legged_sdk/example/example_velocity.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tianyu/github/Robotic-Dog-Interface-AIMS-Lab/externals/unitree_legged_sdk/example/example_velocity.cpp > CMakeFiles/example_velocity.dir/externals/unitree_legged_sdk/example/example_velocity.cpp.i

CMakeFiles/example_velocity.dir/externals/unitree_legged_sdk/example/example_velocity.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/example_velocity.dir/externals/unitree_legged_sdk/example/example_velocity.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tianyu/github/Robotic-Dog-Interface-AIMS-Lab/externals/unitree_legged_sdk/example/example_velocity.cpp -o CMakeFiles/example_velocity.dir/externals/unitree_legged_sdk/example/example_velocity.cpp.s

# Object files for target example_velocity
example_velocity_OBJECTS = \
"CMakeFiles/example_velocity.dir/externals/unitree_legged_sdk/example/example_velocity.cpp.o"

# External object files for target example_velocity
example_velocity_EXTERNAL_OBJECTS =

example_velocity: CMakeFiles/example_velocity.dir/externals/unitree_legged_sdk/example/example_velocity.cpp.o
example_velocity: CMakeFiles/example_velocity.dir/build.make
example_velocity: CMakeFiles/example_velocity.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/tianyu/github/Robotic-Dog-Interface-AIMS-Lab/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable example_velocity"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/example_velocity.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/example_velocity.dir/build: example_velocity

.PHONY : CMakeFiles/example_velocity.dir/build

CMakeFiles/example_velocity.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/example_velocity.dir/cmake_clean.cmake
.PHONY : CMakeFiles/example_velocity.dir/clean

CMakeFiles/example_velocity.dir/depend:
	cd /home/tianyu/github/Robotic-Dog-Interface-AIMS-Lab/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/tianyu/github/Robotic-Dog-Interface-AIMS-Lab /home/tianyu/github/Robotic-Dog-Interface-AIMS-Lab /home/tianyu/github/Robotic-Dog-Interface-AIMS-Lab/build /home/tianyu/github/Robotic-Dog-Interface-AIMS-Lab/build /home/tianyu/github/Robotic-Dog-Interface-AIMS-Lab/build/CMakeFiles/example_velocity.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/example_velocity.dir/depend
