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
CMAKE_SOURCE_DIR = /home/yilangliu/a1_real_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/yilangliu/a1_real_ws/build

# Include any dependencies generated for this target.
include unitree_legged_real/CMakeFiles/a1_dynamics_lib.dir/depend.make

# Include the progress variables for this target.
include unitree_legged_real/CMakeFiles/a1_dynamics_lib.dir/progress.make

# Include the compile flags for this target's objects.
include unitree_legged_real/CMakeFiles/a1_dynamics_lib.dir/flags.make

unitree_legged_real/CMakeFiles/a1_dynamics_lib.dir/src/exe/A1_dynamics_pino.cpp.o: unitree_legged_real/CMakeFiles/a1_dynamics_lib.dir/flags.make
unitree_legged_real/CMakeFiles/a1_dynamics_lib.dir/src/exe/A1_dynamics_pino.cpp.o: /home/yilangliu/a1_real_ws/src/unitree_legged_real/src/exe/A1_dynamics_pino.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yilangliu/a1_real_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object unitree_legged_real/CMakeFiles/a1_dynamics_lib.dir/src/exe/A1_dynamics_pino.cpp.o"
	cd /home/yilangliu/a1_real_ws/build/unitree_legged_real && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/a1_dynamics_lib.dir/src/exe/A1_dynamics_pino.cpp.o -c /home/yilangliu/a1_real_ws/src/unitree_legged_real/src/exe/A1_dynamics_pino.cpp

unitree_legged_real/CMakeFiles/a1_dynamics_lib.dir/src/exe/A1_dynamics_pino.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/a1_dynamics_lib.dir/src/exe/A1_dynamics_pino.cpp.i"
	cd /home/yilangliu/a1_real_ws/build/unitree_legged_real && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yilangliu/a1_real_ws/src/unitree_legged_real/src/exe/A1_dynamics_pino.cpp > CMakeFiles/a1_dynamics_lib.dir/src/exe/A1_dynamics_pino.cpp.i

unitree_legged_real/CMakeFiles/a1_dynamics_lib.dir/src/exe/A1_dynamics_pino.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/a1_dynamics_lib.dir/src/exe/A1_dynamics_pino.cpp.s"
	cd /home/yilangliu/a1_real_ws/build/unitree_legged_real && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yilangliu/a1_real_ws/src/unitree_legged_real/src/exe/A1_dynamics_pino.cpp -o CMakeFiles/a1_dynamics_lib.dir/src/exe/A1_dynamics_pino.cpp.s

# Object files for target a1_dynamics_lib
a1_dynamics_lib_OBJECTS = \
"CMakeFiles/a1_dynamics_lib.dir/src/exe/A1_dynamics_pino.cpp.o"

# External object files for target a1_dynamics_lib
a1_dynamics_lib_EXTERNAL_OBJECTS =

/home/yilangliu/a1_real_ws/devel/lib/liba1_dynamics_lib.so: unitree_legged_real/CMakeFiles/a1_dynamics_lib.dir/src/exe/A1_dynamics_pino.cpp.o
/home/yilangliu/a1_real_ws/devel/lib/liba1_dynamics_lib.so: unitree_legged_real/CMakeFiles/a1_dynamics_lib.dir/build.make
/home/yilangliu/a1_real_ws/devel/lib/liba1_dynamics_lib.so: unitree_legged_real/CMakeFiles/a1_dynamics_lib.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/yilangliu/a1_real_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library /home/yilangliu/a1_real_ws/devel/lib/liba1_dynamics_lib.so"
	cd /home/yilangliu/a1_real_ws/build/unitree_legged_real && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/a1_dynamics_lib.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
unitree_legged_real/CMakeFiles/a1_dynamics_lib.dir/build: /home/yilangliu/a1_real_ws/devel/lib/liba1_dynamics_lib.so

.PHONY : unitree_legged_real/CMakeFiles/a1_dynamics_lib.dir/build

unitree_legged_real/CMakeFiles/a1_dynamics_lib.dir/clean:
	cd /home/yilangliu/a1_real_ws/build/unitree_legged_real && $(CMAKE_COMMAND) -P CMakeFiles/a1_dynamics_lib.dir/cmake_clean.cmake
.PHONY : unitree_legged_real/CMakeFiles/a1_dynamics_lib.dir/clean

unitree_legged_real/CMakeFiles/a1_dynamics_lib.dir/depend:
	cd /home/yilangliu/a1_real_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/yilangliu/a1_real_ws/src /home/yilangliu/a1_real_ws/src/unitree_legged_real /home/yilangliu/a1_real_ws/build /home/yilangliu/a1_real_ws/build/unitree_legged_real /home/yilangliu/a1_real_ws/build/unitree_legged_real/CMakeFiles/a1_dynamics_lib.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : unitree_legged_real/CMakeFiles/a1_dynamics_lib.dir/depend

