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
CMAKE_SOURCE_DIR = /home/sinofairy/workroot/SlamParse/eigen-3.3.7

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/sinofairy/workroot/SlamParse/eigen-3.3.7/build

# Include any dependencies generated for this target.
include test/CMakeFiles/bdcsvd_9.dir/depend.make

# Include the progress variables for this target.
include test/CMakeFiles/bdcsvd_9.dir/progress.make

# Include the compile flags for this target's objects.
include test/CMakeFiles/bdcsvd_9.dir/flags.make

test/CMakeFiles/bdcsvd_9.dir/bdcsvd.cpp.o: test/CMakeFiles/bdcsvd_9.dir/flags.make
test/CMakeFiles/bdcsvd_9.dir/bdcsvd.cpp.o: ../test/bdcsvd.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/sinofairy/workroot/SlamParse/eigen-3.3.7/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object test/CMakeFiles/bdcsvd_9.dir/bdcsvd.cpp.o"
	cd /home/sinofairy/workroot/SlamParse/eigen-3.3.7/build/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/bdcsvd_9.dir/bdcsvd.cpp.o -c /home/sinofairy/workroot/SlamParse/eigen-3.3.7/test/bdcsvd.cpp

test/CMakeFiles/bdcsvd_9.dir/bdcsvd.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/bdcsvd_9.dir/bdcsvd.cpp.i"
	cd /home/sinofairy/workroot/SlamParse/eigen-3.3.7/build/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/sinofairy/workroot/SlamParse/eigen-3.3.7/test/bdcsvd.cpp > CMakeFiles/bdcsvd_9.dir/bdcsvd.cpp.i

test/CMakeFiles/bdcsvd_9.dir/bdcsvd.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/bdcsvd_9.dir/bdcsvd.cpp.s"
	cd /home/sinofairy/workroot/SlamParse/eigen-3.3.7/build/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/sinofairy/workroot/SlamParse/eigen-3.3.7/test/bdcsvd.cpp -o CMakeFiles/bdcsvd_9.dir/bdcsvd.cpp.s

test/CMakeFiles/bdcsvd_9.dir/bdcsvd.cpp.o.requires:

.PHONY : test/CMakeFiles/bdcsvd_9.dir/bdcsvd.cpp.o.requires

test/CMakeFiles/bdcsvd_9.dir/bdcsvd.cpp.o.provides: test/CMakeFiles/bdcsvd_9.dir/bdcsvd.cpp.o.requires
	$(MAKE) -f test/CMakeFiles/bdcsvd_9.dir/build.make test/CMakeFiles/bdcsvd_9.dir/bdcsvd.cpp.o.provides.build
.PHONY : test/CMakeFiles/bdcsvd_9.dir/bdcsvd.cpp.o.provides

test/CMakeFiles/bdcsvd_9.dir/bdcsvd.cpp.o.provides.build: test/CMakeFiles/bdcsvd_9.dir/bdcsvd.cpp.o


# Object files for target bdcsvd_9
bdcsvd_9_OBJECTS = \
"CMakeFiles/bdcsvd_9.dir/bdcsvd.cpp.o"

# External object files for target bdcsvd_9
bdcsvd_9_EXTERNAL_OBJECTS =

test/bdcsvd_9: test/CMakeFiles/bdcsvd_9.dir/bdcsvd.cpp.o
test/bdcsvd_9: test/CMakeFiles/bdcsvd_9.dir/build.make
test/bdcsvd_9: test/CMakeFiles/bdcsvd_9.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/sinofairy/workroot/SlamParse/eigen-3.3.7/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable bdcsvd_9"
	cd /home/sinofairy/workroot/SlamParse/eigen-3.3.7/build/test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/bdcsvd_9.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/CMakeFiles/bdcsvd_9.dir/build: test/bdcsvd_9

.PHONY : test/CMakeFiles/bdcsvd_9.dir/build

test/CMakeFiles/bdcsvd_9.dir/requires: test/CMakeFiles/bdcsvd_9.dir/bdcsvd.cpp.o.requires

.PHONY : test/CMakeFiles/bdcsvd_9.dir/requires

test/CMakeFiles/bdcsvd_9.dir/clean:
	cd /home/sinofairy/workroot/SlamParse/eigen-3.3.7/build/test && $(CMAKE_COMMAND) -P CMakeFiles/bdcsvd_9.dir/cmake_clean.cmake
.PHONY : test/CMakeFiles/bdcsvd_9.dir/clean

test/CMakeFiles/bdcsvd_9.dir/depend:
	cd /home/sinofairy/workroot/SlamParse/eigen-3.3.7/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/sinofairy/workroot/SlamParse/eigen-3.3.7 /home/sinofairy/workroot/SlamParse/eigen-3.3.7/test /home/sinofairy/workroot/SlamParse/eigen-3.3.7/build /home/sinofairy/workroot/SlamParse/eigen-3.3.7/build/test /home/sinofairy/workroot/SlamParse/eigen-3.3.7/build/test/CMakeFiles/bdcsvd_9.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/CMakeFiles/bdcsvd_9.dir/depend

