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
include test/CMakeFiles/exceptions.dir/depend.make

# Include the progress variables for this target.
include test/CMakeFiles/exceptions.dir/progress.make

# Include the compile flags for this target's objects.
include test/CMakeFiles/exceptions.dir/flags.make

test/CMakeFiles/exceptions.dir/exceptions.cpp.o: test/CMakeFiles/exceptions.dir/flags.make
test/CMakeFiles/exceptions.dir/exceptions.cpp.o: ../test/exceptions.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/sinofairy/workroot/SlamParse/eigen-3.3.7/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object test/CMakeFiles/exceptions.dir/exceptions.cpp.o"
	cd /home/sinofairy/workroot/SlamParse/eigen-3.3.7/build/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/exceptions.dir/exceptions.cpp.o -c /home/sinofairy/workroot/SlamParse/eigen-3.3.7/test/exceptions.cpp

test/CMakeFiles/exceptions.dir/exceptions.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/exceptions.dir/exceptions.cpp.i"
	cd /home/sinofairy/workroot/SlamParse/eigen-3.3.7/build/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/sinofairy/workroot/SlamParse/eigen-3.3.7/test/exceptions.cpp > CMakeFiles/exceptions.dir/exceptions.cpp.i

test/CMakeFiles/exceptions.dir/exceptions.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/exceptions.dir/exceptions.cpp.s"
	cd /home/sinofairy/workroot/SlamParse/eigen-3.3.7/build/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/sinofairy/workroot/SlamParse/eigen-3.3.7/test/exceptions.cpp -o CMakeFiles/exceptions.dir/exceptions.cpp.s

test/CMakeFiles/exceptions.dir/exceptions.cpp.o.requires:

.PHONY : test/CMakeFiles/exceptions.dir/exceptions.cpp.o.requires

test/CMakeFiles/exceptions.dir/exceptions.cpp.o.provides: test/CMakeFiles/exceptions.dir/exceptions.cpp.o.requires
	$(MAKE) -f test/CMakeFiles/exceptions.dir/build.make test/CMakeFiles/exceptions.dir/exceptions.cpp.o.provides.build
.PHONY : test/CMakeFiles/exceptions.dir/exceptions.cpp.o.provides

test/CMakeFiles/exceptions.dir/exceptions.cpp.o.provides.build: test/CMakeFiles/exceptions.dir/exceptions.cpp.o


# Object files for target exceptions
exceptions_OBJECTS = \
"CMakeFiles/exceptions.dir/exceptions.cpp.o"

# External object files for target exceptions
exceptions_EXTERNAL_OBJECTS =

test/exceptions: test/CMakeFiles/exceptions.dir/exceptions.cpp.o
test/exceptions: test/CMakeFiles/exceptions.dir/build.make
test/exceptions: test/CMakeFiles/exceptions.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/sinofairy/workroot/SlamParse/eigen-3.3.7/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable exceptions"
	cd /home/sinofairy/workroot/SlamParse/eigen-3.3.7/build/test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/exceptions.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/CMakeFiles/exceptions.dir/build: test/exceptions

.PHONY : test/CMakeFiles/exceptions.dir/build

test/CMakeFiles/exceptions.dir/requires: test/CMakeFiles/exceptions.dir/exceptions.cpp.o.requires

.PHONY : test/CMakeFiles/exceptions.dir/requires

test/CMakeFiles/exceptions.dir/clean:
	cd /home/sinofairy/workroot/SlamParse/eigen-3.3.7/build/test && $(CMAKE_COMMAND) -P CMakeFiles/exceptions.dir/cmake_clean.cmake
.PHONY : test/CMakeFiles/exceptions.dir/clean

test/CMakeFiles/exceptions.dir/depend:
	cd /home/sinofairy/workroot/SlamParse/eigen-3.3.7/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/sinofairy/workroot/SlamParse/eigen-3.3.7 /home/sinofairy/workroot/SlamParse/eigen-3.3.7/test /home/sinofairy/workroot/SlamParse/eigen-3.3.7/build /home/sinofairy/workroot/SlamParse/eigen-3.3.7/build/test /home/sinofairy/workroot/SlamParse/eigen-3.3.7/build/test/CMakeFiles/exceptions.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/CMakeFiles/exceptions.dir/depend

