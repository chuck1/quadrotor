# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor

# Include any dependencies generated for this target.
include CMakeFiles/quadrotor.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/quadrotor.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/quadrotor.dir/flags.make

CMakeFiles/quadrotor.dir/src/quadrotor/array.cpp.o: CMakeFiles/quadrotor.dir/flags.make
CMakeFiles/quadrotor.dir/src/quadrotor/array.cpp.o: src/quadrotor/array.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/quadrotor.dir/src/quadrotor/array.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/quadrotor.dir/src/quadrotor/array.cpp.o -c /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor/src/quadrotor/array.cpp

CMakeFiles/quadrotor.dir/src/quadrotor/array.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/quadrotor.dir/src/quadrotor/array.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor/src/quadrotor/array.cpp > CMakeFiles/quadrotor.dir/src/quadrotor/array.cpp.i

CMakeFiles/quadrotor.dir/src/quadrotor/array.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/quadrotor.dir/src/quadrotor/array.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor/src/quadrotor/array.cpp -o CMakeFiles/quadrotor.dir/src/quadrotor/array.cpp.s

CMakeFiles/quadrotor.dir/src/quadrotor/array.cpp.o.requires:
.PHONY : CMakeFiles/quadrotor.dir/src/quadrotor/array.cpp.o.requires

CMakeFiles/quadrotor.dir/src/quadrotor/array.cpp.o.provides: CMakeFiles/quadrotor.dir/src/quadrotor/array.cpp.o.requires
	$(MAKE) -f CMakeFiles/quadrotor.dir/build.make CMakeFiles/quadrotor.dir/src/quadrotor/array.cpp.o.provides.build
.PHONY : CMakeFiles/quadrotor.dir/src/quadrotor/array.cpp.o.provides

CMakeFiles/quadrotor.dir/src/quadrotor/array.cpp.o.provides.build: CMakeFiles/quadrotor.dir/src/quadrotor/array.cpp.o

CMakeFiles/quadrotor.dir/src/quadrotor/attitude.cpp.o: CMakeFiles/quadrotor.dir/flags.make
CMakeFiles/quadrotor.dir/src/quadrotor/attitude.cpp.o: src/quadrotor/attitude.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/quadrotor.dir/src/quadrotor/attitude.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/quadrotor.dir/src/quadrotor/attitude.cpp.o -c /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor/src/quadrotor/attitude.cpp

CMakeFiles/quadrotor.dir/src/quadrotor/attitude.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/quadrotor.dir/src/quadrotor/attitude.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor/src/quadrotor/attitude.cpp > CMakeFiles/quadrotor.dir/src/quadrotor/attitude.cpp.i

CMakeFiles/quadrotor.dir/src/quadrotor/attitude.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/quadrotor.dir/src/quadrotor/attitude.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor/src/quadrotor/attitude.cpp -o CMakeFiles/quadrotor.dir/src/quadrotor/attitude.cpp.s

CMakeFiles/quadrotor.dir/src/quadrotor/attitude.cpp.o.requires:
.PHONY : CMakeFiles/quadrotor.dir/src/quadrotor/attitude.cpp.o.requires

CMakeFiles/quadrotor.dir/src/quadrotor/attitude.cpp.o.provides: CMakeFiles/quadrotor.dir/src/quadrotor/attitude.cpp.o.requires
	$(MAKE) -f CMakeFiles/quadrotor.dir/build.make CMakeFiles/quadrotor.dir/src/quadrotor/attitude.cpp.o.provides.build
.PHONY : CMakeFiles/quadrotor.dir/src/quadrotor/attitude.cpp.o.provides

CMakeFiles/quadrotor.dir/src/quadrotor/attitude.cpp.o.provides.build: CMakeFiles/quadrotor.dir/src/quadrotor/attitude.cpp.o

CMakeFiles/quadrotor.dir/src/quadrotor/brain.cpp.o: CMakeFiles/quadrotor.dir/flags.make
CMakeFiles/quadrotor.dir/src/quadrotor/brain.cpp.o: src/quadrotor/brain.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/quadrotor.dir/src/quadrotor/brain.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/quadrotor.dir/src/quadrotor/brain.cpp.o -c /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor/src/quadrotor/brain.cpp

CMakeFiles/quadrotor.dir/src/quadrotor/brain.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/quadrotor.dir/src/quadrotor/brain.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor/src/quadrotor/brain.cpp > CMakeFiles/quadrotor.dir/src/quadrotor/brain.cpp.i

CMakeFiles/quadrotor.dir/src/quadrotor/brain.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/quadrotor.dir/src/quadrotor/brain.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor/src/quadrotor/brain.cpp -o CMakeFiles/quadrotor.dir/src/quadrotor/brain.cpp.s

CMakeFiles/quadrotor.dir/src/quadrotor/brain.cpp.o.requires:
.PHONY : CMakeFiles/quadrotor.dir/src/quadrotor/brain.cpp.o.requires

CMakeFiles/quadrotor.dir/src/quadrotor/brain.cpp.o.provides: CMakeFiles/quadrotor.dir/src/quadrotor/brain.cpp.o.requires
	$(MAKE) -f CMakeFiles/quadrotor.dir/build.make CMakeFiles/quadrotor.dir/src/quadrotor/brain.cpp.o.provides.build
.PHONY : CMakeFiles/quadrotor.dir/src/quadrotor/brain.cpp.o.provides

CMakeFiles/quadrotor.dir/src/quadrotor/brain.cpp.o.provides.build: CMakeFiles/quadrotor.dir/src/quadrotor/brain.cpp.o

CMakeFiles/quadrotor.dir/src/quadrotor/command.cpp.o: CMakeFiles/quadrotor.dir/flags.make
CMakeFiles/quadrotor.dir/src/quadrotor/command.cpp.o: src/quadrotor/command.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor/CMakeFiles $(CMAKE_PROGRESS_4)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/quadrotor.dir/src/quadrotor/command.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/quadrotor.dir/src/quadrotor/command.cpp.o -c /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor/src/quadrotor/command.cpp

CMakeFiles/quadrotor.dir/src/quadrotor/command.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/quadrotor.dir/src/quadrotor/command.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor/src/quadrotor/command.cpp > CMakeFiles/quadrotor.dir/src/quadrotor/command.cpp.i

CMakeFiles/quadrotor.dir/src/quadrotor/command.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/quadrotor.dir/src/quadrotor/command.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor/src/quadrotor/command.cpp -o CMakeFiles/quadrotor.dir/src/quadrotor/command.cpp.s

CMakeFiles/quadrotor.dir/src/quadrotor/command.cpp.o.requires:
.PHONY : CMakeFiles/quadrotor.dir/src/quadrotor/command.cpp.o.requires

CMakeFiles/quadrotor.dir/src/quadrotor/command.cpp.o.provides: CMakeFiles/quadrotor.dir/src/quadrotor/command.cpp.o.requires
	$(MAKE) -f CMakeFiles/quadrotor.dir/build.make CMakeFiles/quadrotor.dir/src/quadrotor/command.cpp.o.provides.build
.PHONY : CMakeFiles/quadrotor.dir/src/quadrotor/command.cpp.o.provides

CMakeFiles/quadrotor.dir/src/quadrotor/command.cpp.o.provides.build: CMakeFiles/quadrotor.dir/src/quadrotor/command.cpp.o

CMakeFiles/quadrotor.dir/src/quadrotor/Input.cpp.o: CMakeFiles/quadrotor.dir/flags.make
CMakeFiles/quadrotor.dir/src/quadrotor/Input.cpp.o: src/quadrotor/Input.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor/CMakeFiles $(CMAKE_PROGRESS_5)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/quadrotor.dir/src/quadrotor/Input.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/quadrotor.dir/src/quadrotor/Input.cpp.o -c /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor/src/quadrotor/Input.cpp

CMakeFiles/quadrotor.dir/src/quadrotor/Input.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/quadrotor.dir/src/quadrotor/Input.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor/src/quadrotor/Input.cpp > CMakeFiles/quadrotor.dir/src/quadrotor/Input.cpp.i

CMakeFiles/quadrotor.dir/src/quadrotor/Input.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/quadrotor.dir/src/quadrotor/Input.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor/src/quadrotor/Input.cpp -o CMakeFiles/quadrotor.dir/src/quadrotor/Input.cpp.s

CMakeFiles/quadrotor.dir/src/quadrotor/Input.cpp.o.requires:
.PHONY : CMakeFiles/quadrotor.dir/src/quadrotor/Input.cpp.o.requires

CMakeFiles/quadrotor.dir/src/quadrotor/Input.cpp.o.provides: CMakeFiles/quadrotor.dir/src/quadrotor/Input.cpp.o.requires
	$(MAKE) -f CMakeFiles/quadrotor.dir/build.make CMakeFiles/quadrotor.dir/src/quadrotor/Input.cpp.o.provides.build
.PHONY : CMakeFiles/quadrotor.dir/src/quadrotor/Input.cpp.o.provides

CMakeFiles/quadrotor.dir/src/quadrotor/Input.cpp.o.provides.build: CMakeFiles/quadrotor.dir/src/quadrotor/Input.cpp.o

CMakeFiles/quadrotor.dir/src/quadrotor/plant.cpp.o: CMakeFiles/quadrotor.dir/flags.make
CMakeFiles/quadrotor.dir/src/quadrotor/plant.cpp.o: src/quadrotor/plant.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor/CMakeFiles $(CMAKE_PROGRESS_6)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/quadrotor.dir/src/quadrotor/plant.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/quadrotor.dir/src/quadrotor/plant.cpp.o -c /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor/src/quadrotor/plant.cpp

CMakeFiles/quadrotor.dir/src/quadrotor/plant.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/quadrotor.dir/src/quadrotor/plant.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor/src/quadrotor/plant.cpp > CMakeFiles/quadrotor.dir/src/quadrotor/plant.cpp.i

CMakeFiles/quadrotor.dir/src/quadrotor/plant.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/quadrotor.dir/src/quadrotor/plant.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor/src/quadrotor/plant.cpp -o CMakeFiles/quadrotor.dir/src/quadrotor/plant.cpp.s

CMakeFiles/quadrotor.dir/src/quadrotor/plant.cpp.o.requires:
.PHONY : CMakeFiles/quadrotor.dir/src/quadrotor/plant.cpp.o.requires

CMakeFiles/quadrotor.dir/src/quadrotor/plant.cpp.o.provides: CMakeFiles/quadrotor.dir/src/quadrotor/plant.cpp.o.requires
	$(MAKE) -f CMakeFiles/quadrotor.dir/build.make CMakeFiles/quadrotor.dir/src/quadrotor/plant.cpp.o.provides.build
.PHONY : CMakeFiles/quadrotor.dir/src/quadrotor/plant.cpp.o.provides

CMakeFiles/quadrotor.dir/src/quadrotor/plant.cpp.o.provides.build: CMakeFiles/quadrotor.dir/src/quadrotor/plant.cpp.o

CMakeFiles/quadrotor.dir/src/quadrotor/position.cpp.o: CMakeFiles/quadrotor.dir/flags.make
CMakeFiles/quadrotor.dir/src/quadrotor/position.cpp.o: src/quadrotor/position.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor/CMakeFiles $(CMAKE_PROGRESS_7)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/quadrotor.dir/src/quadrotor/position.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/quadrotor.dir/src/quadrotor/position.cpp.o -c /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor/src/quadrotor/position.cpp

CMakeFiles/quadrotor.dir/src/quadrotor/position.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/quadrotor.dir/src/quadrotor/position.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor/src/quadrotor/position.cpp > CMakeFiles/quadrotor.dir/src/quadrotor/position.cpp.i

CMakeFiles/quadrotor.dir/src/quadrotor/position.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/quadrotor.dir/src/quadrotor/position.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor/src/quadrotor/position.cpp -o CMakeFiles/quadrotor.dir/src/quadrotor/position.cpp.s

CMakeFiles/quadrotor.dir/src/quadrotor/position.cpp.o.requires:
.PHONY : CMakeFiles/quadrotor.dir/src/quadrotor/position.cpp.o.requires

CMakeFiles/quadrotor.dir/src/quadrotor/position.cpp.o.provides: CMakeFiles/quadrotor.dir/src/quadrotor/position.cpp.o.requires
	$(MAKE) -f CMakeFiles/quadrotor.dir/build.make CMakeFiles/quadrotor.dir/src/quadrotor/position.cpp.o.provides.build
.PHONY : CMakeFiles/quadrotor.dir/src/quadrotor/position.cpp.o.provides

CMakeFiles/quadrotor.dir/src/quadrotor/position.cpp.o.provides.build: CMakeFiles/quadrotor.dir/src/quadrotor/position.cpp.o

CMakeFiles/quadrotor.dir/src/quadrotor/quadrotor.cpp.o: CMakeFiles/quadrotor.dir/flags.make
CMakeFiles/quadrotor.dir/src/quadrotor/quadrotor.cpp.o: src/quadrotor/quadrotor.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor/CMakeFiles $(CMAKE_PROGRESS_8)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/quadrotor.dir/src/quadrotor/quadrotor.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/quadrotor.dir/src/quadrotor/quadrotor.cpp.o -c /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor/src/quadrotor/quadrotor.cpp

CMakeFiles/quadrotor.dir/src/quadrotor/quadrotor.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/quadrotor.dir/src/quadrotor/quadrotor.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor/src/quadrotor/quadrotor.cpp > CMakeFiles/quadrotor.dir/src/quadrotor/quadrotor.cpp.i

CMakeFiles/quadrotor.dir/src/quadrotor/quadrotor.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/quadrotor.dir/src/quadrotor/quadrotor.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor/src/quadrotor/quadrotor.cpp -o CMakeFiles/quadrotor.dir/src/quadrotor/quadrotor.cpp.s

CMakeFiles/quadrotor.dir/src/quadrotor/quadrotor.cpp.o.requires:
.PHONY : CMakeFiles/quadrotor.dir/src/quadrotor/quadrotor.cpp.o.requires

CMakeFiles/quadrotor.dir/src/quadrotor/quadrotor.cpp.o.provides: CMakeFiles/quadrotor.dir/src/quadrotor/quadrotor.cpp.o.requires
	$(MAKE) -f CMakeFiles/quadrotor.dir/build.make CMakeFiles/quadrotor.dir/src/quadrotor/quadrotor.cpp.o.provides.build
.PHONY : CMakeFiles/quadrotor.dir/src/quadrotor/quadrotor.cpp.o.provides

CMakeFiles/quadrotor.dir/src/quadrotor/quadrotor.cpp.o.provides.build: CMakeFiles/quadrotor.dir/src/quadrotor/quadrotor.cpp.o

CMakeFiles/quadrotor.dir/src/quadrotor/telem.cpp.o: CMakeFiles/quadrotor.dir/flags.make
CMakeFiles/quadrotor.dir/src/quadrotor/telem.cpp.o: src/quadrotor/telem.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor/CMakeFiles $(CMAKE_PROGRESS_9)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/quadrotor.dir/src/quadrotor/telem.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/quadrotor.dir/src/quadrotor/telem.cpp.o -c /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor/src/quadrotor/telem.cpp

CMakeFiles/quadrotor.dir/src/quadrotor/telem.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/quadrotor.dir/src/quadrotor/telem.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor/src/quadrotor/telem.cpp > CMakeFiles/quadrotor.dir/src/quadrotor/telem.cpp.i

CMakeFiles/quadrotor.dir/src/quadrotor/telem.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/quadrotor.dir/src/quadrotor/telem.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor/src/quadrotor/telem.cpp -o CMakeFiles/quadrotor.dir/src/quadrotor/telem.cpp.s

CMakeFiles/quadrotor.dir/src/quadrotor/telem.cpp.o.requires:
.PHONY : CMakeFiles/quadrotor.dir/src/quadrotor/telem.cpp.o.requires

CMakeFiles/quadrotor.dir/src/quadrotor/telem.cpp.o.provides: CMakeFiles/quadrotor.dir/src/quadrotor/telem.cpp.o.requires
	$(MAKE) -f CMakeFiles/quadrotor.dir/build.make CMakeFiles/quadrotor.dir/src/quadrotor/telem.cpp.o.provides.build
.PHONY : CMakeFiles/quadrotor.dir/src/quadrotor/telem.cpp.o.provides

CMakeFiles/quadrotor.dir/src/quadrotor/telem.cpp.o.provides.build: CMakeFiles/quadrotor.dir/src/quadrotor/telem.cpp.o

CMakeFiles/quadrotor.dir/src/quadrotor/fda.cpp.o: CMakeFiles/quadrotor.dir/flags.make
CMakeFiles/quadrotor.dir/src/quadrotor/fda.cpp.o: src/quadrotor/fda.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor/CMakeFiles $(CMAKE_PROGRESS_10)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/quadrotor.dir/src/quadrotor/fda.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/quadrotor.dir/src/quadrotor/fda.cpp.o -c /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor/src/quadrotor/fda.cpp

CMakeFiles/quadrotor.dir/src/quadrotor/fda.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/quadrotor.dir/src/quadrotor/fda.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor/src/quadrotor/fda.cpp > CMakeFiles/quadrotor.dir/src/quadrotor/fda.cpp.i

CMakeFiles/quadrotor.dir/src/quadrotor/fda.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/quadrotor.dir/src/quadrotor/fda.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor/src/quadrotor/fda.cpp -o CMakeFiles/quadrotor.dir/src/quadrotor/fda.cpp.s

CMakeFiles/quadrotor.dir/src/quadrotor/fda.cpp.o.requires:
.PHONY : CMakeFiles/quadrotor.dir/src/quadrotor/fda.cpp.o.requires

CMakeFiles/quadrotor.dir/src/quadrotor/fda.cpp.o.provides: CMakeFiles/quadrotor.dir/src/quadrotor/fda.cpp.o.requires
	$(MAKE) -f CMakeFiles/quadrotor.dir/build.make CMakeFiles/quadrotor.dir/src/quadrotor/fda.cpp.o.provides.build
.PHONY : CMakeFiles/quadrotor.dir/src/quadrotor/fda.cpp.o.provides

CMakeFiles/quadrotor.dir/src/quadrotor/fda.cpp.o.provides.build: CMakeFiles/quadrotor.dir/src/quadrotor/fda.cpp.o

CMakeFiles/quadrotor.dir/src/quadrotor/command/Stop.cpp.o: CMakeFiles/quadrotor.dir/flags.make
CMakeFiles/quadrotor.dir/src/quadrotor/command/Stop.cpp.o: src/quadrotor/command/Stop.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor/CMakeFiles $(CMAKE_PROGRESS_11)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/quadrotor.dir/src/quadrotor/command/Stop.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/quadrotor.dir/src/quadrotor/command/Stop.cpp.o -c /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor/src/quadrotor/command/Stop.cpp

CMakeFiles/quadrotor.dir/src/quadrotor/command/Stop.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/quadrotor.dir/src/quadrotor/command/Stop.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor/src/quadrotor/command/Stop.cpp > CMakeFiles/quadrotor.dir/src/quadrotor/command/Stop.cpp.i

CMakeFiles/quadrotor.dir/src/quadrotor/command/Stop.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/quadrotor.dir/src/quadrotor/command/Stop.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor/src/quadrotor/command/Stop.cpp -o CMakeFiles/quadrotor.dir/src/quadrotor/command/Stop.cpp.s

CMakeFiles/quadrotor.dir/src/quadrotor/command/Stop.cpp.o.requires:
.PHONY : CMakeFiles/quadrotor.dir/src/quadrotor/command/Stop.cpp.o.requires

CMakeFiles/quadrotor.dir/src/quadrotor/command/Stop.cpp.o.provides: CMakeFiles/quadrotor.dir/src/quadrotor/command/Stop.cpp.o.requires
	$(MAKE) -f CMakeFiles/quadrotor.dir/build.make CMakeFiles/quadrotor.dir/src/quadrotor/command/Stop.cpp.o.provides.build
.PHONY : CMakeFiles/quadrotor.dir/src/quadrotor/command/Stop.cpp.o.provides

CMakeFiles/quadrotor.dir/src/quadrotor/command/Stop.cpp.o.provides.build: CMakeFiles/quadrotor.dir/src/quadrotor/command/Stop.cpp.o

CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/ControlLaw.cpp.o: CMakeFiles/quadrotor.dir/flags.make
CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/ControlLaw.cpp.o: src/quadrotor/ControlLaw/ControlLaw.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor/CMakeFiles $(CMAKE_PROGRESS_12)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/ControlLaw.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/ControlLaw.cpp.o -c /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor/src/quadrotor/ControlLaw/ControlLaw.cpp

CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/ControlLaw.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/ControlLaw.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor/src/quadrotor/ControlLaw/ControlLaw.cpp > CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/ControlLaw.cpp.i

CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/ControlLaw.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/ControlLaw.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor/src/quadrotor/ControlLaw/ControlLaw.cpp -o CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/ControlLaw.cpp.s

CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/ControlLaw.cpp.o.requires:
.PHONY : CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/ControlLaw.cpp.o.requires

CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/ControlLaw.cpp.o.provides: CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/ControlLaw.cpp.o.requires
	$(MAKE) -f CMakeFiles/quadrotor.dir/build.make CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/ControlLaw.cpp.o.provides.build
.PHONY : CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/ControlLaw.cpp.o.provides

CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/ControlLaw.cpp.o.provides.build: CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/ControlLaw.cpp.o

CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/Alpha.cpp.o: CMakeFiles/quadrotor.dir/flags.make
CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/Alpha.cpp.o: src/quadrotor/ControlLaw/Alpha.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor/CMakeFiles $(CMAKE_PROGRESS_13)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/Alpha.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/Alpha.cpp.o -c /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor/src/quadrotor/ControlLaw/Alpha.cpp

CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/Alpha.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/Alpha.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor/src/quadrotor/ControlLaw/Alpha.cpp > CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/Alpha.cpp.i

CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/Alpha.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/Alpha.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor/src/quadrotor/ControlLaw/Alpha.cpp -o CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/Alpha.cpp.s

CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/Alpha.cpp.o.requires:
.PHONY : CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/Alpha.cpp.o.requires

CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/Alpha.cpp.o.provides: CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/Alpha.cpp.o.requires
	$(MAKE) -f CMakeFiles/quadrotor.dir/build.make CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/Alpha.cpp.o.provides.build
.PHONY : CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/Alpha.cpp.o.provides

CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/Alpha.cpp.o.provides.build: CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/Alpha.cpp.o

CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/Jerk.cpp.o: CMakeFiles/quadrotor.dir/flags.make
CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/Jerk.cpp.o: src/quadrotor/ControlLaw/Jerk.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor/CMakeFiles $(CMAKE_PROGRESS_14)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/Jerk.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/Jerk.cpp.o -c /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor/src/quadrotor/ControlLaw/Jerk.cpp

CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/Jerk.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/Jerk.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor/src/quadrotor/ControlLaw/Jerk.cpp > CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/Jerk.cpp.i

CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/Jerk.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/Jerk.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor/src/quadrotor/ControlLaw/Jerk.cpp -o CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/Jerk.cpp.s

CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/Jerk.cpp.o.requires:
.PHONY : CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/Jerk.cpp.o.requires

CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/Jerk.cpp.o.provides: CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/Jerk.cpp.o.requires
	$(MAKE) -f CMakeFiles/quadrotor.dir/build.make CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/Jerk.cpp.o.provides.build
.PHONY : CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/Jerk.cpp.o.provides

CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/Jerk.cpp.o.provides.build: CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/Jerk.cpp.o

CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/Jounce.cpp.o: CMakeFiles/quadrotor.dir/flags.make
CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/Jounce.cpp.o: src/quadrotor/ControlLaw/Jounce.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor/CMakeFiles $(CMAKE_PROGRESS_15)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/Jounce.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/Jounce.cpp.o -c /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor/src/quadrotor/ControlLaw/Jounce.cpp

CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/Jounce.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/Jounce.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor/src/quadrotor/ControlLaw/Jounce.cpp > CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/Jounce.cpp.i

CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/Jounce.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/Jounce.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor/src/quadrotor/ControlLaw/Jounce.cpp -o CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/Jounce.cpp.s

CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/Jounce.cpp.o.requires:
.PHONY : CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/Jounce.cpp.o.requires

CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/Jounce.cpp.o.provides: CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/Jounce.cpp.o.requires
	$(MAKE) -f CMakeFiles/quadrotor.dir/build.make CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/Jounce.cpp.o.provides.build
.PHONY : CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/Jounce.cpp.o.provides

CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/Jounce.cpp.o.provides.build: CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/Jounce.cpp.o

# Object files for target quadrotor
quadrotor_OBJECTS = \
"CMakeFiles/quadrotor.dir/src/quadrotor/array.cpp.o" \
"CMakeFiles/quadrotor.dir/src/quadrotor/attitude.cpp.o" \
"CMakeFiles/quadrotor.dir/src/quadrotor/brain.cpp.o" \
"CMakeFiles/quadrotor.dir/src/quadrotor/command.cpp.o" \
"CMakeFiles/quadrotor.dir/src/quadrotor/Input.cpp.o" \
"CMakeFiles/quadrotor.dir/src/quadrotor/plant.cpp.o" \
"CMakeFiles/quadrotor.dir/src/quadrotor/position.cpp.o" \
"CMakeFiles/quadrotor.dir/src/quadrotor/quadrotor.cpp.o" \
"CMakeFiles/quadrotor.dir/src/quadrotor/telem.cpp.o" \
"CMakeFiles/quadrotor.dir/src/quadrotor/fda.cpp.o" \
"CMakeFiles/quadrotor.dir/src/quadrotor/command/Stop.cpp.o" \
"CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/ControlLaw.cpp.o" \
"CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/Alpha.cpp.o" \
"CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/Jerk.cpp.o" \
"CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/Jounce.cpp.o"

# External object files for target quadrotor
quadrotor_EXTERNAL_OBJECTS =

libquadrotor.a: CMakeFiles/quadrotor.dir/src/quadrotor/array.cpp.o
libquadrotor.a: CMakeFiles/quadrotor.dir/src/quadrotor/attitude.cpp.o
libquadrotor.a: CMakeFiles/quadrotor.dir/src/quadrotor/brain.cpp.o
libquadrotor.a: CMakeFiles/quadrotor.dir/src/quadrotor/command.cpp.o
libquadrotor.a: CMakeFiles/quadrotor.dir/src/quadrotor/Input.cpp.o
libquadrotor.a: CMakeFiles/quadrotor.dir/src/quadrotor/plant.cpp.o
libquadrotor.a: CMakeFiles/quadrotor.dir/src/quadrotor/position.cpp.o
libquadrotor.a: CMakeFiles/quadrotor.dir/src/quadrotor/quadrotor.cpp.o
libquadrotor.a: CMakeFiles/quadrotor.dir/src/quadrotor/telem.cpp.o
libquadrotor.a: CMakeFiles/quadrotor.dir/src/quadrotor/fda.cpp.o
libquadrotor.a: CMakeFiles/quadrotor.dir/src/quadrotor/command/Stop.cpp.o
libquadrotor.a: CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/ControlLaw.cpp.o
libquadrotor.a: CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/Alpha.cpp.o
libquadrotor.a: CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/Jerk.cpp.o
libquadrotor.a: CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/Jounce.cpp.o
libquadrotor.a: CMakeFiles/quadrotor.dir/build.make
libquadrotor.a: CMakeFiles/quadrotor.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX static library libquadrotor.a"
	$(CMAKE_COMMAND) -P CMakeFiles/quadrotor.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/quadrotor.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/quadrotor.dir/build: libquadrotor.a
.PHONY : CMakeFiles/quadrotor.dir/build

CMakeFiles/quadrotor.dir/requires: CMakeFiles/quadrotor.dir/src/quadrotor/array.cpp.o.requires
CMakeFiles/quadrotor.dir/requires: CMakeFiles/quadrotor.dir/src/quadrotor/attitude.cpp.o.requires
CMakeFiles/quadrotor.dir/requires: CMakeFiles/quadrotor.dir/src/quadrotor/brain.cpp.o.requires
CMakeFiles/quadrotor.dir/requires: CMakeFiles/quadrotor.dir/src/quadrotor/command.cpp.o.requires
CMakeFiles/quadrotor.dir/requires: CMakeFiles/quadrotor.dir/src/quadrotor/Input.cpp.o.requires
CMakeFiles/quadrotor.dir/requires: CMakeFiles/quadrotor.dir/src/quadrotor/plant.cpp.o.requires
CMakeFiles/quadrotor.dir/requires: CMakeFiles/quadrotor.dir/src/quadrotor/position.cpp.o.requires
CMakeFiles/quadrotor.dir/requires: CMakeFiles/quadrotor.dir/src/quadrotor/quadrotor.cpp.o.requires
CMakeFiles/quadrotor.dir/requires: CMakeFiles/quadrotor.dir/src/quadrotor/telem.cpp.o.requires
CMakeFiles/quadrotor.dir/requires: CMakeFiles/quadrotor.dir/src/quadrotor/fda.cpp.o.requires
CMakeFiles/quadrotor.dir/requires: CMakeFiles/quadrotor.dir/src/quadrotor/command/Stop.cpp.o.requires
CMakeFiles/quadrotor.dir/requires: CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/ControlLaw.cpp.o.requires
CMakeFiles/quadrotor.dir/requires: CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/Alpha.cpp.o.requires
CMakeFiles/quadrotor.dir/requires: CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/Jerk.cpp.o.requires
CMakeFiles/quadrotor.dir/requires: CMakeFiles/quadrotor.dir/src/quadrotor/ControlLaw/Jounce.cpp.o.requires
.PHONY : CMakeFiles/quadrotor.dir/requires

CMakeFiles/quadrotor.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/quadrotor.dir/cmake_clean.cmake
.PHONY : CMakeFiles/quadrotor.dir/clean

CMakeFiles/quadrotor.dir/depend:
	cd /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor /nfs/stak/students/r/rymalc/Documents/Programming/C++/quadrotor/c/quadrotor/CMakeFiles/quadrotor.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/quadrotor.dir/depend

