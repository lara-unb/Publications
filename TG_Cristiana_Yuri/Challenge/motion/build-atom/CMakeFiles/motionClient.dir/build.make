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

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/cris/nao/workspace/UnBeatables2016/UnBeatables/Challenge/motion

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/cris/nao/workspace/UnBeatables2016/UnBeatables/Challenge/motion/build-atom

# Include any dependencies generated for this target.
include CMakeFiles/motionClient.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/motionClient.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/motionClient.dir/flags.make

CMakeFiles/motionClient.dir/motionClient.cpp.o: CMakeFiles/motionClient.dir/flags.make
CMakeFiles/motionClient.dir/motionClient.cpp.o: ../motionClient.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/cris/nao/workspace/UnBeatables2016/UnBeatables/Challenge/motion/build-atom/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/motionClient.dir/motionClient.cpp.o"
	/home/cris/nao/devtools/ctc-linux64-atom-2.1.4.13/cross/bin/i686-aldebaran-linux-gnu-g++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/motionClient.dir/motionClient.cpp.o -c /home/cris/nao/workspace/UnBeatables2016/UnBeatables/Challenge/motion/motionClient.cpp

CMakeFiles/motionClient.dir/motionClient.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/motionClient.dir/motionClient.cpp.i"
	/home/cris/nao/devtools/ctc-linux64-atom-2.1.4.13/cross/bin/i686-aldebaran-linux-gnu-g++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/cris/nao/workspace/UnBeatables2016/UnBeatables/Challenge/motion/motionClient.cpp > CMakeFiles/motionClient.dir/motionClient.cpp.i

CMakeFiles/motionClient.dir/motionClient.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/motionClient.dir/motionClient.cpp.s"
	/home/cris/nao/devtools/ctc-linux64-atom-2.1.4.13/cross/bin/i686-aldebaran-linux-gnu-g++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/cris/nao/workspace/UnBeatables2016/UnBeatables/Challenge/motion/motionClient.cpp -o CMakeFiles/motionClient.dir/motionClient.cpp.s

CMakeFiles/motionClient.dir/motionClient.cpp.o.requires:
.PHONY : CMakeFiles/motionClient.dir/motionClient.cpp.o.requires

CMakeFiles/motionClient.dir/motionClient.cpp.o.provides: CMakeFiles/motionClient.dir/motionClient.cpp.o.requires
	$(MAKE) -f CMakeFiles/motionClient.dir/build.make CMakeFiles/motionClient.dir/motionClient.cpp.o.provides.build
.PHONY : CMakeFiles/motionClient.dir/motionClient.cpp.o.provides

CMakeFiles/motionClient.dir/motionClient.cpp.o.provides.build: CMakeFiles/motionClient.dir/motionClient.cpp.o

CMakeFiles/motionClient.dir/runswift/perception/kinematics/SonarFilter.cpp.o: CMakeFiles/motionClient.dir/flags.make
CMakeFiles/motionClient.dir/runswift/perception/kinematics/SonarFilter.cpp.o: ../runswift/perception/kinematics/SonarFilter.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/cris/nao/workspace/UnBeatables2016/UnBeatables/Challenge/motion/build-atom/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/motionClient.dir/runswift/perception/kinematics/SonarFilter.cpp.o"
	/home/cris/nao/devtools/ctc-linux64-atom-2.1.4.13/cross/bin/i686-aldebaran-linux-gnu-g++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/motionClient.dir/runswift/perception/kinematics/SonarFilter.cpp.o -c /home/cris/nao/workspace/UnBeatables2016/UnBeatables/Challenge/motion/runswift/perception/kinematics/SonarFilter.cpp

CMakeFiles/motionClient.dir/runswift/perception/kinematics/SonarFilter.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/motionClient.dir/runswift/perception/kinematics/SonarFilter.cpp.i"
	/home/cris/nao/devtools/ctc-linux64-atom-2.1.4.13/cross/bin/i686-aldebaran-linux-gnu-g++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/cris/nao/workspace/UnBeatables2016/UnBeatables/Challenge/motion/runswift/perception/kinematics/SonarFilter.cpp > CMakeFiles/motionClient.dir/runswift/perception/kinematics/SonarFilter.cpp.i

CMakeFiles/motionClient.dir/runswift/perception/kinematics/SonarFilter.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/motionClient.dir/runswift/perception/kinematics/SonarFilter.cpp.s"
	/home/cris/nao/devtools/ctc-linux64-atom-2.1.4.13/cross/bin/i686-aldebaran-linux-gnu-g++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/cris/nao/workspace/UnBeatables2016/UnBeatables/Challenge/motion/runswift/perception/kinematics/SonarFilter.cpp -o CMakeFiles/motionClient.dir/runswift/perception/kinematics/SonarFilter.cpp.s

CMakeFiles/motionClient.dir/runswift/perception/kinematics/SonarFilter.cpp.o.requires:
.PHONY : CMakeFiles/motionClient.dir/runswift/perception/kinematics/SonarFilter.cpp.o.requires

CMakeFiles/motionClient.dir/runswift/perception/kinematics/SonarFilter.cpp.o.provides: CMakeFiles/motionClient.dir/runswift/perception/kinematics/SonarFilter.cpp.o.requires
	$(MAKE) -f CMakeFiles/motionClient.dir/build.make CMakeFiles/motionClient.dir/runswift/perception/kinematics/SonarFilter.cpp.o.provides.build
.PHONY : CMakeFiles/motionClient.dir/runswift/perception/kinematics/SonarFilter.cpp.o.provides

CMakeFiles/motionClient.dir/runswift/perception/kinematics/SonarFilter.cpp.o.provides.build: CMakeFiles/motionClient.dir/runswift/perception/kinematics/SonarFilter.cpp.o

CMakeFiles/motionClient.dir/home/cris/nao/workspace/UnBeatables2016/UnBeatables/Challenge/common/motionData.cpp.o: CMakeFiles/motionClient.dir/flags.make
CMakeFiles/motionClient.dir/home/cris/nao/workspace/UnBeatables2016/UnBeatables/Challenge/common/motionData.cpp.o: /home/cris/nao/workspace/UnBeatables2016/UnBeatables/Challenge/common/motionData.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/cris/nao/workspace/UnBeatables2016/UnBeatables/Challenge/motion/build-atom/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/motionClient.dir/home/cris/nao/workspace/UnBeatables2016/UnBeatables/Challenge/common/motionData.cpp.o"
	/home/cris/nao/devtools/ctc-linux64-atom-2.1.4.13/cross/bin/i686-aldebaran-linux-gnu-g++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/motionClient.dir/home/cris/nao/workspace/UnBeatables2016/UnBeatables/Challenge/common/motionData.cpp.o -c /home/cris/nao/workspace/UnBeatables2016/UnBeatables/Challenge/common/motionData.cpp

CMakeFiles/motionClient.dir/home/cris/nao/workspace/UnBeatables2016/UnBeatables/Challenge/common/motionData.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/motionClient.dir/home/cris/nao/workspace/UnBeatables2016/UnBeatables/Challenge/common/motionData.cpp.i"
	/home/cris/nao/devtools/ctc-linux64-atom-2.1.4.13/cross/bin/i686-aldebaran-linux-gnu-g++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/cris/nao/workspace/UnBeatables2016/UnBeatables/Challenge/common/motionData.cpp > CMakeFiles/motionClient.dir/home/cris/nao/workspace/UnBeatables2016/UnBeatables/Challenge/common/motionData.cpp.i

CMakeFiles/motionClient.dir/home/cris/nao/workspace/UnBeatables2016/UnBeatables/Challenge/common/motionData.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/motionClient.dir/home/cris/nao/workspace/UnBeatables2016/UnBeatables/Challenge/common/motionData.cpp.s"
	/home/cris/nao/devtools/ctc-linux64-atom-2.1.4.13/cross/bin/i686-aldebaran-linux-gnu-g++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/cris/nao/workspace/UnBeatables2016/UnBeatables/Challenge/common/motionData.cpp -o CMakeFiles/motionClient.dir/home/cris/nao/workspace/UnBeatables2016/UnBeatables/Challenge/common/motionData.cpp.s

CMakeFiles/motionClient.dir/home/cris/nao/workspace/UnBeatables2016/UnBeatables/Challenge/common/motionData.cpp.o.requires:
.PHONY : CMakeFiles/motionClient.dir/home/cris/nao/workspace/UnBeatables2016/UnBeatables/Challenge/common/motionData.cpp.o.requires

CMakeFiles/motionClient.dir/home/cris/nao/workspace/UnBeatables2016/UnBeatables/Challenge/common/motionData.cpp.o.provides: CMakeFiles/motionClient.dir/home/cris/nao/workspace/UnBeatables2016/UnBeatables/Challenge/common/motionData.cpp.o.requires
	$(MAKE) -f CMakeFiles/motionClient.dir/build.make CMakeFiles/motionClient.dir/home/cris/nao/workspace/UnBeatables2016/UnBeatables/Challenge/common/motionData.cpp.o.provides.build
.PHONY : CMakeFiles/motionClient.dir/home/cris/nao/workspace/UnBeatables2016/UnBeatables/Challenge/common/motionData.cpp.o.provides

CMakeFiles/motionClient.dir/home/cris/nao/workspace/UnBeatables2016/UnBeatables/Challenge/common/motionData.cpp.o.provides.build: CMakeFiles/motionClient.dir/home/cris/nao/workspace/UnBeatables2016/UnBeatables/Challenge/common/motionData.cpp.o

# Object files for target motionClient
motionClient_OBJECTS = \
"CMakeFiles/motionClient.dir/motionClient.cpp.o" \
"CMakeFiles/motionClient.dir/runswift/perception/kinematics/SonarFilter.cpp.o" \
"CMakeFiles/motionClient.dir/home/cris/nao/workspace/UnBeatables2016/UnBeatables/Challenge/common/motionData.cpp.o"

# External object files for target motionClient
motionClient_EXTERNAL_OBJECTS =

sdk/bin/motionClient: CMakeFiles/motionClient.dir/motionClient.cpp.o
sdk/bin/motionClient: CMakeFiles/motionClient.dir/runswift/perception/kinematics/SonarFilter.cpp.o
sdk/bin/motionClient: CMakeFiles/motionClient.dir/home/cris/nao/workspace/UnBeatables2016/UnBeatables/Challenge/common/motionData.cpp.o
sdk/bin/motionClient: CMakeFiles/motionClient.dir/build.make
sdk/bin/motionClient: /home/cris/nao/devtools/ctc-linux64-atom-2.1.4.13/libnaoqi/lib/libalproxies.so
sdk/bin/motionClient: /home/cris/nao/devtools/ctc-linux64-atom-2.1.4.13/libnaoqi/lib/libalcommon.so
sdk/bin/motionClient: /home/cris/nao/devtools/ctc-linux64-atom-2.1.4.13/boost/lib/libboost_signals-mt-1_55.so
sdk/bin/motionClient: /home/cris/nao/devtools/ctc-linux64-atom-2.1.4.13/libnaoqi/lib/librttools.so
sdk/bin/motionClient: /home/cris/nao/devtools/ctc-linux64-atom-2.1.4.13/libnaoqi/lib/libalvalue.so
sdk/bin/motionClient: /home/cris/nao/devtools/ctc-linux64-atom-2.1.4.13/libnaoqi/lib/libalerror.so
sdk/bin/motionClient: /home/cris/nao/devtools/ctc-linux64-atom-2.1.4.13/libnaoqi/lib/libqimessaging.so
sdk/bin/motionClient: /home/cris/nao/devtools/ctc-linux64-atom-2.1.4.13/libnaoqi/lib/libqitype.so
sdk/bin/motionClient: /home/cris/nao/devtools/ctc-linux64-atom-2.1.4.13/libnaoqi/lib/libqi.so
sdk/bin/motionClient: /home/cris/nao/devtools/ctc-linux64-atom-2.1.4.13/boost/lib/libboost_chrono-mt-1_55.so
sdk/bin/motionClient: /home/cris/nao/devtools/ctc-linux64-atom-2.1.4.13/boost/lib/libboost_filesystem-mt-1_55.so
sdk/bin/motionClient: /home/cris/nao/devtools/ctc-linux64-atom-2.1.4.13/boost/lib/libboost_program_options-mt-1_55.so
sdk/bin/motionClient: /home/cris/nao/devtools/ctc-linux64-atom-2.1.4.13/boost/lib/libboost_regex-mt-1_55.so
sdk/bin/motionClient: /home/cris/nao/devtools/ctc-linux64-atom-2.1.4.13/cross/i686-aldebaran-linux-gnu/sysroot/usr/lib/libdl.so
sdk/bin/motionClient: /home/cris/nao/devtools/ctc-linux64-atom-2.1.4.13/cross/i686-aldebaran-linux-gnu/sysroot/usr/lib/librt.so
sdk/bin/motionClient: /home/cris/nao/devtools/ctc-linux64-atom-2.1.4.13/boost/lib/libboost_date_time-mt-1_55.so
sdk/bin/motionClient: /home/cris/nao/devtools/ctc-linux64-atom-2.1.4.13/boost/lib/libboost_system-mt-1_55.so
sdk/bin/motionClient: /home/cris/nao/devtools/ctc-linux64-atom-2.1.4.13/boost/lib/libboost_locale-mt-1_55.so
sdk/bin/motionClient: /home/cris/nao/devtools/ctc-linux64-atom-2.1.4.13/boost/lib/libboost_thread-mt-1_55.so
sdk/bin/motionClient: /home/cris/nao/devtools/ctc-linux64-atom-2.1.4.13/cross/i686-aldebaran-linux-gnu/sysroot/usr/lib/libpthread.so
sdk/bin/motionClient: CMakeFiles/motionClient.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable sdk/bin/motionClient"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/motionClient.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/motionClient.dir/build: sdk/bin/motionClient
.PHONY : CMakeFiles/motionClient.dir/build

CMakeFiles/motionClient.dir/requires: CMakeFiles/motionClient.dir/motionClient.cpp.o.requires
CMakeFiles/motionClient.dir/requires: CMakeFiles/motionClient.dir/runswift/perception/kinematics/SonarFilter.cpp.o.requires
CMakeFiles/motionClient.dir/requires: CMakeFiles/motionClient.dir/home/cris/nao/workspace/UnBeatables2016/UnBeatables/Challenge/common/motionData.cpp.o.requires
.PHONY : CMakeFiles/motionClient.dir/requires

CMakeFiles/motionClient.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/motionClient.dir/cmake_clean.cmake
.PHONY : CMakeFiles/motionClient.dir/clean

CMakeFiles/motionClient.dir/depend:
	cd /home/cris/nao/workspace/UnBeatables2016/UnBeatables/Challenge/motion/build-atom && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cris/nao/workspace/UnBeatables2016/UnBeatables/Challenge/motion /home/cris/nao/workspace/UnBeatables2016/UnBeatables/Challenge/motion /home/cris/nao/workspace/UnBeatables2016/UnBeatables/Challenge/motion/build-atom /home/cris/nao/workspace/UnBeatables2016/UnBeatables/Challenge/motion/build-atom /home/cris/nao/workspace/UnBeatables2016/UnBeatables/Challenge/motion/build-atom/CMakeFiles/motionClient.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/motionClient.dir/depend

