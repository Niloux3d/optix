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
CMAKE_SOURCE_DIR = /home/saimo/wuyou/project/optix

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/saimo/wuyou/project/optix/build

# Include any dependencies generated for this target.
include CMakeFiles/test.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/test.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/test.dir/flags.make

CMakeFiles/test.dir/main.cpp.o: CMakeFiles/test.dir/flags.make
CMakeFiles/test.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/saimo/wuyou/project/optix/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/test.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test.dir/main.cpp.o -c /home/saimo/wuyou/project/optix/main.cpp

CMakeFiles/test.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/saimo/wuyou/project/optix/main.cpp > CMakeFiles/test.dir/main.cpp.i

CMakeFiles/test.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/saimo/wuyou/project/optix/main.cpp -o CMakeFiles/test.dir/main.cpp.s

CMakeFiles/test.dir/Lidar.cpp.o: CMakeFiles/test.dir/flags.make
CMakeFiles/test.dir/Lidar.cpp.o: ../Lidar.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/saimo/wuyou/project/optix/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/test.dir/Lidar.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test.dir/Lidar.cpp.o -c /home/saimo/wuyou/project/optix/Lidar.cpp

CMakeFiles/test.dir/Lidar.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test.dir/Lidar.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/saimo/wuyou/project/optix/Lidar.cpp > CMakeFiles/test.dir/Lidar.cpp.i

CMakeFiles/test.dir/Lidar.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test.dir/Lidar.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/saimo/wuyou/project/optix/Lidar.cpp -o CMakeFiles/test.dir/Lidar.cpp.s

CMakeFiles/test.dir/LidarData.cpp.o: CMakeFiles/test.dir/flags.make
CMakeFiles/test.dir/LidarData.cpp.o: ../LidarData.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/saimo/wuyou/project/optix/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/test.dir/LidarData.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test.dir/LidarData.cpp.o -c /home/saimo/wuyou/project/optix/LidarData.cpp

CMakeFiles/test.dir/LidarData.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test.dir/LidarData.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/saimo/wuyou/project/optix/LidarData.cpp > CMakeFiles/test.dir/LidarData.cpp.i

CMakeFiles/test.dir/LidarData.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test.dir/LidarData.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/saimo/wuyou/project/optix/LidarData.cpp -o CMakeFiles/test.dir/LidarData.cpp.s

CMakeFiles/test.dir/SensorData.cpp.o: CMakeFiles/test.dir/flags.make
CMakeFiles/test.dir/SensorData.cpp.o: ../SensorData.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/saimo/wuyou/project/optix/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/test.dir/SensorData.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test.dir/SensorData.cpp.o -c /home/saimo/wuyou/project/optix/SensorData.cpp

CMakeFiles/test.dir/SensorData.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test.dir/SensorData.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/saimo/wuyou/project/optix/SensorData.cpp > CMakeFiles/test.dir/SensorData.cpp.i

CMakeFiles/test.dir/SensorData.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test.dir/SensorData.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/saimo/wuyou/project/optix/SensorData.cpp -o CMakeFiles/test.dir/SensorData.cpp.s

CMakeFiles/test.dir/RayGeneratorClass.cpp.o: CMakeFiles/test.dir/flags.make
CMakeFiles/test.dir/RayGeneratorClass.cpp.o: ../RayGeneratorClass.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/saimo/wuyou/project/optix/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/test.dir/RayGeneratorClass.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test.dir/RayGeneratorClass.cpp.o -c /home/saimo/wuyou/project/optix/RayGeneratorClass.cpp

CMakeFiles/test.dir/RayGeneratorClass.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test.dir/RayGeneratorClass.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/saimo/wuyou/project/optix/RayGeneratorClass.cpp > CMakeFiles/test.dir/RayGeneratorClass.cpp.i

CMakeFiles/test.dir/RayGeneratorClass.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test.dir/RayGeneratorClass.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/saimo/wuyou/project/optix/RayGeneratorClass.cpp -o CMakeFiles/test.dir/RayGeneratorClass.cpp.s

CMakeFiles/test.dir/LidarRayGenerator.cpp.o: CMakeFiles/test.dir/flags.make
CMakeFiles/test.dir/LidarRayGenerator.cpp.o: ../LidarRayGenerator.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/saimo/wuyou/project/optix/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/test.dir/LidarRayGenerator.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test.dir/LidarRayGenerator.cpp.o -c /home/saimo/wuyou/project/optix/LidarRayGenerator.cpp

CMakeFiles/test.dir/LidarRayGenerator.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test.dir/LidarRayGenerator.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/saimo/wuyou/project/optix/LidarRayGenerator.cpp > CMakeFiles/test.dir/LidarRayGenerator.cpp.i

CMakeFiles/test.dir/LidarRayGenerator.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test.dir/LidarRayGenerator.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/saimo/wuyou/project/optix/LidarRayGenerator.cpp -o CMakeFiles/test.dir/LidarRayGenerator.cpp.s

# Object files for target test
test_OBJECTS = \
"CMakeFiles/test.dir/main.cpp.o" \
"CMakeFiles/test.dir/Lidar.cpp.o" \
"CMakeFiles/test.dir/LidarData.cpp.o" \
"CMakeFiles/test.dir/SensorData.cpp.o" \
"CMakeFiles/test.dir/RayGeneratorClass.cpp.o" \
"CMakeFiles/test.dir/LidarRayGenerator.cpp.o"

# External object files for target test
test_EXTERNAL_OBJECTS =

test: CMakeFiles/test.dir/main.cpp.o
test: CMakeFiles/test.dir/Lidar.cpp.o
test: CMakeFiles/test.dir/LidarData.cpp.o
test: CMakeFiles/test.dir/SensorData.cpp.o
test: CMakeFiles/test.dir/RayGeneratorClass.cpp.o
test: CMakeFiles/test.dir/LidarRayGenerator.cpp.o
test: CMakeFiles/test.dir/build.make
test: ../cuda/libcalculateLidarRay.a
test: /home/saimo/wuyou/build/lib/libglad.so
test: /home/saimo/wuyou/build/lib/libsutil_7_sdk.so
test: /usr/local/cuda/lib64/libcudart_static.a
test: /usr/lib/x86_64-linux-gnu/librt.so
test: CMakeFiles/test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/saimo/wuyou/project/optix/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Linking CXX executable test"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/test.dir/build: test

.PHONY : CMakeFiles/test.dir/build

CMakeFiles/test.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/test.dir/cmake_clean.cmake
.PHONY : CMakeFiles/test.dir/clean

CMakeFiles/test.dir/depend:
	cd /home/saimo/wuyou/project/optix/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/saimo/wuyou/project/optix /home/saimo/wuyou/project/optix /home/saimo/wuyou/project/optix/build /home/saimo/wuyou/project/optix/build /home/saimo/wuyou/project/optix/build/CMakeFiles/test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/test.dir/depend
