# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/andrea/workspaceC/reconstructionEvaluator/external/CImg/examples

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/andrea/workspaceC/reconstructionEvaluator/external/CImg/examples/build

# Include any dependencies generated for this target.
include CMakeFiles/jawbreaker.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/jawbreaker.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/jawbreaker.dir/flags.make

CMakeFiles/jawbreaker.dir/jawbreaker.cpp.o: CMakeFiles/jawbreaker.dir/flags.make
CMakeFiles/jawbreaker.dir/jawbreaker.cpp.o: ../jawbreaker.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/andrea/workspaceC/reconstructionEvaluator/external/CImg/examples/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/jawbreaker.dir/jawbreaker.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/jawbreaker.dir/jawbreaker.cpp.o -c /home/andrea/workspaceC/reconstructionEvaluator/external/CImg/examples/jawbreaker.cpp

CMakeFiles/jawbreaker.dir/jawbreaker.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/jawbreaker.dir/jawbreaker.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/andrea/workspaceC/reconstructionEvaluator/external/CImg/examples/jawbreaker.cpp > CMakeFiles/jawbreaker.dir/jawbreaker.cpp.i

CMakeFiles/jawbreaker.dir/jawbreaker.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/jawbreaker.dir/jawbreaker.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/andrea/workspaceC/reconstructionEvaluator/external/CImg/examples/jawbreaker.cpp -o CMakeFiles/jawbreaker.dir/jawbreaker.cpp.s

CMakeFiles/jawbreaker.dir/jawbreaker.cpp.o.requires:

.PHONY : CMakeFiles/jawbreaker.dir/jawbreaker.cpp.o.requires

CMakeFiles/jawbreaker.dir/jawbreaker.cpp.o.provides: CMakeFiles/jawbreaker.dir/jawbreaker.cpp.o.requires
	$(MAKE) -f CMakeFiles/jawbreaker.dir/build.make CMakeFiles/jawbreaker.dir/jawbreaker.cpp.o.provides.build
.PHONY : CMakeFiles/jawbreaker.dir/jawbreaker.cpp.o.provides

CMakeFiles/jawbreaker.dir/jawbreaker.cpp.o.provides.build: CMakeFiles/jawbreaker.dir/jawbreaker.cpp.o


# Object files for target jawbreaker
jawbreaker_OBJECTS = \
"CMakeFiles/jawbreaker.dir/jawbreaker.cpp.o"

# External object files for target jawbreaker
jawbreaker_EXTERNAL_OBJECTS =

../jawbreaker: CMakeFiles/jawbreaker.dir/jawbreaker.cpp.o
../jawbreaker: CMakeFiles/jawbreaker.dir/build.make
../jawbreaker: /usr/lib/x86_64-linux-gnu/libtiff.so
../jawbreaker: /usr/lib/x86_64-linux-gnu/libjpeg.so
../jawbreaker: /usr/lib/x86_64-linux-gnu/libz.so
../jawbreaker: /usr/lib/x86_64-linux-gnu/libpng.so
../jawbreaker: /usr/lib/x86_64-linux-gnu/libz.so
../jawbreaker: /usr/lib/x86_64-linux-gnu/libSM.so
../jawbreaker: /usr/lib/x86_64-linux-gnu/libICE.so
../jawbreaker: /usr/lib/x86_64-linux-gnu/libX11.so
../jawbreaker: /usr/lib/x86_64-linux-gnu/libXext.so
../jawbreaker: /usr/local/lib/libopencv_viz.so.3.1.0
../jawbreaker: /usr/local/lib/libopencv_videostab.so.3.1.0
../jawbreaker: /usr/local/lib/libopencv_superres.so.3.1.0
../jawbreaker: /usr/local/lib/libopencv_stitching.so.3.1.0
../jawbreaker: /usr/local/lib/libopencv_shape.so.3.1.0
../jawbreaker: /usr/local/lib/libopencv_photo.so.3.1.0
../jawbreaker: /usr/local/lib/libopencv_objdetect.so.3.1.0
../jawbreaker: /usr/local/lib/libopencv_calib3d.so.3.1.0
../jawbreaker: /usr/lib/liblapack.so
../jawbreaker: /usr/lib/libblas.so
../jawbreaker: /usr/lib/libblas.so
../jawbreaker: /usr/lib/x86_64-linux-gnu/libpng.so
../jawbreaker: /usr/lib/x86_64-linux-gnu/libSM.so
../jawbreaker: /usr/lib/x86_64-linux-gnu/libICE.so
../jawbreaker: /usr/lib/x86_64-linux-gnu/libX11.so
../jawbreaker: /usr/lib/x86_64-linux-gnu/libXext.so
../jawbreaker: /usr/lib/liblapack.so
../jawbreaker: /usr/lib/libblas.so
../jawbreaker: /usr/local/lib/libopencv_features2d.so.3.1.0
../jawbreaker: /usr/local/lib/libopencv_ml.so.3.1.0
../jawbreaker: /usr/local/lib/libopencv_highgui.so.3.1.0
../jawbreaker: /usr/local/lib/libopencv_videoio.so.3.1.0
../jawbreaker: /usr/local/lib/libopencv_imgcodecs.so.3.1.0
../jawbreaker: /usr/local/lib/libopencv_flann.so.3.1.0
../jawbreaker: /usr/local/lib/libopencv_video.so.3.1.0
../jawbreaker: /usr/local/lib/libopencv_imgproc.so.3.1.0
../jawbreaker: /usr/local/lib/libopencv_core.so.3.1.0
../jawbreaker: CMakeFiles/jawbreaker.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/andrea/workspaceC/reconstructionEvaluator/external/CImg/examples/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../jawbreaker"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/jawbreaker.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/jawbreaker.dir/build: ../jawbreaker

.PHONY : CMakeFiles/jawbreaker.dir/build

CMakeFiles/jawbreaker.dir/requires: CMakeFiles/jawbreaker.dir/jawbreaker.cpp.o.requires

.PHONY : CMakeFiles/jawbreaker.dir/requires

CMakeFiles/jawbreaker.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/jawbreaker.dir/cmake_clean.cmake
.PHONY : CMakeFiles/jawbreaker.dir/clean

CMakeFiles/jawbreaker.dir/depend:
	cd /home/andrea/workspaceC/reconstructionEvaluator/external/CImg/examples/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/andrea/workspaceC/reconstructionEvaluator/external/CImg/examples /home/andrea/workspaceC/reconstructionEvaluator/external/CImg/examples /home/andrea/workspaceC/reconstructionEvaluator/external/CImg/examples/build /home/andrea/workspaceC/reconstructionEvaluator/external/CImg/examples/build /home/andrea/workspaceC/reconstructionEvaluator/external/CImg/examples/build/CMakeFiles/jawbreaker.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/jawbreaker.dir/depend
