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
include CMakeFiles/image_registration2d.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/image_registration2d.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/image_registration2d.dir/flags.make

CMakeFiles/image_registration2d.dir/image_registration2d.cpp.o: CMakeFiles/image_registration2d.dir/flags.make
CMakeFiles/image_registration2d.dir/image_registration2d.cpp.o: ../image_registration2d.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/andrea/workspaceC/reconstructionEvaluator/external/CImg/examples/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/image_registration2d.dir/image_registration2d.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/image_registration2d.dir/image_registration2d.cpp.o -c /home/andrea/workspaceC/reconstructionEvaluator/external/CImg/examples/image_registration2d.cpp

CMakeFiles/image_registration2d.dir/image_registration2d.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/image_registration2d.dir/image_registration2d.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/andrea/workspaceC/reconstructionEvaluator/external/CImg/examples/image_registration2d.cpp > CMakeFiles/image_registration2d.dir/image_registration2d.cpp.i

CMakeFiles/image_registration2d.dir/image_registration2d.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/image_registration2d.dir/image_registration2d.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/andrea/workspaceC/reconstructionEvaluator/external/CImg/examples/image_registration2d.cpp -o CMakeFiles/image_registration2d.dir/image_registration2d.cpp.s

CMakeFiles/image_registration2d.dir/image_registration2d.cpp.o.requires:

.PHONY : CMakeFiles/image_registration2d.dir/image_registration2d.cpp.o.requires

CMakeFiles/image_registration2d.dir/image_registration2d.cpp.o.provides: CMakeFiles/image_registration2d.dir/image_registration2d.cpp.o.requires
	$(MAKE) -f CMakeFiles/image_registration2d.dir/build.make CMakeFiles/image_registration2d.dir/image_registration2d.cpp.o.provides.build
.PHONY : CMakeFiles/image_registration2d.dir/image_registration2d.cpp.o.provides

CMakeFiles/image_registration2d.dir/image_registration2d.cpp.o.provides.build: CMakeFiles/image_registration2d.dir/image_registration2d.cpp.o


# Object files for target image_registration2d
image_registration2d_OBJECTS = \
"CMakeFiles/image_registration2d.dir/image_registration2d.cpp.o"

# External object files for target image_registration2d
image_registration2d_EXTERNAL_OBJECTS =

../image_registration2d: CMakeFiles/image_registration2d.dir/image_registration2d.cpp.o
../image_registration2d: CMakeFiles/image_registration2d.dir/build.make
../image_registration2d: /usr/lib/x86_64-linux-gnu/libtiff.so
../image_registration2d: /usr/lib/x86_64-linux-gnu/libjpeg.so
../image_registration2d: /usr/lib/x86_64-linux-gnu/libz.so
../image_registration2d: /usr/lib/x86_64-linux-gnu/libpng.so
../image_registration2d: /usr/lib/x86_64-linux-gnu/libz.so
../image_registration2d: /usr/lib/x86_64-linux-gnu/libSM.so
../image_registration2d: /usr/lib/x86_64-linux-gnu/libICE.so
../image_registration2d: /usr/lib/x86_64-linux-gnu/libX11.so
../image_registration2d: /usr/lib/x86_64-linux-gnu/libXext.so
../image_registration2d: /usr/local/lib/libopencv_viz.so.3.1.0
../image_registration2d: /usr/local/lib/libopencv_videostab.so.3.1.0
../image_registration2d: /usr/local/lib/libopencv_superres.so.3.1.0
../image_registration2d: /usr/local/lib/libopencv_stitching.so.3.1.0
../image_registration2d: /usr/local/lib/libopencv_shape.so.3.1.0
../image_registration2d: /usr/local/lib/libopencv_photo.so.3.1.0
../image_registration2d: /usr/local/lib/libopencv_objdetect.so.3.1.0
../image_registration2d: /usr/local/lib/libopencv_calib3d.so.3.1.0
../image_registration2d: /usr/lib/liblapack.so
../image_registration2d: /usr/lib/libblas.so
../image_registration2d: /usr/lib/libblas.so
../image_registration2d: /usr/lib/x86_64-linux-gnu/libpng.so
../image_registration2d: /usr/lib/x86_64-linux-gnu/libSM.so
../image_registration2d: /usr/lib/x86_64-linux-gnu/libICE.so
../image_registration2d: /usr/lib/x86_64-linux-gnu/libX11.so
../image_registration2d: /usr/lib/x86_64-linux-gnu/libXext.so
../image_registration2d: /usr/lib/liblapack.so
../image_registration2d: /usr/lib/libblas.so
../image_registration2d: /usr/local/lib/libopencv_features2d.so.3.1.0
../image_registration2d: /usr/local/lib/libopencv_ml.so.3.1.0
../image_registration2d: /usr/local/lib/libopencv_highgui.so.3.1.0
../image_registration2d: /usr/local/lib/libopencv_videoio.so.3.1.0
../image_registration2d: /usr/local/lib/libopencv_imgcodecs.so.3.1.0
../image_registration2d: /usr/local/lib/libopencv_flann.so.3.1.0
../image_registration2d: /usr/local/lib/libopencv_video.so.3.1.0
../image_registration2d: /usr/local/lib/libopencv_imgproc.so.3.1.0
../image_registration2d: /usr/local/lib/libopencv_core.so.3.1.0
../image_registration2d: CMakeFiles/image_registration2d.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/andrea/workspaceC/reconstructionEvaluator/external/CImg/examples/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../image_registration2d"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/image_registration2d.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/image_registration2d.dir/build: ../image_registration2d

.PHONY : CMakeFiles/image_registration2d.dir/build

CMakeFiles/image_registration2d.dir/requires: CMakeFiles/image_registration2d.dir/image_registration2d.cpp.o.requires

.PHONY : CMakeFiles/image_registration2d.dir/requires

CMakeFiles/image_registration2d.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/image_registration2d.dir/cmake_clean.cmake
.PHONY : CMakeFiles/image_registration2d.dir/clean

CMakeFiles/image_registration2d.dir/depend:
	cd /home/andrea/workspaceC/reconstructionEvaluator/external/CImg/examples/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/andrea/workspaceC/reconstructionEvaluator/external/CImg/examples /home/andrea/workspaceC/reconstructionEvaluator/external/CImg/examples /home/andrea/workspaceC/reconstructionEvaluator/external/CImg/examples/build /home/andrea/workspaceC/reconstructionEvaluator/external/CImg/examples/build /home/andrea/workspaceC/reconstructionEvaluator/external/CImg/examples/build/CMakeFiles/image_registration2d.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/image_registration2d.dir/depend

