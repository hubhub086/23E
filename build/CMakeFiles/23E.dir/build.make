# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.23

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/k06/Desktop/23E

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/k06/Desktop/23E/build

# Include any dependencies generated for this target.
include CMakeFiles/23E.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/23E.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/23E.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/23E.dir/flags.make

CMakeFiles/23E.dir/main.cpp.o: CMakeFiles/23E.dir/flags.make
CMakeFiles/23E.dir/main.cpp.o: ../main.cpp
CMakeFiles/23E.dir/main.cpp.o: CMakeFiles/23E.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/k06/Desktop/23E/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/23E.dir/main.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/23E.dir/main.cpp.o -MF CMakeFiles/23E.dir/main.cpp.o.d -o CMakeFiles/23E.dir/main.cpp.o -c /home/k06/Desktop/23E/main.cpp

CMakeFiles/23E.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/23E.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/k06/Desktop/23E/main.cpp > CMakeFiles/23E.dir/main.cpp.i

CMakeFiles/23E.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/23E.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/k06/Desktop/23E/main.cpp -o CMakeFiles/23E.dir/main.cpp.s

CMakeFiles/23E.dir/uart.cpp.o: CMakeFiles/23E.dir/flags.make
CMakeFiles/23E.dir/uart.cpp.o: ../uart.cpp
CMakeFiles/23E.dir/uart.cpp.o: CMakeFiles/23E.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/k06/Desktop/23E/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/23E.dir/uart.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/23E.dir/uart.cpp.o -MF CMakeFiles/23E.dir/uart.cpp.o.d -o CMakeFiles/23E.dir/uart.cpp.o -c /home/k06/Desktop/23E/uart.cpp

CMakeFiles/23E.dir/uart.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/23E.dir/uart.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/k06/Desktop/23E/uart.cpp > CMakeFiles/23E.dir/uart.cpp.i

CMakeFiles/23E.dir/uart.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/23E.dir/uart.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/k06/Desktop/23E/uart.cpp -o CMakeFiles/23E.dir/uart.cpp.s

CMakeFiles/23E.dir/tft.cpp.o: CMakeFiles/23E.dir/flags.make
CMakeFiles/23E.dir/tft.cpp.o: ../tft.cpp
CMakeFiles/23E.dir/tft.cpp.o: CMakeFiles/23E.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/k06/Desktop/23E/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/23E.dir/tft.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/23E.dir/tft.cpp.o -MF CMakeFiles/23E.dir/tft.cpp.o.d -o CMakeFiles/23E.dir/tft.cpp.o -c /home/k06/Desktop/23E/tft.cpp

CMakeFiles/23E.dir/tft.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/23E.dir/tft.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/k06/Desktop/23E/tft.cpp > CMakeFiles/23E.dir/tft.cpp.i

CMakeFiles/23E.dir/tft.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/23E.dir/tft.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/k06/Desktop/23E/tft.cpp -o CMakeFiles/23E.dir/tft.cpp.s

# Object files for target 23E
23E_OBJECTS = \
"CMakeFiles/23E.dir/main.cpp.o" \
"CMakeFiles/23E.dir/uart.cpp.o" \
"CMakeFiles/23E.dir/tft.cpp.o"

# External object files for target 23E
23E_EXTERNAL_OBJECTS =

23E: CMakeFiles/23E.dir/main.cpp.o
23E: CMakeFiles/23E.dir/uart.cpp.o
23E: CMakeFiles/23E.dir/tft.cpp.o
23E: CMakeFiles/23E.dir/build.make
23E: /usr/local/lib/aarch64-linux-gnu/libopencv_gapi.so.4.5.5
23E: /usr/local/lib/aarch64-linux-gnu/libopencv_stitching.so.4.5.5
23E: /usr/local/lib/aarch64-linux-gnu/libopencv_alphamat.so.4.5.5
23E: /usr/local/lib/aarch64-linux-gnu/libopencv_aruco.so.4.5.5
23E: /usr/local/lib/aarch64-linux-gnu/libopencv_barcode.so.4.5.5
23E: /usr/local/lib/aarch64-linux-gnu/libopencv_bgsegm.so.4.5.5
23E: /usr/local/lib/aarch64-linux-gnu/libopencv_bioinspired.so.4.5.5
23E: /usr/local/lib/aarch64-linux-gnu/libopencv_ccalib.so.4.5.5
23E: /usr/local/lib/aarch64-linux-gnu/libopencv_cudabgsegm.so.4.5.5
23E: /usr/local/lib/aarch64-linux-gnu/libopencv_cudafeatures2d.so.4.5.5
23E: /usr/local/lib/aarch64-linux-gnu/libopencv_cudaobjdetect.so.4.5.5
23E: /usr/local/lib/aarch64-linux-gnu/libopencv_cudastereo.so.4.5.5
23E: /usr/local/lib/aarch64-linux-gnu/libopencv_cvv.so.4.5.5
23E: /usr/local/lib/aarch64-linux-gnu/libopencv_dnn_objdetect.so.4.5.5
23E: /usr/local/lib/aarch64-linux-gnu/libopencv_dnn_superres.so.4.5.5
23E: /usr/local/lib/aarch64-linux-gnu/libopencv_dpm.so.4.5.5
23E: /usr/local/lib/aarch64-linux-gnu/libopencv_face.so.4.5.5
23E: /usr/local/lib/aarch64-linux-gnu/libopencv_freetype.so.4.5.5
23E: /usr/local/lib/aarch64-linux-gnu/libopencv_fuzzy.so.4.5.5
23E: /usr/local/lib/aarch64-linux-gnu/libopencv_hfs.so.4.5.5
23E: /usr/local/lib/aarch64-linux-gnu/libopencv_img_hash.so.4.5.5
23E: /usr/local/lib/aarch64-linux-gnu/libopencv_intensity_transform.so.4.5.5
23E: /usr/local/lib/aarch64-linux-gnu/libopencv_line_descriptor.so.4.5.5
23E: /usr/local/lib/aarch64-linux-gnu/libopencv_mcc.so.4.5.5
23E: /usr/local/lib/aarch64-linux-gnu/libopencv_quality.so.4.5.5
23E: /usr/local/lib/aarch64-linux-gnu/libopencv_rapid.so.4.5.5
23E: /usr/local/lib/aarch64-linux-gnu/libopencv_reg.so.4.5.5
23E: /usr/local/lib/aarch64-linux-gnu/libopencv_rgbd.so.4.5.5
23E: /usr/local/lib/aarch64-linux-gnu/libopencv_saliency.so.4.5.5
23E: /usr/local/lib/aarch64-linux-gnu/libopencv_stereo.so.4.5.5
23E: /usr/local/lib/aarch64-linux-gnu/libopencv_structured_light.so.4.5.5
23E: /usr/local/lib/aarch64-linux-gnu/libopencv_superres.so.4.5.5
23E: /usr/local/lib/aarch64-linux-gnu/libopencv_surface_matching.so.4.5.5
23E: /usr/local/lib/aarch64-linux-gnu/libopencv_tracking.so.4.5.5
23E: /usr/local/lib/aarch64-linux-gnu/libopencv_videostab.so.4.5.5
23E: /usr/local/lib/aarch64-linux-gnu/libopencv_wechat_qrcode.so.4.5.5
23E: /usr/local/lib/aarch64-linux-gnu/libopencv_xfeatures2d.so.4.5.5
23E: /usr/local/lib/aarch64-linux-gnu/libopencv_xobjdetect.so.4.5.5
23E: /usr/local/lib/aarch64-linux-gnu/libopencv_xphoto.so.4.5.5
23E: /usr/local/lib/aarch64-linux-gnu/libJetsonGPIO.so.1.2.4
23E: /usr/local/lib/libspidev-lib++.a
23E: /usr/local/lib/aarch64-linux-gnu/libopencv_shape.so.4.5.5
23E: /usr/local/lib/aarch64-linux-gnu/libopencv_highgui.so.4.5.5
23E: /usr/local/lib/aarch64-linux-gnu/libopencv_datasets.so.4.5.5
23E: /usr/local/lib/aarch64-linux-gnu/libopencv_plot.so.4.5.5
23E: /usr/local/lib/aarch64-linux-gnu/libopencv_text.so.4.5.5
23E: /usr/local/lib/aarch64-linux-gnu/libopencv_ml.so.4.5.5
23E: /usr/local/lib/aarch64-linux-gnu/libopencv_phase_unwrapping.so.4.5.5
23E: /usr/local/lib/aarch64-linux-gnu/libopencv_cudacodec.so.4.5.5
23E: /usr/local/lib/aarch64-linux-gnu/libopencv_videoio.so.4.5.5
23E: /usr/local/lib/aarch64-linux-gnu/libopencv_cudaoptflow.so.4.5.5
23E: /usr/local/lib/aarch64-linux-gnu/libopencv_cudalegacy.so.4.5.5
23E: /usr/local/lib/aarch64-linux-gnu/libopencv_cudawarping.so.4.5.5
23E: /usr/local/lib/aarch64-linux-gnu/libopencv_optflow.so.4.5.5
23E: /usr/local/lib/aarch64-linux-gnu/libopencv_ximgproc.so.4.5.5
23E: /usr/local/lib/aarch64-linux-gnu/libopencv_video.so.4.5.5
23E: /usr/local/lib/aarch64-linux-gnu/libopencv_imgcodecs.so.4.5.5
23E: /usr/local/lib/aarch64-linux-gnu/libopencv_objdetect.so.4.5.5
23E: /usr/local/lib/aarch64-linux-gnu/libopencv_calib3d.so.4.5.5
23E: /usr/local/lib/aarch64-linux-gnu/libopencv_dnn.so.4.5.5
23E: /usr/local/lib/aarch64-linux-gnu/libopencv_features2d.so.4.5.5
23E: /usr/local/lib/aarch64-linux-gnu/libopencv_flann.so.4.5.5
23E: /usr/local/lib/aarch64-linux-gnu/libopencv_photo.so.4.5.5
23E: /usr/local/lib/aarch64-linux-gnu/libopencv_cudaimgproc.so.4.5.5
23E: /usr/local/lib/aarch64-linux-gnu/libopencv_cudafilters.so.4.5.5
23E: /usr/local/lib/aarch64-linux-gnu/libopencv_imgproc.so.4.5.5
23E: /usr/local/lib/aarch64-linux-gnu/libopencv_cudaarithm.so.4.5.5
23E: /usr/local/lib/aarch64-linux-gnu/libopencv_core.so.4.5.5
23E: /usr/local/lib/aarch64-linux-gnu/libopencv_cudev.so.4.5.5
23E: CMakeFiles/23E.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/k06/Desktop/23E/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable 23E"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/23E.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/23E.dir/build: 23E
.PHONY : CMakeFiles/23E.dir/build

CMakeFiles/23E.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/23E.dir/cmake_clean.cmake
.PHONY : CMakeFiles/23E.dir/clean

CMakeFiles/23E.dir/depend:
	cd /home/k06/Desktop/23E/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/k06/Desktop/23E /home/k06/Desktop/23E /home/k06/Desktop/23E/build /home/k06/Desktop/23E/build /home/k06/Desktop/23E/build/CMakeFiles/23E.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/23E.dir/depend

