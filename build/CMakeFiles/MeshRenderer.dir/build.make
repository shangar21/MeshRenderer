# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

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
CMAKE_SOURCE_DIR = /home/shangar21/Documents/MeshRenderer

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/shangar21/Documents/MeshRenderer/build

# Include any dependencies generated for this target.
include CMakeFiles/MeshRenderer.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/MeshRenderer.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/MeshRenderer.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/MeshRenderer.dir/flags.make

CMakeFiles/MeshRenderer.dir/src/main.cpp.o: CMakeFiles/MeshRenderer.dir/flags.make
CMakeFiles/MeshRenderer.dir/src/main.cpp.o: ../src/main.cpp
CMakeFiles/MeshRenderer.dir/src/main.cpp.o: CMakeFiles/MeshRenderer.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/shangar21/Documents/MeshRenderer/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/MeshRenderer.dir/src/main.cpp.o"
	/usr/bin/g++-11 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/MeshRenderer.dir/src/main.cpp.o -MF CMakeFiles/MeshRenderer.dir/src/main.cpp.o.d -o CMakeFiles/MeshRenderer.dir/src/main.cpp.o -c /home/shangar21/Documents/MeshRenderer/src/main.cpp

CMakeFiles/MeshRenderer.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/MeshRenderer.dir/src/main.cpp.i"
	/usr/bin/g++-11 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/shangar21/Documents/MeshRenderer/src/main.cpp > CMakeFiles/MeshRenderer.dir/src/main.cpp.i

CMakeFiles/MeshRenderer.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/MeshRenderer.dir/src/main.cpp.s"
	/usr/bin/g++-11 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/shangar21/Documents/MeshRenderer/src/main.cpp -o CMakeFiles/MeshRenderer.dir/src/main.cpp.s

CMakeFiles/MeshRenderer.dir/src/Mesh.cpp.o: CMakeFiles/MeshRenderer.dir/flags.make
CMakeFiles/MeshRenderer.dir/src/Mesh.cpp.o: ../src/Mesh.cpp
CMakeFiles/MeshRenderer.dir/src/Mesh.cpp.o: CMakeFiles/MeshRenderer.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/shangar21/Documents/MeshRenderer/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/MeshRenderer.dir/src/Mesh.cpp.o"
	/usr/bin/g++-11 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/MeshRenderer.dir/src/Mesh.cpp.o -MF CMakeFiles/MeshRenderer.dir/src/Mesh.cpp.o.d -o CMakeFiles/MeshRenderer.dir/src/Mesh.cpp.o -c /home/shangar21/Documents/MeshRenderer/src/Mesh.cpp

CMakeFiles/MeshRenderer.dir/src/Mesh.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/MeshRenderer.dir/src/Mesh.cpp.i"
	/usr/bin/g++-11 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/shangar21/Documents/MeshRenderer/src/Mesh.cpp > CMakeFiles/MeshRenderer.dir/src/Mesh.cpp.i

CMakeFiles/MeshRenderer.dir/src/Mesh.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/MeshRenderer.dir/src/Mesh.cpp.s"
	/usr/bin/g++-11 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/shangar21/Documents/MeshRenderer/src/Mesh.cpp -o CMakeFiles/MeshRenderer.dir/src/Mesh.cpp.s

CMakeFiles/MeshRenderer.dir/src/Camera.cpp.o: CMakeFiles/MeshRenderer.dir/flags.make
CMakeFiles/MeshRenderer.dir/src/Camera.cpp.o: ../src/Camera.cpp
CMakeFiles/MeshRenderer.dir/src/Camera.cpp.o: CMakeFiles/MeshRenderer.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/shangar21/Documents/MeshRenderer/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/MeshRenderer.dir/src/Camera.cpp.o"
	/usr/bin/g++-11 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/MeshRenderer.dir/src/Camera.cpp.o -MF CMakeFiles/MeshRenderer.dir/src/Camera.cpp.o.d -o CMakeFiles/MeshRenderer.dir/src/Camera.cpp.o -c /home/shangar21/Documents/MeshRenderer/src/Camera.cpp

CMakeFiles/MeshRenderer.dir/src/Camera.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/MeshRenderer.dir/src/Camera.cpp.i"
	/usr/bin/g++-11 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/shangar21/Documents/MeshRenderer/src/Camera.cpp > CMakeFiles/MeshRenderer.dir/src/Camera.cpp.i

CMakeFiles/MeshRenderer.dir/src/Camera.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/MeshRenderer.dir/src/Camera.cpp.s"
	/usr/bin/g++-11 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/shangar21/Documents/MeshRenderer/src/Camera.cpp -o CMakeFiles/MeshRenderer.dir/src/Camera.cpp.s

CMakeFiles/MeshRenderer.dir/src/Renderer.cpp.o: CMakeFiles/MeshRenderer.dir/flags.make
CMakeFiles/MeshRenderer.dir/src/Renderer.cpp.o: ../src/Renderer.cpp
CMakeFiles/MeshRenderer.dir/src/Renderer.cpp.o: CMakeFiles/MeshRenderer.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/shangar21/Documents/MeshRenderer/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/MeshRenderer.dir/src/Renderer.cpp.o"
	/usr/bin/g++-11 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/MeshRenderer.dir/src/Renderer.cpp.o -MF CMakeFiles/MeshRenderer.dir/src/Renderer.cpp.o.d -o CMakeFiles/MeshRenderer.dir/src/Renderer.cpp.o -c /home/shangar21/Documents/MeshRenderer/src/Renderer.cpp

CMakeFiles/MeshRenderer.dir/src/Renderer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/MeshRenderer.dir/src/Renderer.cpp.i"
	/usr/bin/g++-11 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/shangar21/Documents/MeshRenderer/src/Renderer.cpp > CMakeFiles/MeshRenderer.dir/src/Renderer.cpp.i

CMakeFiles/MeshRenderer.dir/src/Renderer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/MeshRenderer.dir/src/Renderer.cpp.s"
	/usr/bin/g++-11 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/shangar21/Documents/MeshRenderer/src/Renderer.cpp -o CMakeFiles/MeshRenderer.dir/src/Renderer.cpp.s

CMakeFiles/MeshRenderer.dir/src/Ray.cpp.o: CMakeFiles/MeshRenderer.dir/flags.make
CMakeFiles/MeshRenderer.dir/src/Ray.cpp.o: ../src/Ray.cpp
CMakeFiles/MeshRenderer.dir/src/Ray.cpp.o: CMakeFiles/MeshRenderer.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/shangar21/Documents/MeshRenderer/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/MeshRenderer.dir/src/Ray.cpp.o"
	/usr/bin/g++-11 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/MeshRenderer.dir/src/Ray.cpp.o -MF CMakeFiles/MeshRenderer.dir/src/Ray.cpp.o.d -o CMakeFiles/MeshRenderer.dir/src/Ray.cpp.o -c /home/shangar21/Documents/MeshRenderer/src/Ray.cpp

CMakeFiles/MeshRenderer.dir/src/Ray.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/MeshRenderer.dir/src/Ray.cpp.i"
	/usr/bin/g++-11 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/shangar21/Documents/MeshRenderer/src/Ray.cpp > CMakeFiles/MeshRenderer.dir/src/Ray.cpp.i

CMakeFiles/MeshRenderer.dir/src/Ray.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/MeshRenderer.dir/src/Ray.cpp.s"
	/usr/bin/g++-11 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/shangar21/Documents/MeshRenderer/src/Ray.cpp -o CMakeFiles/MeshRenderer.dir/src/Ray.cpp.s

CMakeFiles/MeshRenderer.dir/src/Hit.cpp.o: CMakeFiles/MeshRenderer.dir/flags.make
CMakeFiles/MeshRenderer.dir/src/Hit.cpp.o: ../src/Hit.cpp
CMakeFiles/MeshRenderer.dir/src/Hit.cpp.o: CMakeFiles/MeshRenderer.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/shangar21/Documents/MeshRenderer/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/MeshRenderer.dir/src/Hit.cpp.o"
	/usr/bin/g++-11 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/MeshRenderer.dir/src/Hit.cpp.o -MF CMakeFiles/MeshRenderer.dir/src/Hit.cpp.o.d -o CMakeFiles/MeshRenderer.dir/src/Hit.cpp.o -c /home/shangar21/Documents/MeshRenderer/src/Hit.cpp

CMakeFiles/MeshRenderer.dir/src/Hit.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/MeshRenderer.dir/src/Hit.cpp.i"
	/usr/bin/g++-11 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/shangar21/Documents/MeshRenderer/src/Hit.cpp > CMakeFiles/MeshRenderer.dir/src/Hit.cpp.i

CMakeFiles/MeshRenderer.dir/src/Hit.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/MeshRenderer.dir/src/Hit.cpp.s"
	/usr/bin/g++-11 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/shangar21/Documents/MeshRenderer/src/Hit.cpp -o CMakeFiles/MeshRenderer.dir/src/Hit.cpp.s

# Object files for target MeshRenderer
MeshRenderer_OBJECTS = \
"CMakeFiles/MeshRenderer.dir/src/main.cpp.o" \
"CMakeFiles/MeshRenderer.dir/src/Mesh.cpp.o" \
"CMakeFiles/MeshRenderer.dir/src/Camera.cpp.o" \
"CMakeFiles/MeshRenderer.dir/src/Renderer.cpp.o" \
"CMakeFiles/MeshRenderer.dir/src/Ray.cpp.o" \
"CMakeFiles/MeshRenderer.dir/src/Hit.cpp.o"

# External object files for target MeshRenderer
MeshRenderer_EXTERNAL_OBJECTS =

MeshRenderer: CMakeFiles/MeshRenderer.dir/src/main.cpp.o
MeshRenderer: CMakeFiles/MeshRenderer.dir/src/Mesh.cpp.o
MeshRenderer: CMakeFiles/MeshRenderer.dir/src/Camera.cpp.o
MeshRenderer: CMakeFiles/MeshRenderer.dir/src/Renderer.cpp.o
MeshRenderer: CMakeFiles/MeshRenderer.dir/src/Ray.cpp.o
MeshRenderer: CMakeFiles/MeshRenderer.dir/src/Hit.cpp.o
MeshRenderer: CMakeFiles/MeshRenderer.dir/build.make
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_core.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_dnn.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_video.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_alphamat.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_barcode.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_dnn_objdetect.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_dnn_superres.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_face.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_hfs.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_img_hash.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_intensity_transform.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_mcc.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_quality.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_rapid.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_text.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_tracking.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_wechat_qrcode.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_text.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_video.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_dnn.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.4.5.4d
MeshRenderer: /usr/lib/x86_64-linux-gnu/libopencv_core.so.4.5.4d
MeshRenderer: CMakeFiles/MeshRenderer.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/shangar21/Documents/MeshRenderer/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Linking CXX executable MeshRenderer"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/MeshRenderer.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/MeshRenderer.dir/build: MeshRenderer
.PHONY : CMakeFiles/MeshRenderer.dir/build

CMakeFiles/MeshRenderer.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/MeshRenderer.dir/cmake_clean.cmake
.PHONY : CMakeFiles/MeshRenderer.dir/clean

CMakeFiles/MeshRenderer.dir/depend:
	cd /home/shangar21/Documents/MeshRenderer/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/shangar21/Documents/MeshRenderer /home/shangar21/Documents/MeshRenderer /home/shangar21/Documents/MeshRenderer/build /home/shangar21/Documents/MeshRenderer/build /home/shangar21/Documents/MeshRenderer/build/CMakeFiles/MeshRenderer.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/MeshRenderer.dir/depend

