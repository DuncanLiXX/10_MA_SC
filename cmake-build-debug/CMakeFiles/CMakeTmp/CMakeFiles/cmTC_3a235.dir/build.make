# CMAKE generated file: DO NOT EDIT!
# Generated by "MinGW Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Produce verbose output by default.
VERBOSE = 1

# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

SHELL = cmd.exe

# The CMake executable.
CMAKE_COMMAND = "D:\IDE\CLion 2020.1\bin\cmake\win\bin\cmake.exe"

# The command to remove a file.
RM = "D:\IDE\CLion 2020.1\bin\cmake\win\bin\cmake.exe" -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = D:\FPGA\SC_Module\cmake-build-debug\CMakeFiles\CMakeTmp

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = D:\FPGA\SC_Module\cmake-build-debug\CMakeFiles\CMakeTmp

# Include any dependencies generated for this target.
include CMakeFiles/cmTC_3a235.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/cmTC_3a235.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/cmTC_3a235.dir/flags.make

CMakeFiles/cmTC_3a235.dir/testCCompiler.c.o: CMakeFiles/cmTC_3a235.dir/flags.make
CMakeFiles/cmTC_3a235.dir/testCCompiler.c.o: testCCompiler.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --progress-dir=D:\FPGA\SC_Module\cmake-build-debug\CMakeFiles\CMakeTmp\CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object CMakeFiles/cmTC_3a235.dir/testCCompiler.c.o"
	C:\IDE\Xilinx\SDK\2019.1\gnu\aarch32\nt\gcc-arm-linux-gnueabi\bin\arm-linux-gnueabihf-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles\cmTC_3a235.dir\testCCompiler.c.o   -c D:\FPGA\SC_Module\cmake-build-debug\CMakeFiles\CMakeTmp\testCCompiler.c

CMakeFiles/cmTC_3a235.dir/testCCompiler.c.i: cmake_force
	@echo Preprocessing C source to CMakeFiles/cmTC_3a235.dir/testCCompiler.c.i
	C:\IDE\Xilinx\SDK\2019.1\gnu\aarch32\nt\gcc-arm-linux-gnueabi\bin\arm-linux-gnueabihf-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E D:\FPGA\SC_Module\cmake-build-debug\CMakeFiles\CMakeTmp\testCCompiler.c > CMakeFiles\cmTC_3a235.dir\testCCompiler.c.i

CMakeFiles/cmTC_3a235.dir/testCCompiler.c.s: cmake_force
	@echo Compiling C source to assembly CMakeFiles/cmTC_3a235.dir/testCCompiler.c.s
	C:\IDE\Xilinx\SDK\2019.1\gnu\aarch32\nt\gcc-arm-linux-gnueabi\bin\arm-linux-gnueabihf-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S D:\FPGA\SC_Module\cmake-build-debug\CMakeFiles\CMakeTmp\testCCompiler.c -o CMakeFiles\cmTC_3a235.dir\testCCompiler.c.s

# Object files for target cmTC_3a235
cmTC_3a235_OBJECTS = \
"CMakeFiles/cmTC_3a235.dir/testCCompiler.c.o"

# External object files for target cmTC_3a235
cmTC_3a235_EXTERNAL_OBJECTS =

cmTC_3a235: CMakeFiles/cmTC_3a235.dir/testCCompiler.c.o
cmTC_3a235: CMakeFiles/cmTC_3a235.dir/build.make
cmTC_3a235: CMakeFiles/cmTC_3a235.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --progress-dir=D:\FPGA\SC_Module\cmake-build-debug\CMakeFiles\CMakeTmp\CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C executable cmTC_3a235"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles\cmTC_3a235.dir\link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/cmTC_3a235.dir/build: cmTC_3a235

.PHONY : CMakeFiles/cmTC_3a235.dir/build

CMakeFiles/cmTC_3a235.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles\cmTC_3a235.dir\cmake_clean.cmake
.PHONY : CMakeFiles/cmTC_3a235.dir/clean

CMakeFiles/cmTC_3a235.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" D:\FPGA\SC_Module\cmake-build-debug\CMakeFiles\CMakeTmp D:\FPGA\SC_Module\cmake-build-debug\CMakeFiles\CMakeTmp D:\FPGA\SC_Module\cmake-build-debug\CMakeFiles\CMakeTmp D:\FPGA\SC_Module\cmake-build-debug\CMakeFiles\CMakeTmp D:\FPGA\SC_Module\cmake-build-debug\CMakeFiles\CMakeTmp\CMakeFiles\cmTC_3a235.dir\DependInfo.cmake
.PHONY : CMakeFiles/cmTC_3a235.dir/depend

