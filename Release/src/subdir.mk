################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/geometry_data.cpp \
../src/global_structure.cpp \
../src/global_var_func.cpp \
../src/inifile.cpp \
../src/main.cpp \
../src/trace.cpp \
../src/variable.cpp 

OBJS += \
./src/geometry_data.o \
./src/global_structure.o \
./src/global_var_func.o \
./src/inifile.o \
./src/main.o \
./src/trace.o \
./src/variable.o 

CPP_DEPS += \
./src/geometry_data.d \
./src/global_structure.d \
./src/global_var_func.d \
./src/inifile.d \
./src/main.d \
./src/trace.d \
./src/variable.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: ARM v7 Linux g++ compiler'
	arm-linux-gnueabihf-g++ -Wall -O2 -I"D:\FPGA\10MA_SC_Module_MultiChn\inc" -I"D:\FPGA\10MA_SC_Module_MultiChn\inc\tool_compensate" -I"D:\FPGA\10MA_SC_Module_MultiChn\lib\inc" -I"D:\FPGA\10MA_SC_Module_MultiChn\inc\pmc" -I"D:\FPGA\10MA_SC_Module_MultiChn\inc\Algorithm" -I"D:\FPGA\10MA_SC_Module_MultiChn\inc\alarm_processor" -I"D:\FPGA\10MA_SC_Module_MultiChn\inc\parameter" -I"D:\FPGA\10MA_SC_Module_MultiChn\inc\communication" -I"D:\FPGA\10MA_SC_Module_MultiChn\inc\channel" -I"D:\FPGA\10MA_SC_Module_MultiChn\inc\compiler" -c -fmessage-length=0 -MT"$@" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


