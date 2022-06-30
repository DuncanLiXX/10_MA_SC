################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/tool_compensate/tool_comp_data.cpp \
../src/tool_compensate/tool_compensate.cpp \
../src/tool_compensate/tool_math.cpp \
../src/tool_compensate/tools_comp.cpp 

OBJS += \
./src/tool_compensate/tool_comp_data.o \
./src/tool_compensate/tool_compensate.o \
./src/tool_compensate/tool_math.o \
./src/tool_compensate/tools_comp.o 

CPP_DEPS += \
./src/tool_compensate/tool_comp_data.d \
./src/tool_compensate/tool_compensate.d \
./src/tool_compensate/tool_math.d \
./src/tool_compensate/tools_comp.d 


# Each subdirectory must supply rules for building sources it contributes
src/tool_compensate/%.o: ../src/tool_compensate/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: ARM v7 Linux g++ compiler'
	arm-linux-gnueabihf-g++ -Wall -O2 -I"D:\FPGA\SC_Module\inc" -I"D:\FPGA\SC_Module\inc\tool_compensate" -I"D:\FPGA\SC_Module\lib\inc" -I"D:\FPGA\SC_Module\inc\pmc" -I"D:\FPGA\SC_Module\inc\Algorithm" -I"D:\FPGA\SC_Module\inc\alarm_processor" -I"D:\FPGA\SC_Module\inc\parameter" -I"D:\FPGA\SC_Module\inc\communication" -I"D:\FPGA\SC_Module\inc\channel" -I"D:\FPGA\SC_Module\inc\compiler" -c -fmessage-length=0 -MT"$@" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


