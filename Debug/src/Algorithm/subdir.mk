################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/Algorithm/coord_processor.cpp 

OBJS += \
./src/Algorithm/coord_processor.o 

CPP_DEPS += \
./src/Algorithm/coord_processor.d 


# Each subdirectory must supply rules for building sources it contributes
src/Algorithm/%.o: ../src/Algorithm/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: ARM v7 Linux g++ compiler'
	arm-linux-gnueabihf-g++ -Wall -O0 -g3 -I"D:\FPGA\SC_Module\inc" -I"D:\FPGA\SC_Module\inc\tool_compensate" -I"D:\FPGA\SC_Module\lib\inc" -I"D:\FPGA\SC_Module\inc\pmc" -I"D:\FPGA\SC_Module\inc\Algorithm" -I"D:\FPGA\SC_Module\inc\alarm_processor" -I"D:\FPGA\SC_Module\inc\parameter" -I"D:\FPGA\SC_Module\inc\communication" -I"D:\FPGA\SC_Module\inc\channel" -I"D:\FPGA\SC_Module\inc\compiler" -c -fmessage-length=0 -MT"$@" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


