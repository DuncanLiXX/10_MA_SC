################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/pmc/pmc_axis_ctrl.cpp \
../src/pmc/pmc_register.cpp 

OBJS += \
./src/pmc/pmc_axis_ctrl.o \
./src/pmc/pmc_register.o 

CPP_DEPS += \
./src/pmc/pmc_axis_ctrl.d \
./src/pmc/pmc_register.d 


# Each subdirectory must supply rules for building sources it contributes
src/pmc/%.o: ../src/pmc/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: ARM v7 Linux g++ compiler'
	arm-linux-gnueabihf-g++ -Wall -O0 -g3 -I"D:\FPGA\SC_Module\inc" -I"D:\FPGA\SC_Module\inc\tool_compensate" -I"D:\FPGA\SC_Module\lib\inc" -I"D:\FPGA\SC_Module\inc\pmc" -I"D:\FPGA\SC_Module\inc\Algorithm" -I"D:\FPGA\SC_Module\inc\alarm_processor" -I"D:\FPGA\SC_Module\inc\parameter" -I"D:\FPGA\SC_Module\inc\communication" -I"D:\FPGA\SC_Module\inc\channel" -I"D:\FPGA\SC_Module\inc\compiler" -c -fmessage-length=0 -MT"$@" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


