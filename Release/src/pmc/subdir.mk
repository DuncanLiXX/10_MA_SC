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
	arm-linux-gnueabihf-g++ -Wall -O2 -I"D:\FPGA\10MA_SC_Module_MultiChn\inc" -I"D:\FPGA\10MA_SC_Module_MultiChn\inc\tool_compensate" -I"D:\FPGA\10MA_SC_Module_MultiChn\lib\inc" -I"D:\FPGA\10MA_SC_Module_MultiChn\inc\pmc" -I"D:\FPGA\10MA_SC_Module_MultiChn\inc\Algorithm" -I"D:\FPGA\10MA_SC_Module_MultiChn\inc\alarm_processor" -I"D:\FPGA\10MA_SC_Module_MultiChn\inc\parameter" -I"D:\FPGA\10MA_SC_Module_MultiChn\inc\communication" -I"D:\FPGA\10MA_SC_Module_MultiChn\inc\channel" -I"D:\FPGA\10MA_SC_Module_MultiChn\inc\compiler" -c -fmessage-length=0 -MT"$@" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


