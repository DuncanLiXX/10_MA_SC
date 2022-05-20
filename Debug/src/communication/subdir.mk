################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/communication/ad_communication.cpp \
../src/communication/hmi_communication.cpp \
../src/communication/mc_communication.cpp \
../src/communication/mi_communication.cpp 

OBJS += \
./src/communication/ad_communication.o \
./src/communication/hmi_communication.o \
./src/communication/mc_communication.o \
./src/communication/mi_communication.o 

CPP_DEPS += \
./src/communication/ad_communication.d \
./src/communication/hmi_communication.d \
./src/communication/mc_communication.d \
./src/communication/mi_communication.d 


# Each subdirectory must supply rules for building sources it contributes
src/communication/%.o: ../src/communication/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: ARM v7 Linux g++ compiler'
	arm-linux-gnueabihf-g++ -Wall -O0 -g3 -I"D:\FPGA\10MA_SC_Module_MultiChn\inc" -I"D:\FPGA\10MA_SC_Module_MultiChn\inc\tool_compensate" -I"D:\FPGA\10MA_SC_Module_MultiChn\lib\inc" -I"D:\FPGA\10MA_SC_Module_MultiChn\inc\pmc" -I"D:\FPGA\10MA_SC_Module_MultiChn\inc\Algorithm" -I"D:\FPGA\10MA_SC_Module_MultiChn\inc\alarm_processor" -I"D:\FPGA\10MA_SC_Module_MultiChn\inc\parameter" -I"D:\FPGA\10MA_SC_Module_MultiChn\inc\communication" -I"D:\FPGA\10MA_SC_Module_MultiChn\inc\channel" -I"D:\FPGA\10MA_SC_Module_MultiChn\inc\compiler" -c -fmessage-length=0 -MT"$@" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


