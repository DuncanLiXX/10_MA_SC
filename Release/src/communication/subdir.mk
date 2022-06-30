################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/communication/ad_communication.cpp \
../src/communication/hmi_communication.cpp \
../src/communication/mc_communication.cpp \
../src/communication/mc_communication_arm.cpp \
../src/communication/mi_communication.cpp 

OBJS += \
./src/communication/ad_communication.o \
./src/communication/hmi_communication.o \
./src/communication/mc_communication.o \
./src/communication/mc_communication_arm.o \
./src/communication/mi_communication.o 

CPP_DEPS += \
./src/communication/ad_communication.d \
./src/communication/hmi_communication.d \
./src/communication/mc_communication.d \
./src/communication/mc_communication_arm.d \
./src/communication/mi_communication.d 


# Each subdirectory must supply rules for building sources it contributes
src/communication/%.o: ../src/communication/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: ARM v7 Linux g++ compiler'
	arm-linux-gnueabihf-g++ -Wall -O2 -I"D:\FPGA\SC_Module\inc" -I"D:\FPGA\SC_Module\inc\tool_compensate" -I"D:\FPGA\SC_Module\lib\inc" -I"D:\FPGA\SC_Module\inc\pmc" -I"D:\FPGA\SC_Module\inc\Algorithm" -I"D:\FPGA\SC_Module\inc\alarm_processor" -I"D:\FPGA\SC_Module\inc\parameter" -I"D:\FPGA\SC_Module\inc\communication" -I"D:\FPGA\SC_Module\inc\channel" -I"D:\FPGA\SC_Module\inc\compiler" -c -fmessage-length=0 -MT"$@" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


