################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/compiler/compile_message.cpp \
../src/compiler/compiler.cpp \
../src/compiler/compiler_data.cpp \
../src/compiler/lexer.cpp \
../src/compiler/parser.cpp 

OBJS += \
./src/compiler/compile_message.o \
./src/compiler/compiler.o \
./src/compiler/compiler_data.o \
./src/compiler/lexer.o \
./src/compiler/parser.o 

CPP_DEPS += \
./src/compiler/compile_message.d \
./src/compiler/compiler.d \
./src/compiler/compiler_data.d \
./src/compiler/lexer.d \
./src/compiler/parser.d 


# Each subdirectory must supply rules for building sources it contributes
src/compiler/%.o: ../src/compiler/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: ARM v7 Linux g++ compiler'
	arm-linux-gnueabihf-g++ -Wall -O0 -g3 -I"D:\FPGA\SC_Module\inc" -I"D:\FPGA\SC_Module\inc\tool_compensate" -I"D:\FPGA\SC_Module\lib\inc" -I"D:\FPGA\SC_Module\inc\pmc" -I"D:\FPGA\SC_Module\inc\Algorithm" -I"D:\FPGA\SC_Module\inc\alarm_processor" -I"D:\FPGA\SC_Module\inc\parameter" -I"D:\FPGA\SC_Module\inc\communication" -I"D:\FPGA\SC_Module\inc\channel" -I"D:\FPGA\SC_Module\inc\compiler" -c -fmessage-length=0 -MT"$@" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


