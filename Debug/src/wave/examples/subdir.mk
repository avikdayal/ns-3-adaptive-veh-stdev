################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CC_SRCS += \
../src/wave/examples/vanet-routing-compare.cc \
../src/wave/examples/vanet-routing-compare_2node.cc \
../src/wave/examples/wave-simple-80211p.cc \
../src/wave/examples/wave-simple-device.cc 

CC_DEPS += \
./src/wave/examples/vanet-routing-compare.d \
./src/wave/examples/vanet-routing-compare_2node.d \
./src/wave/examples/wave-simple-80211p.d \
./src/wave/examples/wave-simple-device.d 

OBJS += \
./src/wave/examples/vanet-routing-compare.o \
./src/wave/examples/vanet-routing-compare_2node.o \
./src/wave/examples/wave-simple-80211p.o \
./src/wave/examples/wave-simple-device.o 


# Each subdirectory must supply rules for building sources it contributes
src/wave/examples/%.o: ../src/wave/examples/%.cc
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


