################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CC_SRCS += \
../scratch/myfirst.cc \
../scratch/scratch-simulator.cc \
../scratch/test-plot.cc \
../scratch/test.cc \
../scratch/vanet-routing-compare.cc \
../scratch/vanet-routing-compare_2node.cc \
../scratch/wave-simple-80211p_test.cc \
../scratch/wave-simple-device.cc 

CC_DEPS += \
./scratch/myfirst.d \
./scratch/scratch-simulator.d \
./scratch/test-plot.d \
./scratch/test.d \
./scratch/vanet-routing-compare.d \
./scratch/vanet-routing-compare_2node.d \
./scratch/wave-simple-80211p_test.d \
./scratch/wave-simple-device.d 

OBJS += \
./scratch/myfirst.o \
./scratch/scratch-simulator.o \
./scratch/test-plot.o \
./scratch/test.o \
./scratch/vanet-routing-compare.o \
./scratch/vanet-routing-compare_2node.o \
./scratch/wave-simple-80211p_test.o \
./scratch/wave-simple-device.o 


# Each subdirectory must supply rules for building sources it contributes
scratch/%.o: ../scratch/%.cc
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


