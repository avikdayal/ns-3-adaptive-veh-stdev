################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CC_SRCS += \
../examples/realtime/realtime-udp-echo.cc 

CC_DEPS += \
./examples/realtime/realtime-udp-echo.d 

OBJS += \
./examples/realtime/realtime-udp-echo.o 


# Each subdirectory must supply rules for building sources it contributes
examples/realtime/%.o: ../examples/realtime/%.cc
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


