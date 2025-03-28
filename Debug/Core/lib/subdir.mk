################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/lib/Canbus.c \
../Core/lib/MCP4922.c \
../Core/lib/MotorControl.c \
../Core/lib/hydraulic.c 

OBJS += \
./Core/lib/Canbus.o \
./Core/lib/MCP4922.o \
./Core/lib/MotorControl.o \
./Core/lib/hydraulic.o 

C_DEPS += \
./Core/lib/Canbus.d \
./Core/lib/MCP4922.d \
./Core/lib/MotorControl.d \
./Core/lib/hydraulic.d 


# Each subdirectory must supply rules for building sources it contributes
Core/lib/%.o Core/lib/%.su Core/lib/%.cyclo: ../Core/lib/%.c Core/lib/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"D:/CodeSTM32/program_hydralic/Core/lib" -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-lib

clean-Core-2f-lib:
	-$(RM) ./Core/lib/Canbus.cyclo ./Core/lib/Canbus.d ./Core/lib/Canbus.o ./Core/lib/Canbus.su ./Core/lib/MCP4922.cyclo ./Core/lib/MCP4922.d ./Core/lib/MCP4922.o ./Core/lib/MCP4922.su ./Core/lib/MotorControl.cyclo ./Core/lib/MotorControl.d ./Core/lib/MotorControl.o ./Core/lib/MotorControl.su ./Core/lib/hydraulic.cyclo ./Core/lib/hydraulic.d ./Core/lib/hydraulic.o ./Core/lib/hydraulic.su

.PHONY: clean-Core-2f-lib

