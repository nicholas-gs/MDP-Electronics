################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../PIDController/Src/PIDController.cpp 

OBJS += \
./PIDController/Src/PIDController.o 

CPP_DEPS += \
./PIDController/Src/PIDController.d 


# Each subdirectory must supply rules for building sources it contributes
PIDController/Src/%.o: ../PIDController/Src/%.cpp PIDController/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"C:/Users/Nicho/Documents/NTU/AY2021-2022 SEM2/CE3004/src/stm32/sketches/image_recognition/PIDController/Inc" -I"C:/Users/Nicho/Documents/NTU/AY2021-2022 SEM2/CE3004/src/stm32/sketches/image_recognition/PeripheralDriver/Inc" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-PIDController-2f-Src

clean-PIDController-2f-Src:
	-$(RM) ./PIDController/Src/PIDController.d ./PIDController/Src/PIDController.o

.PHONY: clean-PIDController-2f-Src

