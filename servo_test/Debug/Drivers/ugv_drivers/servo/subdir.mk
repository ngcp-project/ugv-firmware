################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/ugv_drivers/servo/ugv_servo.c 

OBJS += \
./Drivers/ugv_drivers/servo/ugv_servo.o 

C_DEPS += \
./Drivers/ugv_drivers/servo/ugv_servo.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/ugv_drivers/servo/%.o Drivers/ugv_drivers/servo/%.su Drivers/ugv_drivers/servo/%.cyclo: ../Drivers/ugv_drivers/servo/%.c Drivers/ugv_drivers/servo/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F767xx -c -I../Core/Inc -I"/home/chris/NGCP/NGCP_UGV_24_25/ugv-firmware/servo_test/Drivers/ugv_drivers/servo" -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-ugv_drivers-2f-servo

clean-Drivers-2f-ugv_drivers-2f-servo:
	-$(RM) ./Drivers/ugv_drivers/servo/ugv_servo.cyclo ./Drivers/ugv_drivers/servo/ugv_servo.d ./Drivers/ugv_drivers/servo/ugv_servo.o ./Drivers/ugv_drivers/servo/ugv_servo.su

.PHONY: clean-Drivers-2f-ugv_drivers-2f-servo

