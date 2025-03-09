################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/pid/pid_controller.c 

OBJS += \
./Core/Src/pid/pid_controller.o 

C_DEPS += \
./Core/Src/pid/pid_controller.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/pid/%.o Core/Src/pid/%.su Core/Src/pid/%.cyclo: ../Core/Src/pid/%.c Core/Src/pid/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F767xx -c -I../LWIP/App -I"C:/Users/korbi/STM32CubeIDE/workspace_1.13.2/ugv_autonomous/Drivers/ugv_drivers/drive_motor" -I"C:/Users/korbi/STM32CubeIDE/workspace_1.13.2/ugv_autonomous/Drivers/ugv_drivers/servo" -I../LWIP/Target -I../Core/Inc -I../Middlewares/Third_Party/LwIP/src/include -I../Middlewares/Third_Party/LwIP/system -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/BSP/Components/lan8742 -I../Middlewares/Third_Party/LwIP/src/include/netif/ppp -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Middlewares/Third_Party/LwIP/src/include/lwip -I../Middlewares/Third_Party/LwIP/src/include/lwip/apps -I../Middlewares/Third_Party/LwIP/src/include/lwip/priv -I../Middlewares/Third_Party/LwIP/src/include/lwip/prot -I../Middlewares/Third_Party/LwIP/src/include/netif -I../Middlewares/Third_Party/LwIP/src/include/compat/posix -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/arpa -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/net -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/sys -I../Middlewares/Third_Party/LwIP/src/include/compat/stdc -I../Middlewares/Third_Party/LwIP/system/arch -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-pid

clean-Core-2f-Src-2f-pid:
	-$(RM) ./Core/Src/pid/pid_controller.cyclo ./Core/Src/pid/pid_controller.d ./Core/Src/pid/pid_controller.o ./Core/Src/pid/pid_controller.su

.PHONY: clean-Core-2f-Src-2f-pid

