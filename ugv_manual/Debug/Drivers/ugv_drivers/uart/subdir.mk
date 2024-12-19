################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/ugv_drivers/uart/uart_utilities.c 

C_DEPS += \
./Drivers/ugv_drivers/uart/uart_utilities.d 

OBJS += \
./Drivers/ugv_drivers/uart/uart_utilities.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/ugv_drivers/uart/%.o Drivers/ugv_drivers/uart/%.su Drivers/ugv_drivers/uart/%.cyclo: ../Drivers/ugv_drivers/uart/%.c Drivers/ugv_drivers/uart/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F767xx -c -I../LWIP/App -I"/home/chris/NGCP/NGCP_24_25/ugv-firmware/ugv_manual/Drivers/ugv_drivers/drive_motor" -I"/home/chris/NGCP/NGCP_24_25/ugv-firmware/ugv_manual/Drivers/ugv_drivers/servo" -I../LWIP/Target -I../Core/Inc -I../Middlewares/Third_Party/LwIP/src/include -I../Middlewares/Third_Party/LwIP/system -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/BSP/Components/lan8742 -I../Middlewares/Third_Party/LwIP/src/include/netif/ppp -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Middlewares/Third_Party/LwIP/src/include/lwip -I../Middlewares/Third_Party/LwIP/src/include/lwip/apps -I../Middlewares/Third_Party/LwIP/src/include/lwip/priv -I../Middlewares/Third_Party/LwIP/src/include/lwip/prot -I../Middlewares/Third_Party/LwIP/src/include/netif -I../Middlewares/Third_Party/LwIP/src/include/compat/posix -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/arpa -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/net -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/sys -I../Middlewares/Third_Party/LwIP/src/include/compat/stdc -I../Middlewares/Third_Party/LwIP/system/arch -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-ugv_drivers-2f-uart

clean-Drivers-2f-ugv_drivers-2f-uart:
	-$(RM) ./Drivers/ugv_drivers/uart/uart_utilities.cyclo ./Drivers/ugv_drivers/uart/uart_utilities.d ./Drivers/ugv_drivers/uart/uart_utilities.o ./Drivers/ugv_drivers/uart/uart_utilities.su

.PHONY: clean-Drivers-2f-ugv_drivers-2f-uart

