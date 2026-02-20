################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/bh1750.c \
../Core/Src/bme280.c \
../Core/Src/esp8266.c \
../Core/Src/main.c \
../Core/Src/power_monitor.c \
../Core/Src/sen5x_i2c.c \
../Core/Src/sen5x_wrapper.c \
../Core/Src/sensirion_common.c \
../Core/Src/sensirion_i2c.c \
../Core/Src/sensirion_i2c_hal.c \
../Core/Src/stm32f7xx_hal_msp.c \
../Core/Src/stm32f7xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f7xx.c 

OBJS += \
./Core/Src/bh1750.o \
./Core/Src/bme280.o \
./Core/Src/esp8266.o \
./Core/Src/main.o \
./Core/Src/power_monitor.o \
./Core/Src/sen5x_i2c.o \
./Core/Src/sen5x_wrapper.o \
./Core/Src/sensirion_common.o \
./Core/Src/sensirion_i2c.o \
./Core/Src/sensirion_i2c_hal.o \
./Core/Src/stm32f7xx_hal_msp.o \
./Core/Src/stm32f7xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f7xx.o 

C_DEPS += \
./Core/Src/bh1750.d \
./Core/Src/bme280.d \
./Core/Src/esp8266.d \
./Core/Src/main.d \
./Core/Src/power_monitor.d \
./Core/Src/sen5x_i2c.d \
./Core/Src/sen5x_wrapper.d \
./Core/Src/sensirion_common.d \
./Core/Src/sensirion_i2c.d \
./Core/Src/sensirion_i2c_hal.d \
./Core/Src/stm32f7xx_hal_msp.d \
./Core/Src/stm32f7xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f7xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F769xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/bh1750.cyclo ./Core/Src/bh1750.d ./Core/Src/bh1750.o ./Core/Src/bh1750.su ./Core/Src/bme280.cyclo ./Core/Src/bme280.d ./Core/Src/bme280.o ./Core/Src/bme280.su ./Core/Src/esp8266.cyclo ./Core/Src/esp8266.d ./Core/Src/esp8266.o ./Core/Src/esp8266.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/power_monitor.cyclo ./Core/Src/power_monitor.d ./Core/Src/power_monitor.o ./Core/Src/power_monitor.su ./Core/Src/sen5x_i2c.cyclo ./Core/Src/sen5x_i2c.d ./Core/Src/sen5x_i2c.o ./Core/Src/sen5x_i2c.su ./Core/Src/sen5x_wrapper.cyclo ./Core/Src/sen5x_wrapper.d ./Core/Src/sen5x_wrapper.o ./Core/Src/sen5x_wrapper.su ./Core/Src/sensirion_common.cyclo ./Core/Src/sensirion_common.d ./Core/Src/sensirion_common.o ./Core/Src/sensirion_common.su ./Core/Src/sensirion_i2c.cyclo ./Core/Src/sensirion_i2c.d ./Core/Src/sensirion_i2c.o ./Core/Src/sensirion_i2c.su ./Core/Src/sensirion_i2c_hal.cyclo ./Core/Src/sensirion_i2c_hal.d ./Core/Src/sensirion_i2c_hal.o ./Core/Src/sensirion_i2c_hal.su ./Core/Src/stm32f7xx_hal_msp.cyclo ./Core/Src/stm32f7xx_hal_msp.d ./Core/Src/stm32f7xx_hal_msp.o ./Core/Src/stm32f7xx_hal_msp.su ./Core/Src/stm32f7xx_it.cyclo ./Core/Src/stm32f7xx_it.d ./Core/Src/stm32f7xx_it.o ./Core/Src/stm32f7xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f7xx.cyclo ./Core/Src/system_stm32f7xx.d ./Core/Src/system_stm32f7xx.o ./Core/Src/system_stm32f7xx.su

.PHONY: clean-Core-2f-Src

