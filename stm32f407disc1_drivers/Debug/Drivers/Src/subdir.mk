################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Src/stm32f407disc1_gpio_driver.c \
../Drivers/Src/stm32f407disc1_i2c_driver.c \
../Drivers/Src/stm32f407disc1_spi_driver.c 

OBJS += \
./Drivers/Src/stm32f407disc1_gpio_driver.o \
./Drivers/Src/stm32f407disc1_i2c_driver.o \
./Drivers/Src/stm32f407disc1_spi_driver.o 

C_DEPS += \
./Drivers/Src/stm32f407disc1_gpio_driver.d \
./Drivers/Src/stm32f407disc1_i2c_driver.d \
./Drivers/Src/stm32f407disc1_spi_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Src/%.o Drivers/Src/%.su: ../Drivers/Src/%.c Drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -I"C:/Users/USER/OneDrive/Desktop/PERSONAL PROJECTS/STM32/stm32f407disc1_drivers/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Src

clean-Drivers-2f-Src:
	-$(RM) ./Drivers/Src/stm32f407disc1_gpio_driver.d ./Drivers/Src/stm32f407disc1_gpio_driver.o ./Drivers/Src/stm32f407disc1_gpio_driver.su ./Drivers/Src/stm32f407disc1_i2c_driver.d ./Drivers/Src/stm32f407disc1_i2c_driver.o ./Drivers/Src/stm32f407disc1_i2c_driver.su ./Drivers/Src/stm32f407disc1_spi_driver.d ./Drivers/Src/stm32f407disc1_spi_driver.o ./Drivers/Src/stm32f407disc1_spi_driver.su

.PHONY: clean-Drivers-2f-Src

