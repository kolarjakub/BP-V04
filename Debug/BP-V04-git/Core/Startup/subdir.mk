################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../BP-V04-git/Core/Startup/startup_stm32c011j6mx.s 

OBJS += \
./BP-V04-git/Core/Startup/startup_stm32c011j6mx.o 

S_DEPS += \
./BP-V04-git/Core/Startup/startup_stm32c011j6mx.d 


# Each subdirectory must supply rules for building sources it contributes
BP-V04-git/Core/Startup/%.o: ../BP-V04-git/Core/Startup/%.s BP-V04-git/Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m0plus -g3 -DDEBUG -c -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@" "$<"

clean: clean-BP-2d-V04-2d-git-2f-Core-2f-Startup

clean-BP-2d-V04-2d-git-2f-Core-2f-Startup:
	-$(RM) ./BP-V04-git/Core/Startup/startup_stm32c011j6mx.d ./BP-V04-git/Core/Startup/startup_stm32c011j6mx.o

.PHONY: clean-BP-2d-V04-2d-git-2f-Core-2f-Startup

