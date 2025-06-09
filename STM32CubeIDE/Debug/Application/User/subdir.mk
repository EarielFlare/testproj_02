################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/STUser/.st_workbench/projects/testproj_02/Src/aspep.c \
C:/Users/STUser/.st_workbench/projects/testproj_02/Src/hf_registers.c \
C:/Users/STUser/.st_workbench/projects/testproj_02/Src/main.c \
C:/Users/STUser/.st_workbench/projects/testproj_02/Src/mc_api.c \
C:/Users/STUser/.st_workbench/projects/testproj_02/Src/mc_app_hooks.c \
C:/Users/STUser/.st_workbench/projects/testproj_02/Src/mc_config.c \
C:/Users/STUser/.st_workbench/projects/testproj_02/Src/mc_config_common.c \
C:/Users/STUser/.st_workbench/projects/testproj_02/Src/mc_configuration_registers.c \
C:/Users/STUser/.st_workbench/projects/testproj_02/Src/mc_interface.c \
C:/Users/STUser/.st_workbench/projects/testproj_02/Src/mc_math.c \
C:/Users/STUser/.st_workbench/projects/testproj_02/Src/mc_parameters.c \
C:/Users/STUser/.st_workbench/projects/testproj_02/Src/mc_tasks.c \
C:/Users/STUser/.st_workbench/projects/testproj_02/Src/mc_tasks_foc.c \
C:/Users/STUser/.st_workbench/projects/testproj_02/Src/mcp.c \
C:/Users/STUser/.st_workbench/projects/testproj_02/Src/mcp_config.c \
C:/Users/STUser/.st_workbench/projects/testproj_02/Src/motorcontrol.c \
C:/Users/STUser/.st_workbench/projects/testproj_02/Src/pwm_common.c \
C:/Users/STUser/.st_workbench/projects/testproj_02/Src/pwm_curr_fdbk.c \
C:/Users/STUser/.st_workbench/projects/testproj_02/Src/regular_conversion_manager.c \
C:/Users/STUser/.st_workbench/projects/testproj_02/Src/speed_torq_ctrl.c \
C:/Users/STUser/.st_workbench/projects/testproj_02/Src/stm32_mc_common_it.c \
C:/Users/STUser/.st_workbench/projects/testproj_02/Src/stm32f30x_mc_it.c \
C:/Users/STUser/.st_workbench/projects/testproj_02/Src/stm32f3xx_hal_msp.c \
C:/Users/STUser/.st_workbench/projects/testproj_02/Src/stm32f3xx_it.c \
C:/Users/STUser/.st_workbench/projects/testproj_02/Src/sync_registers.c \
../Application/User/syscalls.c \
../Application/User/sysmem.c \
C:/Users/STUser/.st_workbench/projects/testproj_02/Src/usart_aspep_driver.c 

OBJS += \
./Application/User/aspep.o \
./Application/User/hf_registers.o \
./Application/User/main.o \
./Application/User/mc_api.o \
./Application/User/mc_app_hooks.o \
./Application/User/mc_config.o \
./Application/User/mc_config_common.o \
./Application/User/mc_configuration_registers.o \
./Application/User/mc_interface.o \
./Application/User/mc_math.o \
./Application/User/mc_parameters.o \
./Application/User/mc_tasks.o \
./Application/User/mc_tasks_foc.o \
./Application/User/mcp.o \
./Application/User/mcp_config.o \
./Application/User/motorcontrol.o \
./Application/User/pwm_common.o \
./Application/User/pwm_curr_fdbk.o \
./Application/User/regular_conversion_manager.o \
./Application/User/speed_torq_ctrl.o \
./Application/User/stm32_mc_common_it.o \
./Application/User/stm32f30x_mc_it.o \
./Application/User/stm32f3xx_hal_msp.o \
./Application/User/stm32f3xx_it.o \
./Application/User/sync_registers.o \
./Application/User/syscalls.o \
./Application/User/sysmem.o \
./Application/User/usart_aspep_driver.o 

C_DEPS += \
./Application/User/aspep.d \
./Application/User/hf_registers.d \
./Application/User/main.d \
./Application/User/mc_api.d \
./Application/User/mc_app_hooks.d \
./Application/User/mc_config.d \
./Application/User/mc_config_common.d \
./Application/User/mc_configuration_registers.d \
./Application/User/mc_interface.d \
./Application/User/mc_math.d \
./Application/User/mc_parameters.d \
./Application/User/mc_tasks.d \
./Application/User/mc_tasks_foc.d \
./Application/User/mcp.d \
./Application/User/mcp_config.d \
./Application/User/motorcontrol.d \
./Application/User/pwm_common.d \
./Application/User/pwm_curr_fdbk.d \
./Application/User/regular_conversion_manager.d \
./Application/User/speed_torq_ctrl.d \
./Application/User/stm32_mc_common_it.d \
./Application/User/stm32f30x_mc_it.d \
./Application/User/stm32f3xx_hal_msp.d \
./Application/User/stm32f3xx_it.d \
./Application/User/sync_registers.d \
./Application/User/syscalls.d \
./Application/User/sysmem.d \
./Application/User/usart_aspep_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Application/User/aspep.o: C:/Users/STUser/.st_workbench/projects/testproj_02/Src/aspep.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../../Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/hf_registers.o: C:/Users/STUser/.st_workbench/projects/testproj_02/Src/hf_registers.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../../Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/main.o: C:/Users/STUser/.st_workbench/projects/testproj_02/Src/main.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../../Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/mc_api.o: C:/Users/STUser/.st_workbench/projects/testproj_02/Src/mc_api.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../../Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/mc_app_hooks.o: C:/Users/STUser/.st_workbench/projects/testproj_02/Src/mc_app_hooks.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../../Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/mc_config.o: C:/Users/STUser/.st_workbench/projects/testproj_02/Src/mc_config.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../../Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/mc_config_common.o: C:/Users/STUser/.st_workbench/projects/testproj_02/Src/mc_config_common.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../../Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/mc_configuration_registers.o: C:/Users/STUser/.st_workbench/projects/testproj_02/Src/mc_configuration_registers.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../../Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/mc_interface.o: C:/Users/STUser/.st_workbench/projects/testproj_02/Src/mc_interface.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../../Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/mc_math.o: C:/Users/STUser/.st_workbench/projects/testproj_02/Src/mc_math.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../../Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/mc_parameters.o: C:/Users/STUser/.st_workbench/projects/testproj_02/Src/mc_parameters.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../../Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/mc_tasks.o: C:/Users/STUser/.st_workbench/projects/testproj_02/Src/mc_tasks.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../../Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/mc_tasks_foc.o: C:/Users/STUser/.st_workbench/projects/testproj_02/Src/mc_tasks_foc.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../../Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/mcp.o: C:/Users/STUser/.st_workbench/projects/testproj_02/Src/mcp.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../../Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/mcp_config.o: C:/Users/STUser/.st_workbench/projects/testproj_02/Src/mcp_config.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../../Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/motorcontrol.o: C:/Users/STUser/.st_workbench/projects/testproj_02/Src/motorcontrol.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../../Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/pwm_common.o: C:/Users/STUser/.st_workbench/projects/testproj_02/Src/pwm_common.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../../Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/pwm_curr_fdbk.o: C:/Users/STUser/.st_workbench/projects/testproj_02/Src/pwm_curr_fdbk.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../../Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/regular_conversion_manager.o: C:/Users/STUser/.st_workbench/projects/testproj_02/Src/regular_conversion_manager.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../../Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/speed_torq_ctrl.o: C:/Users/STUser/.st_workbench/projects/testproj_02/Src/speed_torq_ctrl.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../../Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/stm32_mc_common_it.o: C:/Users/STUser/.st_workbench/projects/testproj_02/Src/stm32_mc_common_it.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../../Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/stm32f30x_mc_it.o: C:/Users/STUser/.st_workbench/projects/testproj_02/Src/stm32f30x_mc_it.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../../Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/stm32f3xx_hal_msp.o: C:/Users/STUser/.st_workbench/projects/testproj_02/Src/stm32f3xx_hal_msp.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../../Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/stm32f3xx_it.o: C:/Users/STUser/.st_workbench/projects/testproj_02/Src/stm32f3xx_it.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../../Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/sync_registers.o: C:/Users/STUser/.st_workbench/projects/testproj_02/Src/sync_registers.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../../Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/%.o: ../Application/User/%.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../../Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/usart_aspep_driver.o: C:/Users/STUser/.st_workbench/projects/testproj_02/Src/usart_aspep_driver.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../../Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

