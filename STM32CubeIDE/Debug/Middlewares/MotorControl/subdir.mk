################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/STUser/.st_workbench/projects/testproj_02/MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Src/bus_voltage_sensor.c \
C:/Users/STUser/.st_workbench/projects/testproj_02/MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Src/circle_limitation.c \
C:/Users/STUser/.st_workbench/projects/testproj_02/MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Src/digital_output.c \
C:/Users/STUser/.st_workbench/projects/testproj_02/MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Src/enc_align_ctrl.c \
C:/Users/STUser/.st_workbench/projects/testproj_02/MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Src/encoder_speed_pos_fdbk.c \
C:/Users/STUser/.st_workbench/projects/testproj_02/MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Src/mcpa.c \
C:/Users/STUser/.st_workbench/projects/testproj_02/MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Src/ntc_temperature_sensor.c \
C:/Users/STUser/.st_workbench/projects/testproj_02/MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Src/open_loop.c \
C:/Users/STUser/.st_workbench/projects/testproj_02/MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Src/pid_regulator.c \
C:/Users/STUser/.st_workbench/projects/testproj_02/MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Src/pqd_motor_power_measurement.c \
C:/Users/STUser/.st_workbench/projects/testproj_02/MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/F3xx/Src/r3_2_f30x_pwm_curr_fdbk.c \
C:/Users/STUser/.st_workbench/projects/testproj_02/MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Src/r_divider_bus_voltage_sensor.c \
C:/Users/STUser/.st_workbench/projects/testproj_02/MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Src/ramp_ext_mngr.c \
C:/Users/STUser/.st_workbench/projects/testproj_02/MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Src/revup_ctrl.c \
C:/Users/STUser/.st_workbench/projects/testproj_02/MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Src/speed_pos_fdbk.c \
C:/Users/STUser/.st_workbench/projects/testproj_02/MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Src/trajectory_ctrl.c \
C:/Users/STUser/.st_workbench/projects/testproj_02/MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Src/virtual_speed_sensor.c 

OBJS += \
./Middlewares/MotorControl/bus_voltage_sensor.o \
./Middlewares/MotorControl/circle_limitation.o \
./Middlewares/MotorControl/digital_output.o \
./Middlewares/MotorControl/enc_align_ctrl.o \
./Middlewares/MotorControl/encoder_speed_pos_fdbk.o \
./Middlewares/MotorControl/mcpa.o \
./Middlewares/MotorControl/ntc_temperature_sensor.o \
./Middlewares/MotorControl/open_loop.o \
./Middlewares/MotorControl/pid_regulator.o \
./Middlewares/MotorControl/pqd_motor_power_measurement.o \
./Middlewares/MotorControl/r3_2_f30x_pwm_curr_fdbk.o \
./Middlewares/MotorControl/r_divider_bus_voltage_sensor.o \
./Middlewares/MotorControl/ramp_ext_mngr.o \
./Middlewares/MotorControl/revup_ctrl.o \
./Middlewares/MotorControl/speed_pos_fdbk.o \
./Middlewares/MotorControl/trajectory_ctrl.o \
./Middlewares/MotorControl/virtual_speed_sensor.o 

C_DEPS += \
./Middlewares/MotorControl/bus_voltage_sensor.d \
./Middlewares/MotorControl/circle_limitation.d \
./Middlewares/MotorControl/digital_output.d \
./Middlewares/MotorControl/enc_align_ctrl.d \
./Middlewares/MotorControl/encoder_speed_pos_fdbk.d \
./Middlewares/MotorControl/mcpa.d \
./Middlewares/MotorControl/ntc_temperature_sensor.d \
./Middlewares/MotorControl/open_loop.d \
./Middlewares/MotorControl/pid_regulator.d \
./Middlewares/MotorControl/pqd_motor_power_measurement.d \
./Middlewares/MotorControl/r3_2_f30x_pwm_curr_fdbk.d \
./Middlewares/MotorControl/r_divider_bus_voltage_sensor.d \
./Middlewares/MotorControl/ramp_ext_mngr.d \
./Middlewares/MotorControl/revup_ctrl.d \
./Middlewares/MotorControl/speed_pos_fdbk.d \
./Middlewares/MotorControl/trajectory_ctrl.d \
./Middlewares/MotorControl/virtual_speed_sensor.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/MotorControl/bus_voltage_sensor.o: C:/Users/STUser/.st_workbench/projects/testproj_02/MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Src/bus_voltage_sensor.c Middlewares/MotorControl/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../../Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/MotorControl/circle_limitation.o: C:/Users/STUser/.st_workbench/projects/testproj_02/MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Src/circle_limitation.c Middlewares/MotorControl/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../../Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/MotorControl/digital_output.o: C:/Users/STUser/.st_workbench/projects/testproj_02/MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Src/digital_output.c Middlewares/MotorControl/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../../Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/MotorControl/enc_align_ctrl.o: C:/Users/STUser/.st_workbench/projects/testproj_02/MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Src/enc_align_ctrl.c Middlewares/MotorControl/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../../Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/MotorControl/encoder_speed_pos_fdbk.o: C:/Users/STUser/.st_workbench/projects/testproj_02/MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Src/encoder_speed_pos_fdbk.c Middlewares/MotorControl/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../../Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/MotorControl/mcpa.o: C:/Users/STUser/.st_workbench/projects/testproj_02/MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Src/mcpa.c Middlewares/MotorControl/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../../Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/MotorControl/ntc_temperature_sensor.o: C:/Users/STUser/.st_workbench/projects/testproj_02/MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Src/ntc_temperature_sensor.c Middlewares/MotorControl/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../../Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/MotorControl/open_loop.o: C:/Users/STUser/.st_workbench/projects/testproj_02/MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Src/open_loop.c Middlewares/MotorControl/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../../Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/MotorControl/pid_regulator.o: C:/Users/STUser/.st_workbench/projects/testproj_02/MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Src/pid_regulator.c Middlewares/MotorControl/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../../Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/MotorControl/pqd_motor_power_measurement.o: C:/Users/STUser/.st_workbench/projects/testproj_02/MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Src/pqd_motor_power_measurement.c Middlewares/MotorControl/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../../Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/MotorControl/r3_2_f30x_pwm_curr_fdbk.o: C:/Users/STUser/.st_workbench/projects/testproj_02/MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/F3xx/Src/r3_2_f30x_pwm_curr_fdbk.c Middlewares/MotorControl/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../../Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/MotorControl/r_divider_bus_voltage_sensor.o: C:/Users/STUser/.st_workbench/projects/testproj_02/MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Src/r_divider_bus_voltage_sensor.c Middlewares/MotorControl/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../../Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/MotorControl/ramp_ext_mngr.o: C:/Users/STUser/.st_workbench/projects/testproj_02/MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Src/ramp_ext_mngr.c Middlewares/MotorControl/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../../Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/MotorControl/revup_ctrl.o: C:/Users/STUser/.st_workbench/projects/testproj_02/MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Src/revup_ctrl.c Middlewares/MotorControl/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../../Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/MotorControl/speed_pos_fdbk.o: C:/Users/STUser/.st_workbench/projects/testproj_02/MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Src/speed_pos_fdbk.c Middlewares/MotorControl/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../../Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/MotorControl/trajectory_ctrl.o: C:/Users/STUser/.st_workbench/projects/testproj_02/MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Src/trajectory_ctrl.c Middlewares/MotorControl/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../../Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/MotorControl/virtual_speed_sensor.o: C:/Users/STUser/.st_workbench/projects/testproj_02/MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Src/virtual_speed_sensor.c Middlewares/MotorControl/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../../Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.4.0-Full/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

