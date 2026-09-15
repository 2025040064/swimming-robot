#ifndef BSP_MPU_MOTOR_TEST_H
#define BSP_MPU_MOTOR_TEST_H

#include "stm32f10x.h"

/* Isolated bench test: bypass vision/ranging, retain all IMU protection.
 * Set to 0 to restore the normal navigation and range-hold path. */
#define ROBOT_IMU_MOTOR_TEST_ENABLE  1
#define ROBOT_IMU_TEST_ONE_WIRE_DIRECTION  0
#define ROBOT_MOTOR_DIRECTION_DIAG_ENABLE  1
#define ROBOT_MOTOR_DIRECTION_DIAG_PWM  7199
#define ROBOT_IMU_TEST_PWM          3600
#define ROBOT_IMU_TEST_DURATION_MS  30000U

void BSP_MpuMotorTest_Init(void);
void BSP_MpuMotorDirectionDiagnostic(void);
void BSP_MpuMotorTest_Run(void);
uint8_t BSP_MpuMotorTest_IsFinished(void);
void BSP_MpuMotorTest_Report(void);
void BSP_MpuMotorTest_SendStatus(float tilt, int16_t left, int16_t right);

#endif
    
