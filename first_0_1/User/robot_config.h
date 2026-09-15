#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include "bsp/test/bsp_mpu_motor_test.h"

/* Passive underside net: keep the unused recovery bridge disabled. */

/* MPU6050 installation: sensor +X up, +Y toward the bow. */
#define ROBOT_IMU_X_UP                1
#define ROBOT_IMU_MIN_CALIB_SAMPLES   200U
#define ROBOT_IMU_STARTUP_MS          3500U
#define ROBOT_IMU_STALE_MS            100U

/* Bench calibration tolerance for the observed stationary sensor offsets.
 * Still require 200 consecutive samples and a bounded peak-to-peak window. */
#define ROBOT_IMU_CALIB_NORM_MIN_SQ   0.7225f
#define ROBOT_IMU_CALIB_NORM_MAX_SQ   1.3225f
#define ROBOT_IMU_CALIB_GYRO_MAX_RAW  280.0f
#define ROBOT_IMU_CALIB_ACCEL_SPAN    256
#define ROBOT_IMU_CALIB_GYRO_SPAN     164

/* Total inclination from upright; bench defaults, to be checked afloat. */
#define ROBOT_TILT_SLOW_DEG           12.0f
#define ROBOT_TILT_STOP_DEG           25.0f
#define ROBOT_TILT_STOP_MS            200U
#define ROBOT_TILT_HARD_DEG           45.0f
#define ROBOT_TILT_MIN_THRUST_SCALE   0.35f

/* AJ-SR04M /58 conversion is for air. The fitted probes are submerged.
 * Leave this at 0 until a suitable underwater ranging driver is installed
 * and its distances are verified; 0 permits telemetry, but holds propulsion.
 * Air-only bench tests can use 1 with the probes operating in air. */
#define ROBOT_RANGING_VALIDATED       0

/* Beams point forward and forward-left/right at 60 degrees to the centre. */
#define ROBOT_FRONT_AVOID_CM          50.0f
#define ROBOT_SIDE_AVOID_CM           60.0f
#define ROBOT_AVOID_CLEAR_CM          80.0f

#endif
