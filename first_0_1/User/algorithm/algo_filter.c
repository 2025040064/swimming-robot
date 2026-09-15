#include "algo_filter.h"
#include "robot_config.h"

static Attitude_t g_att;
static uint8_t g_sampleValid;
static uint8_t g_seeded;
static float g_tilt;

/* Rejection history since filter init: bit 0=calibration acceleration,
 * bit 1=calibration gyro motion, bit 2=invalid acceleration sample. */
volatile uint32_t g_debugCalibRejectMask;
volatile float g_debugCalibRejectNorm;
volatile float g_debugCalibRejectGyro;

static float Filter_Abs(float value)
{
    return value < 0.0f ? -value : value;
}

/* ---- Gyro bias calibration state ---- */
static uint8_t  g_calibrating = 0;
static int32_t  g_calibSumX   = 0;
static int32_t  g_calibSumY   = 0;
static int32_t  g_calibSumZ   = 0;
static uint16_t g_calibCount  = 0;
static int16_t g_calibAccelMin[3], g_calibAccelMax[3];
static int16_t g_calibGyroMin[3], g_calibGyroMax[3];

/* Bit 3 of g_debugCalibRejectMask records an unstable sample window.
 * Compare against the whole window so slow drift is not hidden by small
 * differences between adjacent samples. A stable offset is calibrated out. */
static uint8_t Filter_WindowMoving(int16_t *accel, int16_t *gyro)
{
    uint8_t i;
    if (g_calibCount == 0U) return 0U;
    for (i = 0U; i < 3U; i++)
    {
        if ((int32_t)accel[i] - g_calibAccelMin[i] > ROBOT_IMU_CALIB_ACCEL_SPAN ||
            (int32_t)g_calibAccelMax[i] - accel[i] > ROBOT_IMU_CALIB_ACCEL_SPAN ||
            (int32_t)gyro[i] - g_calibGyroMin[i] > ROBOT_IMU_CALIB_GYRO_SPAN ||
            (int32_t)g_calibGyroMax[i] - gyro[i] > ROBOT_IMU_CALIB_GYRO_SPAN)
            return 1U;
    }
    return 0U;
}

/*
 * Fast sqrt via integer initial guess + 2 Newton iterations.
 * Uses union for type-punning (safe across all compilers).
 */
static float fast_sqrt(float x)
{
    union { float f; int32_t i; } u;
    float r;

    if (x < 0.0001f) return 0.0f;

    /* Bit-level half exponent seed via union (no strict-aliasing violation) */
    u.f = x;
    u.i = (u.i >> 1) + 0x1FC00000;
    r = u.f;

    /* Two Newton-Raphson iterations */
    r = 0.5f * (r + x / r);
    r = 0.5f * (r + x / r);
    return r;
}

void Algo_Filter_Init(void)
{
    g_debugCalibRejectMask = 0U;
    g_debugCalibRejectNorm = 0.0f;
    g_debugCalibRejectGyro = 0.0f;
    g_sampleValid = 0U;
    g_seeded = 0U;
    g_tilt = 0.0f;
    g_calibrating = 0U;
    g_calibCount = 0U;
    g_att.pitch       = 0.0f;
    g_att.roll        = 0.0f;
    g_att.pitchAccel  = 0.0f;
    g_att.rollAccel   = 0.0f;
    g_att.gyroOffsetX = 0.0f;
    g_att.gyroOffsetY = 0.0f;
    g_att.gyroOffsetZ = 0.0f;
}

/*
 * Start gyro bias calibration. Subsequent calls to Algo_Filter_Update()
 * will accumulate raw gyro samples instead of running the filter.
 * Called once at boot during INIT state.
 */
void Algo_Filter_StartCalibration(void)
{
    g_calibSumX   = 0;
    g_calibSumY   = 0;
    g_calibSumZ   = 0;
    g_calibCount  = 0;
    g_calibrating = 1;
    g_seeded = 0U;
}

/*
 * Finish calibration: compute average offset from accumulated samples.
 * The robot MUST be stationary during the calibration window (INIT state).
 */
void Algo_Filter_FinishCalibration(void)
{
    if (g_calibCount > 0)
    {
        g_att.gyroOffsetX = (float)g_calibSumX / (float)g_calibCount;
        g_att.gyroOffsetY = (float)g_calibSumY / (float)g_calibCount;
        g_att.gyroOffsetZ = (float)g_calibSumZ / (float)g_calibCount;
    }
    g_calibrating = 0;
}

void Algo_Filter_Update(int16_t *accel, int16_t *gyro, float dt)
{
    float ax, ay, az, gx, gy, gz;
    float accelPitch, accelRoll;
    float alpha;
    float normSquared;
    uint8_t windowMoving;
    uint8_t i;

    /* The caller maps sensor +X-up/+Y-bow to +Z-up coordinates.
     * AFS_SEL=3 is 2048 LSB/g. Ratios below do not depend on scale. */
    ax = (float)accel[0] / 2048.0f;
    ay = (float)accel[1] / 2048.0f;
    az = (float)accel[2] / 2048.0f;
    normSquared = ax * ax + ay * ay + az * az;
    g_sampleValid = (normSquared >= 0.25f && normSquared <= 2.25f);
    if (!g_sampleValid)
    {
        g_debugCalibRejectMask |= 4U;
        g_debugCalibRejectNorm = normSquared;
        return;
    }

    g_tilt = fast_atan2(fast_sqrt(ax * ax + ay * ay), az) * RAD_TO_DEG;
    windowMoving = g_calibrating ? Filter_WindowMoving(accel, gyro) : 0U;
    if (g_calibrating &&
        (normSquared < ROBOT_IMU_CALIB_NORM_MIN_SQ ||
         normSquared > ROBOT_IMU_CALIB_NORM_MAX_SQ ||
         Filter_Abs((float)gyro[0]) > ROBOT_IMU_CALIB_GYRO_MAX_RAW ||
         Filter_Abs((float)gyro[1]) > ROBOT_IMU_CALIB_GYRO_MAX_RAW ||
         Filter_Abs((float)gyro[2]) > ROBOT_IMU_CALIB_GYRO_MAX_RAW || windowMoving))
    {
        if (windowMoving) g_debugCalibRejectMask |= 8U;
        if (normSquared < ROBOT_IMU_CALIB_NORM_MIN_SQ ||
            normSquared > ROBOT_IMU_CALIB_NORM_MAX_SQ)
        {
            g_debugCalibRejectMask |= 1U;
            g_debugCalibRejectNorm = normSquared;
        }
        if (Filter_Abs((float)gyro[0]) > ROBOT_IMU_CALIB_GYRO_MAX_RAW ||
            Filter_Abs((float)gyro[1]) > ROBOT_IMU_CALIB_GYRO_MAX_RAW ||
            Filter_Abs((float)gyro[2]) > ROBOT_IMU_CALIB_GYRO_MAX_RAW)
        {
            float maxGyro = Filter_Abs((float)gyro[0]);
            if (Filter_Abs((float)gyro[1]) > maxGyro)
                maxGyro = Filter_Abs((float)gyro[1]);
            if (Filter_Abs((float)gyro[2]) > maxGyro)
                maxGyro = Filter_Abs((float)gyro[2]);
            g_debugCalibRejectMask |= 2U;
            g_debugCalibRejectGyro = maxGyro;
        }
        /* Require a consecutive stationary window for gyro bias. */
        g_calibSumX = 0;
        g_calibSumY = 0;
        g_calibSumZ = 0;
        g_calibCount = 0U;
        return;
    }

    /* During calibration: accumulate raw gyro, skip filter */
    if (g_calibrating)
    {
        for (i = 0U; i < 3U; i++)
        {
            if (g_calibCount == 0U)
            {
                g_calibAccelMin[i] = g_calibAccelMax[i] = accel[i];
                g_calibGyroMin[i] = g_calibGyroMax[i] = gyro[i];
            }
            else
            {
                if (accel[i] < g_calibAccelMin[i]) g_calibAccelMin[i] = accel[i];
                if (accel[i] > g_calibAccelMax[i]) g_calibAccelMax[i] = accel[i];
                if (gyro[i] < g_calibGyroMin[i]) g_calibGyroMin[i] = gyro[i];
                if (gyro[i] > g_calibGyroMax[i]) g_calibGyroMax[i] = gyro[i];
            }
        }
        g_calibSumX += gyro[0];
        g_calibSumY += gyro[1];
        g_calibSumZ += gyro[2];
        g_calibCount++;
        return;
    }

    /* ±16g → 4096 LSB/g */
    ax = (float)accel[0] / 4096.0f;
    ay = (float)accel[1] / 4096.0f;
    az = (float)accel[2] / 4096.0f;

    /* ±2000°/s → 0.060975 °/s per LSB */
    gx = (float)(gyro[0] - g_att.gyroOffsetX) * 0.060975f;
    gy = (float)(gyro[1] - g_att.gyroOffsetY) * 0.060975f;
    gz = (float)(gyro[2] - g_att.gyroOffsetZ) * 0.060975f;
    (void)gz;

    /* Accel-based angles */
    accelPitch = fast_atan2(ay, fast_sqrt(ax * ax + az * az)) * 57.29578f;
    accelRoll  = fast_atan2(-ax, az) * 57.29578f;

    if (!g_seeded)
    {
        g_att.pitch = accelPitch;
        g_att.roll = accelRoll;
        g_seeded = 1U;
    }

    /* Complementary filter with a dt-dependent coefficient (see header).
     * alpha = tau/(tau+dt); gyro integrates, accel corrects the drift. */
    alpha = FILTER_TAU / (FILTER_TAU + dt);
    g_att.pitch = alpha * (g_att.pitch + gx * dt) + (1.0f - alpha) * accelPitch;
    g_att.roll  = alpha * (g_att.roll  + gy * dt) + (1.0f - alpha) * accelRoll;

    g_att.pitchAccel = accelPitch;
    g_att.rollAccel  = accelRoll;
}

float Algo_Filter_GetPitch(void)  { return g_att.pitch; }
float Algo_Filter_GetRoll(void)   { return g_att.roll; }

const Attitude_t *Algo_Filter_GetAttitude(void) { return &g_att; }

uint8_t Algo_Filter_SampleValid(void) { return g_sampleValid; }
uint8_t Algo_Filter_IsReady(void)
{
    return !g_calibrating && g_seeded &&
           (g_calibCount >= ROBOT_IMU_MIN_CALIB_SAMPLES);
}
uint16_t Algo_Filter_GetCalibCount(void) { return g_calibCount; }
float Algo_Filter_GetTilt(void) { return g_tilt; }
