#include "algo_filter.h"

static Attitude_t g_att;

/* ---- Gyro bias calibration state ---- */
static uint8_t  g_calibrating = 0;
static int32_t  g_calibSumX   = 0;
static int32_t  g_calibSumY   = 0;
static int32_t  g_calibSumZ   = 0;
static uint16_t g_calibCount  = 0;

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

    /* During calibration: accumulate raw gyro, skip filter */
    if (g_calibrating)
    {
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

    /* Accel-based angles */
    accelPitch = fast_atan2(ay, fast_sqrt(ax * ax + az * az)) * 57.29578f;
    accelRoll  = fast_atan2(-ax, az) * 57.29578f;

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
