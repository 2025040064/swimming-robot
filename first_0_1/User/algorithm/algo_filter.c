#include "algo_filter.h"

static Attitude_t g_att;

/*
 * Fast atan2 approximation — max error ~0.001 rad
 * No math library dependency
 */
static float fast_atan2(float y, float x)
{
    float r, angle, abs_y;

    abs_y = (y < 0.0f) ? -y : y;

    if (x >= 0.0f)
    {
        r = (x - abs_y) / (x + abs_y + 0.000001f);
        angle = 0.1963f * r * r * r - 0.9817f * r + 3.14159265f / 4.0f;
    }
    else
    {
        r = (x + abs_y) / (abs_y - x + 0.000001f);
        angle = 0.1963f * r * r * r - 0.9817f * r + 3.0f * 3.14159265f / 4.0f;
    }

    return (y < 0.0f) ? -angle : angle;
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
    g_att.pitch       = 0.0f;
    g_att.roll        = 0.0f;
    g_att.pitchAccel  = 0.0f;
    g_att.rollAccel   = 0.0f;
    g_att.gyroOffsetX = 0.0f;
    g_att.gyroOffsetY = 0.0f;
    g_att.gyroOffsetZ = 0.0f;
}

void Algo_Filter_Update(int16_t *accel, int16_t *gyro, float dt)
{
    float ax, ay, az, gx, gy, gz;
    float accelPitch, accelRoll;

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

    /* Complementary filter: α=0.96 gyro + 0.04 accel */
    g_att.pitch = 0.96f * (g_att.pitch + gx * dt) + 0.04f * accelPitch;
    g_att.roll  = 0.96f * (g_att.roll  + gy * dt) + 0.04f * accelRoll;

    g_att.pitchAccel = accelPitch;
    g_att.rollAccel  = accelRoll;
}

float Algo_Filter_GetPitch(void)  { return g_att.pitch; }
float Algo_Filter_GetRoll(void)   { return g_att.roll; }

const Attitude_t *Algo_Filter_GetAttitude(void) { return &g_att; }
