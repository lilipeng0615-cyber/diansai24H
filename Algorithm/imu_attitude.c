#include "headfile.h"

#define IMU_NOMINAL_DT            0.020f
#define IMU_MAX_VALID_DT          0.100f
#define IMU_INITIAL_ALIGN_SAMPLES 50U
#define IMU_SENSOR_SAMPLE_MS      20U

IMU_Attitude_t IMU_Attitude;
static VQF_t vqf;
static float yaw_last;
static uint8_t static_bias_enable = 1U;

static float wrap_pi(float angle)
{
    while (angle > PI) {
        angle -= 2.0f * PI;
    }
    while (angle < -PI) {
        angle += 2.0f * PI;
    }
    return angle;
}

void IMU_Attitude_SetStaticBiasEnable(uint8_t enable)
{
    uint8_t next = (enable != 0U) ? 1U : 0U;
    if (static_bias_enable != next) {
        static_bias_enable = next;
        VQF_SetBiasEstimationEnable(&vqf, true, (static_bias_enable != 0U));
    }
}

void IMU_Attitude_Init(void)
{
    VQF_Param_t param;
    float acc_avg[3] = {0.0f, 0.0f, 0.0f};
    uint16_t valid_samples = 0U;

    memset(&IMU_Attitude, 0, sizeof(IMU_Attitude));

    VQF_DefaultParam(&param);
    param.motion_bias_est_enabled = true;
    param.rest_bias_est_enabled = (static_bias_enable != 0U);
    VQF_Init(&vqf, &param, IMU_NOMINAL_DT, IMU_NOMINAL_DT);

    /* 上电静止采样约 1s 取平均重力方向，完成初始倾角快速对齐 */
    for (uint16_t sample = 0U; sample < IMU_INITIAL_ALIGN_SAMPLES; sample++) {
        ICM42688_Read(&ICM42688);
        if (ICM42688.LastAccelSaturated == 0U) {
            acc_avg[0] += ICM42688.Accel[0];
            acc_avg[1] += ICM42688.Accel[1];
            acc_avg[2] += ICM42688.Accel[2];
            valid_samples++;
        }
        delay_ms(IMU_SENSOR_SAMPLE_MS);
    }

    if (valid_samples > 0U) {
        acc_avg[0] /= (float)valid_samples;
        acc_avg[1] /= (float)valid_samples;
        acc_avg[2] /= (float)valid_samples;
        VQF_UpdateAccel(&vqf, acc_avg);
    }

    VQF_GetQuat6D(&vqf, IMU_Attitude.q);
    VQF_GetEulerZYX(&vqf, &IMU_Attitude.Roll, &IMU_Attitude.Pitch, &IMU_Attitude.Yaw);
    IMU_Attitude.Yaw = wrap_pi(IMU_Attitude.Yaw);
    IMU_Attitude.YawTotalAngle = 0.0f;
    IMU_Attitude.YawRoundCount = 0;
    yaw_last = IMU_Attitude.Yaw;
}

void IMU_Attitude_Update(float dt)
{
    float yaw_delta;

    if ((dt <= 0.0f) || (dt > IMU_MAX_VALID_DT)) {
        dt = IMU_NOMINAL_DT;
    }

    ICM42688_Read(&ICM42688);

    if (ICM42688.LastGyroSaturated != 0U) {
        return;
    }

    VQF_UpdateGyro(&vqf, ICM42688.Gyro, dt);
    if (ICM42688.LastAccelSaturated == 0U) {
        VQF_UpdateAccel(&vqf, ICM42688.Accel);
    }

    VQF_GetQuat6D(&vqf, IMU_Attitude.q);
    VQF_GetEulerZYX(&vqf, &IMU_Attitude.Roll, &IMU_Attitude.Pitch, &IMU_Attitude.Yaw);
    IMU_Attitude.Yaw = wrap_pi(IMU_Attitude.Yaw);

    yaw_delta = IMU_Attitude.Yaw - yaw_last;
    if (yaw_delta > PI) {
        IMU_Attitude.YawRoundCount--;
    } else if (yaw_delta < -PI) {
        IMU_Attitude.YawRoundCount++;
    }

    /* 累计最短角度增量，跨越 +/-pi 时保持航向角连续 */
    IMU_Attitude.YawTotalAngle += wrap_pi(yaw_delta);
    yaw_last = IMU_Attitude.Yaw;
}
