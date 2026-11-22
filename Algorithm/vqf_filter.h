#ifndef __VQF_FILTER_H
#define __VQF_FILTER_H

#include <stdbool.h>

typedef struct {
    float tau_accel;                       /* 加速度重力方向修正时间常数 (s) */
    bool motion_bias_est_enabled;          /* 运动状态陀螺仪零偏估计开关 */
    bool rest_bias_est_enabled;            /* 静止状态陀螺仪零偏估计开关 */
    float bias_sigma_init_deg_s;           /* 初始零偏标准差 (deg/s) */
    float bias_forgetting_time;            /* 零偏协方差遗忘时间 (s) */
    float bias_clip_deg_s;                 /* 零偏与残差限幅 (deg/s) */
    float bias_sigma_motion_deg_s;         /* 运动零偏观测标准差 (deg/s) */
    float bias_vertical_forgetting_factor; /* 运动时重力轴方向观测权重因子 */
    float bias_sigma_rest_deg_s;           /* 静止零偏观测标准差 (deg/s) */
    float rest_min_time;                   /* 判定静止最短持续时间 (s) */
    float rest_filter_tau;                 /* 静止检测低通滤波时间常数 (s) */
    float rest_threshold_gyro_deg_s;       /* 静止检测陀螺仪高频残差阈值 (deg/s) */
    float rest_threshold_accel;            /* 静止检测加速度高频残差阈值 (m/s^2) */
} VQF_Param_t;

typedef struct {
    /* 初始化参数与固定系数 */
    VQF_Param_t param;
    float gyro_dt;
    float accel_dt;

    double accel_lp_b[3];
    double accel_lp_a[2];
    double rest_gyro_lp_b[3];
    double rest_gyro_lp_a[2];
    double rest_accel_lp_b[3];
    double rest_accel_lp_a[2];

    float bias_cov_init;
    float bias_process_noise;
    float bias_motion_noise;
    float bias_vertical_noise;
    float bias_rest_noise;

    /* 内部状态 */
    bool initialized;
    float gyro_quat[4];
    float accel_quat[4];

    float last_accel_lp[3];
    double accel_lp_state[6];
    float last_accel_correction_rate;

    float bias_est[3];
    float bias_cov[3][3];
    double motion_rot_lp_state[18];
    double motion_bias_lp_state[4];

    bool rest_detected;
    float rest_time;
    float rest_gyro_sq_dev;
    float rest_accel_sq_dev;
    float rest_last_gyro_lp[3];
    float rest_last_accel_lp[3];
    double rest_gyro_lp_state[6];
    double rest_accel_lp_state[6];

    float last_corrected_gyro[3];
} VQF_t;

void VQF_DefaultParam(VQF_Param_t *param);
void VQF_Init(VQF_t *vqf, const VQF_Param_t *param, float gyro_dt, float accel_dt);
void VQF_Reset(VQF_t *vqf);

void VQF_UpdateGyro(VQF_t *vqf, const float gyro[3], float dt);
void VQF_UpdateAccel(VQF_t *vqf, const float accel[3]);
void VQF_Update(VQF_t *vqf, const float gyro[3], const float accel[3], float dt);

void VQF_SetBiasEstimate(VQF_t *vqf, const float bias[3], float sigma);
void VQF_SetBiasEstimationEnable(VQF_t *vqf, bool motion_enable, bool rest_enable);

void VQF_GetQuat6D(const VQF_t *vqf, float q_out[4]);
void VQF_GetEulerZYX(const VQF_t *vqf, float *roll, float *pitch, float *yaw);
void VQF_GetBiasEstimate(const VQF_t *vqf, float bias_out[3]);
float VQF_GetBiasSigma(const VQF_t *vqf);
bool VQF_GetRestDetected(const VQF_t *vqf);

#endif /* __VQF_FILTER_H */
