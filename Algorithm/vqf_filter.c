#include "vqf_filter.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define VQF_PI_F    3.14159265358979323846f
#define VQF_PI_D    ((double)3.14159265358979323846L)
#define VQF_SQRT2_D ((double)1.41421356237309504880L)
#define VQF_EPS     1.0e-9f

static float vqf_constrain(float x, float min_val, float max_val)
{
    if (x < min_val) {
        return min_val;
    }
    if (x > max_val) {
        return max_val;
    }
    return x;
}

static bool vqf_is_invalid_float(float x)
{
    return (!isfinite(x));
}

static bool vqf_vec3_is_valid(const float v[3])
{
    return (!vqf_is_invalid_float(v[0]) &&
            !vqf_is_invalid_float(v[1]) &&
            !vqf_is_invalid_float(v[2]));
}

static float vqf_vec3_norm(const float v[3])
{
    return sqrtf((v[0] * v[0]) + (v[1] * v[1]) + (v[2] * v[2]));
}

static void vqf_quat_normalize(float q[4])
{
    float norm = sqrtf((q[0] * q[0]) + (q[1] * q[1]) + (q[2] * q[2]) + (q[3] * q[3]));
    if ((norm <= VQF_EPS) || vqf_is_invalid_float(norm)) {
        q[0] = 1.0f;
        q[1] = 0.0f;
        q[2] = 0.0f;
        q[3] = 0.0f;
        return;
    }
    float inv = 1.0f / norm;
    q[0] *= inv;
    q[1] *= inv;
    q[2] *= inv;
    q[3] *= inv;
}

static void vqf_quat_mul(const float a[4], const float b[4], float out[4])
{
    float w = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
    float x = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
    float y = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
    float z = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
    out[0] = w;
    out[1] = x;
    out[2] = y;
    out[3] = z;
}

static void vqf_quat_to_rotmat(const float q_in[4], float r[3][3])
{
    float q[4] = {q_in[0], q_in[1], q_in[2], q_in[3]};
    vqf_quat_normalize(q);

    float q0q1 = q[0] * q[1];
    float q0q2 = q[0] * q[2];
    float q0q3 = q[0] * q[3];
    float q1q1 = q[1] * q[1];
    float q1q2 = q[1] * q[2];
    float q1q3 = q[1] * q[3];
    float q2q2 = q[2] * q[2];
    float q2q3 = q[2] * q[3];
    float q3q3 = q[3] * q[3];

    r[0][0] = 1.0f - 2.0f * (q2q2 + q3q3);
    r[0][1] = 2.0f * (q1q2 - q0q3);
    r[0][2] = 2.0f * (q1q3 + q0q2);
    r[1][0] = 2.0f * (q1q2 + q0q3);
    r[1][1] = 1.0f - 2.0f * (q1q1 + q3q3);
    r[1][2] = 2.0f * (q2q3 - q0q1);
    r[2][0] = 2.0f * (q1q3 - q0q2);
    r[2][1] = 2.0f * (q2q3 + q0q1);
    r[2][2] = 1.0f - 2.0f * (q1q1 + q2q2);
}

static void vqf_mat3_mul_vec3(const float m[3][3], const float v[3], float out[3])
{
    float x = m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2];
    float y = m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2];
    float z = m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2];
    out[0] = x;
    out[1] = y;
    out[2] = z;
}

static void vqf_mat3_mul(const float a[3][3], const float b[3][3], float out[3][3])
{
    float res[3][3];
    for (uint32_t i = 0U; i < 3U; i++) {
        for (uint32_t j = 0U; j < 3U; j++) {
            res[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j];
        }
    }
    memcpy(out, res, sizeof(res));
}

static void vqf_mat3_mul_transpose(const float a[3][3], const float b[3][3], float out[3][3])
{
    float res[3][3];
    for (uint32_t i = 0U; i < 3U; i++) {
        for (uint32_t j = 0U; j < 3U; j++) {
            res[i][j] = a[i][0] * b[j][0] + a[i][1] * b[j][1] + a[i][2] * b[j][2];
        }
    }
    memcpy(out, res, sizeof(res));
}

static bool vqf_inverse_mat3(const float input[3][3], float output[3][3])
{
    double a = (double)input[0][0];
    double b = (double)input[0][1];
    double c = (double)input[0][2];
    double d = (double)input[1][0];
    double e = (double)input[1][1];
    double f = (double)input[1][2];
    double g = (double)input[2][0];
    double h = (double)input[2][1];
    double i = (double)input[2][2];

    double cof_a = e * i - f * h;
    double cof_b = c * h - b * i;
    double cof_c = b * f - c * e;
    double cof_d = f * g - d * i;
    double cof_e = a * i - c * g;
    double cof_f = c * d - a * f;
    double cof_g = d * h - e * g;
    double cof_h = b * g - a * h;
    double cof_i = a * e - b * d;
    double det = a * cof_a + b * cof_d + c * cof_g;

    if (fabs(det) <= (double)1.0e-12L) {
        memset(output, 0, sizeof(float) * 9U);
        return false;
    }

    double inv_det = (double)1.0L / det;
    output[0][0] = (float)(cof_a * inv_det);
    output[0][1] = (float)(cof_b * inv_det);
    output[0][2] = (float)(cof_c * inv_det);
    output[1][0] = (float)(cof_d * inv_det);
    output[1][1] = (float)(cof_e * inv_det);
    output[1][2] = (float)(cof_f * inv_det);
    output[2][0] = (float)(cof_g * inv_det);
    output[2][1] = (float)(cof_h * inv_det);
    output[2][2] = (float)(cof_i * inv_det);
    return true;
}

static void vqf_calc_filter_coeff(float tau, float dt, double b[3], double a[2])
{
    if (tau < 0.5f * dt) {
        b[0] = (double)1.0L;
        b[1] = (double)0.0L;
        b[2] = (double)0.0L;
        a[0] = (double)0.0L;
        a[1] = (double)0.0L;
        return;
    }

    double fc = VQF_SQRT2_D / ((double)2.0L * VQF_PI_D * (double)tau);
    double k = tan(VQF_PI_D * fc * (double)dt);
    double k2 = k * k;
    double denom = k2 + VQF_SQRT2_D * k + (double)1.0L;
    double b0 = k2 / denom;

    b[0] = b0;
    b[1] = (double)2.0L * b0;
    b[2] = b0;
    a[0] = (double)2.0L * (k2 - (double)1.0L) / denom;
    a[1] = ((double)1.0L - VQF_SQRT2_D * k + k2) / denom;
}

static void vqf_reset_filter_state(uint32_t length, double state[])
{
    double invalid = (double)NAN;
    for (uint32_t i = 0U; i < 2U * length; i++) {
        state[i] = invalid;
    }
}

static void vqf_filter_vector(const float input[],
                              uint32_t length,
                              float tau,
                              float dt,
                              const double b[3],
                              const double a[2],
                              double state[],
                              float output[])
{
    if (isnan(state[0])) {
        if (isnan(state[1])) {
            state[1] = (double)0.0L;
            for (uint32_t i = 0U; i < length; i++) {
                state[2U + i] = (double)0.0L;
            }
        }

        state[1] += (double)1.0L;
        for (uint32_t i = 0U; i < length; i++) {
            state[2U + i] += (double)input[i];
            output[i] = (float)(state[2U + i] / state[1]);
        }

        if ((float)state[1] * dt >= tau) {
            for (uint32_t i = 0U; i < length; i++) {
                state[2U * i] = (double)output[i] * ((double)1.0L - b[0]);
                state[2U * i + 1U] = (double)output[i] * (b[2] - a[1]);
            }
        }
        return;
    }

    for (uint32_t i = 0U; i < length; i++) {
        double in_val = (double)input[i];
        double out_val = b[0] * in_val + state[2U * i];
        state[2U * i] = b[1] * in_val - a[0] * out_val + state[2U * i + 1U];
        state[2U * i + 1U] = b[2] * in_val - a[1] * out_val;
        output[i] = (float)out_val;
    }
}

static void vqf_calculate_coefficients(VQF_t *vqf)
{
    vqf_calc_filter_coeff(vqf->param.tau_accel, vqf->accel_dt,
                          vqf->accel_lp_b, vqf->accel_lp_a);
    vqf_calc_filter_coeff(vqf->param.rest_filter_tau, vqf->gyro_dt,
                          vqf->rest_gyro_lp_b, vqf->rest_gyro_lp_a);
    vqf_calc_filter_coeff(vqf->param.rest_filter_tau, vqf->accel_dt,
                          vqf->rest_accel_lp_b, vqf->rest_accel_lp_a);

    float bias_sigma_init = vqf->param.bias_sigma_init_deg_s * 100.0f;
    vqf->bias_cov_init = bias_sigma_init * bias_sigma_init;
    vqf->bias_process_noise = 100.0f * vqf->accel_dt / vqf->param.bias_forgetting_time;

    float motion_cov = vqf->param.bias_sigma_motion_deg_s *
                       vqf->param.bias_sigma_motion_deg_s * 10000.0f;
    vqf->bias_motion_noise = (motion_cov * motion_cov) / vqf->bias_process_noise + motion_cov;

    float vert_factor = vqf->param.bias_vertical_forgetting_factor;
    if (vert_factor < 1.0e-10f) {
        vert_factor = 1.0e-10f;
    }
    vqf->bias_vertical_noise = vqf->bias_motion_noise / vert_factor;

    float rest_cov = vqf->param.bias_sigma_rest_deg_s *
                     vqf->param.bias_sigma_rest_deg_s * 10000.0f;
    vqf->bias_rest_noise = (rest_cov * rest_cov) / vqf->bias_process_noise + rest_cov;
}

static void vqf_update_bias(VQF_t *vqf, const float accel_earth[3])
{
    if (!vqf->param.motion_bias_est_enabled && !vqf->param.rest_bias_est_enabled) {
        return;
    }

    float q6d[4];
    float rot_now[3][3];
    VQF_GetQuat6D(vqf, q6d);
    vqf_quat_to_rotmat(q6d, rot_now);

    float bias_earth_xy[2];
    bias_earth_xy[0] = rot_now[0][0] * vqf->bias_est[0] +
                       rot_now[0][1] * vqf->bias_est[1] +
                       rot_now[0][2] * vqf->bias_est[2];
    bias_earth_xy[1] = rot_now[1][0] * vqf->bias_est[0] +
                       rot_now[1][1] * vqf->bias_est[1] +
                       rot_now[1][2] * vqf->bias_est[2];

    float rot_in[9];
    float rot_lp[9];
    for (uint32_t r = 0U; r < 3U; r++) {
        for (uint32_t c = 0U; c < 3U; c++) {
            rot_in[r * 3U + c] = rot_now[r][c];
        }
    }
    vqf_filter_vector(rot_in, 9U, vqf->param.tau_accel, vqf->accel_dt,
                      vqf->accel_lp_b, vqf->accel_lp_a,
                      vqf->motion_rot_lp_state, rot_lp);

    float bias_lp[2];
    vqf_filter_vector(bias_earth_xy, 2U, vqf->param.tau_accel, vqf->accel_dt,
                      vqf->accel_lp_b, vqf->accel_lp_a,
                      vqf->motion_bias_lp_state, bias_lp);

    float H[3][3];
    for (uint32_t r = 0U; r < 3U; r++) {
        for (uint32_t c = 0U; c < 3U; c++) {
            H[r][c] = rot_lp[r * 3U + c];
        }
    }

    float err[3] = {0.0f, 0.0f, 0.0f};
    float R_diag[3] = {0.0f, 0.0f, 0.0f};
    bool update_enable = false;

    if (vqf->rest_detected && vqf->param.rest_bias_est_enabled) {
        memset(H, 0, sizeof(H));
        H[0][0] = 1.0f;
        H[1][1] = 1.0f;
        H[2][2] = 1.0f;

        err[0] = vqf->rest_last_gyro_lp[0] - vqf->bias_est[0];
        err[1] = vqf->rest_last_gyro_lp[1] - vqf->bias_est[1];
        err[2] = vqf->rest_last_gyro_lp[2] - vqf->bias_est[2];

        R_diag[0] = vqf->bias_rest_noise;
        R_diag[1] = vqf->bias_rest_noise;
        R_diag[2] = vqf->bias_rest_noise;
        update_enable = true;
    } else if (vqf->param.motion_bias_est_enabled) {
        float pred_bias[3];
        vqf_mat3_mul_vec3(H, vqf->bias_est, pred_bias);

        err[0] = -accel_earth[1] / vqf->accel_dt + bias_lp[0] - pred_bias[0];
        err[1] = accel_earth[0] / vqf->accel_dt + bias_lp[1] - pred_bias[1];
        err[2] = -pred_bias[2];

        R_diag[0] = vqf->bias_motion_noise;
        R_diag[1] = vqf->bias_motion_noise;
        R_diag[2] = vqf->bias_vertical_noise;
        update_enable = true;
    }

    for (uint32_t i = 0U; i < 3U; i++) {
        if (vqf->bias_cov[i][i] < vqf->bias_cov_init) {
            vqf->bias_cov[i][i] += vqf->bias_process_noise;
        }
    }

    if (!update_enable) {
        return;
    }

    float bias_clip = vqf->param.bias_clip_deg_s * VQF_PI_F / 180.0f;
    for (uint32_t i = 0U; i < 3U; i++) {
        err[i] = vqf_constrain(err[i], -bias_clip, bias_clip);
    }

    float PHt[3][3];
    float S[3][3];
    float S_inv[3][3];
    float K[3][3];
    float KH[3][3];
    float KHP[3][3];
    float delta_bias[3];

    vqf_mat3_mul_transpose(vqf->bias_cov, H, PHt);
    vqf_mat3_mul(H, PHt, S);
    S[0][0] += R_diag[0];
    S[1][1] += R_diag[1];
    S[2][2] += R_diag[2];

    if (!vqf_inverse_mat3(S, S_inv)) {
        return;
    }

    vqf_mat3_mul(PHt, S_inv, K);
    vqf_mat3_mul_vec3(K, err, delta_bias);
    vqf_mat3_mul(K, H, KH);
    vqf_mat3_mul(KH, vqf->bias_cov, KHP);

    for (uint32_t i = 0U; i < 3U; i++) {
        vqf->bias_est[i] = vqf_constrain(vqf->bias_est[i] + delta_bias[i],
                                         -bias_clip, bias_clip);
        for (uint32_t j = 0U; j < 3U; j++) {
            vqf->bias_cov[i][j] -= KHP[i][j];
        }
    }
}

void VQF_DefaultParam(VQF_Param_t *param)
{
    if (param == NULL) {
        return;
    }

    param->tau_accel = 3.0f;
    param->motion_bias_est_enabled = true;
    param->rest_bias_est_enabled = true;
    param->bias_sigma_init_deg_s = 0.5f;
    param->bias_forgetting_time = 100.0f;
    param->bias_clip_deg_s = 2.0f;
    param->bias_sigma_motion_deg_s = 0.1f;
    param->bias_vertical_forgetting_factor = 0.0001f;
    param->bias_sigma_rest_deg_s = 0.03f;
    param->rest_min_time = 1.5f;
    param->rest_filter_tau = 0.5f;
    param->rest_threshold_gyro_deg_s = 2.0f;
    param->rest_threshold_accel = 0.5f;
}

void VQF_Init(VQF_t *vqf, const VQF_Param_t *param, float gyro_dt, float accel_dt)
{
    if (vqf == NULL) {
        return;
    }

    memset(vqf, 0, sizeof(VQF_t));
    if (param != NULL) {
        vqf->param = *param;
    } else {
        VQF_DefaultParam(&vqf->param);
    }

    vqf->gyro_dt = (gyro_dt > 0.0f) ? gyro_dt : 0.010f;
    vqf->accel_dt = (accel_dt > 0.0f) ? accel_dt : vqf->gyro_dt;

    if (vqf->param.tau_accel <= 0.0f) {
        vqf->param.tau_accel = 3.0f;
    }
    if (vqf->param.rest_filter_tau <= 0.0f) {
        vqf->param.rest_filter_tau = 0.5f;
    }
    if (vqf->param.bias_forgetting_time <= 0.0f) {
        vqf->param.bias_forgetting_time = 100.0f;
    }

    vqf_calculate_coefficients(vqf);
    VQF_Reset(vqf);
    vqf->initialized = true;
}

void VQF_Reset(VQF_t *vqf)
{
    if (vqf == NULL) {
        return;
    }

    vqf->gyro_quat[0] = 1.0f;
    vqf->gyro_quat[1] = 0.0f;
    vqf->gyro_quat[2] = 0.0f;
    vqf->gyro_quat[3] = 0.0f;

    vqf->accel_quat[0] = 1.0f;
    vqf->accel_quat[1] = 0.0f;
    vqf->accel_quat[2] = 0.0f;
    vqf->accel_quat[3] = 0.0f;

    memset(vqf->last_accel_lp, 0, sizeof(vqf->last_accel_lp));
    vqf_reset_filter_state(3U, vqf->accel_lp_state);
    vqf->last_accel_correction_rate = 0.0f;

    memset(vqf->bias_est, 0, sizeof(vqf->bias_est));
    memset(vqf->bias_cov, 0, sizeof(vqf->bias_cov));
    vqf->bias_cov[0][0] = vqf->bias_cov_init;
    vqf->bias_cov[1][1] = vqf->bias_cov_init;
    vqf->bias_cov[2][2] = vqf->bias_cov_init;
    vqf_reset_filter_state(9U, vqf->motion_rot_lp_state);
    vqf_reset_filter_state(2U, vqf->motion_bias_lp_state);

    vqf->rest_detected = false;
    vqf->rest_time = 0.0f;
    vqf->rest_gyro_sq_dev = 0.0f;
    vqf->rest_accel_sq_dev = 0.0f;
    memset(vqf->rest_last_gyro_lp, 0, sizeof(vqf->rest_last_gyro_lp));
    memset(vqf->rest_last_accel_lp, 0, sizeof(vqf->rest_last_accel_lp));
    vqf_reset_filter_state(3U, vqf->rest_gyro_lp_state);
    vqf_reset_filter_state(3U, vqf->rest_accel_lp_state);

    memset(vqf->last_corrected_gyro, 0, sizeof(vqf->last_corrected_gyro));
}

void VQF_UpdateGyro(VQF_t *vqf, const float gyro[3], float dt)
{
    if ((vqf == NULL) || (gyro == NULL) || !vqf->initialized || !vqf_vec3_is_valid(gyro)) {
        return;
    }

    float step_dt = (dt > 0.0f) ? dt : vqf->gyro_dt;

    if (vqf->param.rest_bias_est_enabled) {
        float gyro_lp[3];
        vqf_filter_vector(gyro, 3U, vqf->param.rest_filter_tau, vqf->gyro_dt,
                          vqf->rest_gyro_lp_b, vqf->rest_gyro_lp_a,
                          vqf->rest_gyro_lp_state, gyro_lp);

        vqf->rest_last_gyro_lp[0] = gyro_lp[0];
        vqf->rest_last_gyro_lp[1] = gyro_lp[1];
        vqf->rest_last_gyro_lp[2] = gyro_lp[2];

        float dx = gyro[0] - gyro_lp[0];
        float dy = gyro[1] - gyro_lp[1];
        float dz = gyro[2] - gyro_lp[2];
        vqf->rest_gyro_sq_dev = dx * dx + dy * dy + dz * dz;

        float gyro_th = vqf->param.rest_threshold_gyro_deg_s * VQF_PI_F / 180.0f;
        float bias_clip = vqf->param.bias_clip_deg_s * VQF_PI_F / 180.0f;
        if ((vqf->rest_gyro_sq_dev >= gyro_th * gyro_th) ||
            (fabsf(gyro_lp[0]) > bias_clip) ||
            (fabsf(gyro_lp[1]) > bias_clip) ||
            (fabsf(gyro_lp[2]) > bias_clip)) {
            vqf->rest_time = 0.0f;
            vqf->rest_detected = false;
        }
    }

    vqf->last_corrected_gyro[0] = gyro[0] - vqf->bias_est[0];
    vqf->last_corrected_gyro[1] = gyro[1] - vqf->bias_est[1];
    vqf->last_corrected_gyro[2] = gyro[2] - vqf->bias_est[2];

    float gyro_norm = vqf_vec3_norm(vqf->last_corrected_gyro);
    if (gyro_norm > VQF_EPS) {
        float angle = gyro_norm * step_dt;
        float half_angle = 0.5f * angle;
        float scale = sinf(half_angle) / gyro_norm;
        float step_q[4] = {
            cosf(half_angle),
            scale * vqf->last_corrected_gyro[0],
            scale * vqf->last_corrected_gyro[1],
            scale * vqf->last_corrected_gyro[2]
        };
        float next_q[4];
        vqf_quat_mul(vqf->gyro_quat, step_q, next_q);
        vqf_quat_normalize(next_q);
        memcpy(vqf->gyro_quat, next_q, sizeof(next_q));
    }
}

void VQF_UpdateAccel(VQF_t *vqf, const float accel[3])
{
    if ((vqf == NULL) || (accel == NULL) || !vqf->initialized || !vqf_vec3_is_valid(accel)) {
        return;
    }

    if (vqf_vec3_norm(accel) <= VQF_EPS) {
        return;
    }

    if (vqf->param.rest_bias_est_enabled) {
        float accel_lp[3];
        vqf_filter_vector(accel, 3U, vqf->param.rest_filter_tau, vqf->accel_dt,
                          vqf->rest_accel_lp_b, vqf->rest_accel_lp_a,
                          vqf->rest_accel_lp_state, accel_lp);

        vqf->rest_last_accel_lp[0] = accel_lp[0];
        vqf->rest_last_accel_lp[1] = accel_lp[1];
        vqf->rest_last_accel_lp[2] = accel_lp[2];

        float dx = accel[0] - accel_lp[0];
        float dy = accel[1] - accel_lp[1];
        float dz = accel[2] - accel_lp[2];
        vqf->rest_accel_sq_dev = dx * dx + dy * dy + dz * dz;

        float acc_th = vqf->param.rest_threshold_accel;
        if (vqf->rest_accel_sq_dev >= acc_th * acc_th) {
            vqf->rest_time = 0.0f;
            vqf->rest_detected = false;
        } else {
            vqf->rest_time += vqf->accel_dt;
            if (vqf->rest_time >= vqf->param.rest_min_time) {
                vqf->rest_detected = true;
            }
        }
    }

    float r_gyro[3][3];
    float accel_gyro_earth[3];
    vqf_quat_to_rotmat(vqf->gyro_quat, r_gyro);
    vqf_mat3_mul_vec3(r_gyro, accel, accel_gyro_earth);

    vqf_filter_vector(accel_gyro_earth, 3U, vqf->param.tau_accel, vqf->accel_dt,
                      vqf->accel_lp_b, vqf->accel_lp_a,
                      vqf->accel_lp_state, vqf->last_accel_lp);

    float r_accel[3][3];
    float accel_earth[3];
    vqf_quat_to_rotmat(vqf->accel_quat, r_accel);
    vqf_mat3_mul_vec3(r_accel, vqf->last_accel_lp, accel_earth);

    float norm = vqf_vec3_norm(accel_earth);
    if (norm <= VQF_EPS) {
        return;
    }
    accel_earth[0] /= norm;
    accel_earth[1] /= norm;
    accel_earth[2] /= norm;

    float q_w = sqrtf(vqf_constrain(0.5f * (accel_earth[2] + 1.0f), 0.0f, 1.0f));
    float corr_q[4];
    if (q_w > 1.0e-6f) {
        corr_q[0] = q_w;
        corr_q[1] = 0.5f * accel_earth[1] / q_w;
        corr_q[2] = -0.5f * accel_earth[0] / q_w;
        corr_q[3] = 0.0f;
    } else {
        corr_q[0] = 0.0f;
        corr_q[1] = 1.0f;
        corr_q[2] = 0.0f;
        corr_q[3] = 0.0f;
    }

    float next_accel_q[4];
    vqf_quat_mul(corr_q, vqf->accel_quat, next_accel_q);
    vqf_quat_normalize(next_accel_q);
    memcpy(vqf->accel_quat, next_accel_q, sizeof(next_accel_q));

    vqf->last_accel_correction_rate =
        acosf(vqf_constrain(accel_earth[2], -1.0f, 1.0f)) / vqf->accel_dt;

    vqf_update_bias(vqf, accel_earth);
}

void VQF_Update(VQF_t *vqf, const float gyro[3], const float accel[3], float dt)
{
    VQF_UpdateGyro(vqf, gyro, dt);
    VQF_UpdateAccel(vqf, accel);
}

void VQF_SetBiasEstimate(VQF_t *vqf, const float bias[3], float sigma)
{
    if ((vqf == NULL) || (bias == NULL) || !vqf_vec3_is_valid(bias)) {
        return;
    }

    float bias_clip = vqf->param.bias_clip_deg_s * VQF_PI_F / 180.0f;
    for (uint32_t i = 0U; i < 3U; i++) {
        vqf->bias_est[i] = vqf_constrain(bias[i], -bias_clip, bias_clip);
    }

    if ((sigma > 0.0f) && !vqf_is_invalid_float(sigma)) {
        float sigma_int = sigma * 180.0f * 100.0f / VQF_PI_F;
        float cov = sigma_int * sigma_int;
        memset(vqf->bias_cov, 0, sizeof(vqf->bias_cov));
        vqf->bias_cov[0][0] = cov;
        vqf->bias_cov[1][1] = cov;
        vqf->bias_cov[2][2] = cov;
    }
}

void VQF_SetBiasEstimationEnable(VQF_t *vqf, bool motion_enable, bool rest_enable)
{
    if (vqf == NULL) {
        return;
    }

    bool motion_changed = (vqf->param.motion_bias_est_enabled != motion_enable);
    bool rest_changed = (vqf->param.rest_bias_est_enabled != rest_enable);

    vqf->param.motion_bias_est_enabled = motion_enable;
    vqf->param.rest_bias_est_enabled = rest_enable;

    if (motion_changed) {
        vqf_reset_filter_state(9U, vqf->motion_rot_lp_state);
        vqf_reset_filter_state(2U, vqf->motion_bias_lp_state);
    }
    if (rest_changed) {
        vqf->rest_detected = false;
        vqf->rest_time = 0.0f;
        vqf->rest_gyro_sq_dev = 0.0f;
        vqf->rest_accel_sq_dev = 0.0f;
        vqf_reset_filter_state(3U, vqf->rest_gyro_lp_state);
        vqf_reset_filter_state(3U, vqf->rest_accel_lp_state);
    }
}

void VQF_GetQuat6D(const VQF_t *vqf, float q_out[4])
{
    if (q_out == NULL) {
        return;
    }
    if (vqf == NULL) {
        q_out[0] = 1.0f;
        q_out[1] = 0.0f;
        q_out[2] = 0.0f;
        q_out[3] = 0.0f;
        return;
    }

    vqf_quat_mul(vqf->accel_quat, vqf->gyro_quat, q_out);
    vqf_quat_normalize(q_out);
}

void VQF_GetEulerZYX(const VQF_t *vqf, float *roll, float *pitch, float *yaw)
{
    float q[4];
    VQF_GetQuat6D(vqf, q);

    float sin_pitch = vqf_constrain(2.0f * (q[0] * q[2] - q[3] * q[1]), -1.0f, 1.0f);
    if (pitch != NULL) {
        *pitch = asinf(sin_pitch);
    }
    if (roll != NULL) {
        *roll = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]),
                       1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]));
    }
    if (yaw != NULL) {
        *yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]),
                      1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));
    }
}

void VQF_GetBiasEstimate(const VQF_t *vqf, float bias_out[3])
{
    if ((vqf == NULL) || (bias_out == NULL)) {
        return;
    }
    bias_out[0] = vqf->bias_est[0];
    bias_out[1] = vqf->bias_est[1];
    bias_out[2] = vqf->bias_est[2];
}

float VQF_GetBiasSigma(const VQF_t *vqf)
{
    if (vqf == NULL) {
        return 0.0f;
    }

    float r0 = fabsf(vqf->bias_cov[0][0]) + fabsf(vqf->bias_cov[0][1]) + fabsf(vqf->bias_cov[0][2]);
    float r1 = fabsf(vqf->bias_cov[1][0]) + fabsf(vqf->bias_cov[1][1]) + fabsf(vqf->bias_cov[1][2]);
    float r2 = fabsf(vqf->bias_cov[2][0]) + fabsf(vqf->bias_cov[2][1]) + fabsf(vqf->bias_cov[2][2]);
    float max_row = r0;
    if (r1 > max_row) {
        max_row = r1;
    }
    if (r2 > max_row) {
        max_row = r2;
    }

    float cov = vqf_constrain(max_row, 0.0f, vqf->bias_cov_init);
    return sqrtf(cov) * VQF_PI_F / (100.0f * 180.0f);
}

bool VQF_GetRestDetected(const VQF_t *vqf)
{
    return (vqf != NULL) ? vqf->rest_detected : false;
}
