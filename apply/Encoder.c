#include "headfile.h"

void EncoderInit(Encoder_t *Encoder)
{
    Encoder->EncoderCount = 0;
    Encoder->t_int = 0;
    Encoder->v_int = 0;

    Encoder->V = 0.0f;
    Encoder->X = 0.0f;
    Encoder->pos = 0.0f;
    Encoder->vel = 0.0f;
}

void EncodersInit(Motors_t *Motor)
{
    static Encoder_t encoderLeftFront;
    static Encoder_t encoderRightFront;

    Motor->EncoderLeft = &encoderLeftFront;
    Motor->EncoderRight = &encoderRightFront;

    EncoderInit(Motor->EncoderLeft);
    EncoderInit(Motor->EncoderRight);

    DL_GPIO_clearInterruptStatus(Encoder1_PORT, Encoder1_A1_PIN);
    DL_GPIO_clearInterruptStatus(Encoder2_PORT, Encoder2_A2_PIN);
    NVIC_ClearPendingIRQ(Encoder1_INT_IRQN);
    NVIC_ClearPendingIRQ(Encoder2_INT_IRQN);
    NVIC_EnableIRQ(Encoder1_INT_IRQN);
    NVIC_EnableIRQ(Encoder2_INT_IRQN);
}

void EncoderDataUpdate(Motors_t *Motor)
{
    uint32_t primask;
    int32_t left_cnt;
    int32_t right_cnt;
    float rad_per_pulse;
    float vel_scale;

    if ((Motor == NULL) ||
        (Motor->EncoderLeft == NULL) ||
        (Motor->EncoderRight == NULL)) {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    left_cnt = Motor->EncoderLeft->EncoderCount;
    right_cnt = Motor->EncoderRight->EncoderCount;
    Motor->EncoderLeft->EncoderCount = 0;
    Motor->EncoderRight->EncoderCount = 0;
    __set_PRIMASK(primask);

    Motor->EncoderLeft->v_int = left_cnt;
    Motor->EncoderRight->v_int = right_cnt;

    /* 1. 原始脉冲直接累加车轮转角(rad)与位移(cm)，不经过速度滤波，防止滞后与丢步 */
    rad_per_pulse = (2.0f * PI) / EncoderLines;
    Motor->EncoderLeft->pos  += (float)left_cnt  * rad_per_pulse;
    Motor->EncoderRight->pos += (float)right_cnt * rad_per_pulse;
    Motor->EncoderLeft->X    += (float)left_cnt  * rad_per_pulse * TireRadius;
    Motor->EncoderRight->X   += (float)right_cnt * rad_per_pulse * TireRadius;

    /* 2. 角速度做单层一阶低通滤波 + 极小死区清零，再换算线速度(cm/s) */
    vel_scale = EncoderReadingFrequency * rad_per_pulse;
    Motor->EncoderLeft->vel =
        ((float)left_cnt * vel_scale) * ENCODER_VEL_LPF_ALPHA +
        Motor->EncoderLeft->vel * (1.0f - ENCODER_VEL_LPF_ALPHA);
    Motor->EncoderRight->vel =
        ((float)right_cnt * vel_scale) * ENCODER_VEL_LPF_ALPHA +
        Motor->EncoderRight->vel * (1.0f - ENCODER_VEL_LPF_ALPHA);

    if (ABS(Motor->EncoderLeft->vel) < ENCODER_VEL_DEADBAND) {
        Motor->EncoderLeft->vel = 0.0f;
    }
    if (ABS(Motor->EncoderRight->vel) < ENCODER_VEL_DEADBAND) {
        Motor->EncoderRight->vel = 0.0f;
    }

    Motor->EncoderLeft->V = Motor->EncoderLeft->vel * TireRadius;
    Motor->EncoderRight->V = Motor->EncoderRight->vel * TireRadius;
}

float EncoderTotalLengthGet(Motors_t *Motors)
{
    if ((Motors == NULL) ||
        (Motors->EncoderLeft == NULL) ||
        (Motors->EncoderRight == NULL)) {
        return 0.0f;
    }

    return (Motors->EncoderLeft->X + Motors->EncoderRight->X) * 0.5f;
}

/* 获取左右轮累计位移差 (cm)：>0 表示右轮走得多(左转/左偏)，<0 表示左轮走得多(右转/右偏) */
float EncoderLengthDiffGet(Motors_t *Motors)
{
    if ((Motors == NULL) ||
        (Motors->EncoderLeft == NULL) ||
        (Motors->EncoderRight == NULL)) {
        return 0.0f;
    }

    return Motors->EncoderRight->X - Motors->EncoderLeft->X;
}

/* 获取单个车轮累计旋转角度 (度) */
float EncoderWheelAngleDegGet(const Encoder_t *Encoder)
{
    if (Encoder == NULL) {
        return 0.0f;
    }

    return Encoder->pos * (180.0f / PI);
}

/* 获取小车累计转弯角度 (rad)：左转为正，右转为负，可直接对接 MotorYawPidCtrl */
float EncoderTurnAngleGet(Motors_t *Motors)
{
    return EncoderLengthDiffGet(Motors) / WheelTrack;
}

/* 获取小车累计转弯角度 (度)：左转为正，右转为负 */
float EncoderTurnAngleDegGet(Motors_t *Motors)
{
    return EncoderTurnAngleGet(Motors) * (180.0f / PI);
}

/* 获取小车实时转弯角速度 (rad/s)：左转为正，右转为负 */
float EncoderTurnRateGet(Motors_t *Motors)
{
    if ((Motors == NULL) ||
        (Motors->EncoderLeft == NULL) ||
        (Motors->EncoderRight == NULL)) {
        return 0.0f;
    }

    return (Motors->EncoderRight->V - Motors->EncoderLeft->V) / WheelTrack;
}

void EncoderLengthClear(Motors_t *Motors)
{
    if ((Motors == NULL) ||
        (Motors->EncoderLeft == NULL) ||
        (Motors->EncoderRight == NULL)) {
        return;
    }

    Motors->EncoderLeft->X = 0.0f;
    Motors->EncoderRight->X = 0.0f;
    Motors->EncoderLeft->pos = 0.0f;
    Motors->EncoderRight->pos = 0.0f;
}
