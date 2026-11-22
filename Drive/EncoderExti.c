#include "headfile.h"

extern Car_t car;

/* 左右轮共用第 1 组 GPIO 中断。每个 A 相上升沿计一次数，计数增减由 B 相电平决定。
 * EncoderDataUpdate() 每 20 ms 取出并清零两轮计数，用于计算轮速和行驶距离。
 */
void GROUP1_IRQHandler(void)
{
    /* 电机或编码器尚未初始化时，直接返回以避免访问空指针。 */
    if ((car.Motors == NULL) ||
        (car.Motors->EncoderLeft == NULL) ||
        (car.Motors->EncoderRight == NULL)) {
        return;
    }

    /* 先确定触发中断的 GPIO 端口，再检查该端口上 A 相引脚的中断标志。 */
    switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1))
    {
        case Encoder1_INT_IIDX:
            if (DL_GPIO_getEnabledInterruptStatus(Encoder1_PORT, Encoder1_A1_PIN) != 0U) {
                DL_GPIO_clearInterruptStatus(Encoder1_PORT, Encoder1_A1_PIN);
                /* 左轮 A1（PB23）的上升沿已确认并清除标志，读取 B1（PB24）：高电平减一，低电平加一。 */
                if (DL_GPIO_readPins(Encoder1_PORT, Encoder1_B1_PIN) != 0U) {
                    car.Motors->EncoderLeft->EncoderCount--;
                } else {
                    car.Motors->EncoderLeft->EncoderCount++;
                }
            }
            break;

        case Encoder2_INT_IIDX:
            if (DL_GPIO_getEnabledInterruptStatus(Encoder2_PORT, Encoder2_A2_PIN) != 0U) {
                DL_GPIO_clearInterruptStatus(Encoder2_PORT, Encoder2_A2_PIN);
                /* 右轮 A2（PA25）的上升沿已确认并清除标志，读取 B2（PA26）：高电平减一，低电平加一。 */
                if (DL_GPIO_readPins(Encoder2_PORT, Encoder2_B2_PIN) != 0U) {
                    car.Motors->EncoderRight->EncoderCount--;
                } else {
                    car.Motors->EncoderRight->EncoderCount++;
                }
            }
            break;

        default:
            break;
    }
}
