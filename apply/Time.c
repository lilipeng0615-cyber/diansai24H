#include "headfile.h"

volatile uint8 imuflag = 0;
volatile uint8 controlflag = 0;
volatile uint8 vofaflag = 0;

void Time_Init(void)
{
    /* TIMER_0 (TIMG0): 10ms 中断 2 分频 -> 20ms (50Hz) IMU update */
    DL_TimerG_clearInterruptStatus(TIMER_0_INST, DL_TIMERG_INTERRUPT_ZERO_EVENT);
    DL_TimerG_enableInterrupt(TIMER_0_INST, DL_TIMERG_INTERRUPT_ZERO_EVENT);
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    DL_TimerG_startCounter(TIMER_0_INST);

    /* TIMER_1 (TIMA1): 20ms -> 50Hz control/encoder/task loop */
    DL_TimerA_clearInterruptStatus(TIMER_1_INST, DL_TIMERA_INTERRUPT_ZERO_EVENT);
    DL_TimerA_enableInterrupt(TIMER_1_INST, DL_TIMERA_INTERRUPT_ZERO_EVENT);
    NVIC_ClearPendingIRQ(TIMER_1_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_1_INST_INT_IRQN);
    DL_TimerA_startCounter(TIMER_1_INST);

    /* TIMER_2 (TIMA0): 50ms -> 20Hz VOFA telemetry */
    DL_TimerA_clearInterruptStatus(TIMER_2_INST, DL_TIMERA_INTERRUPT_ZERO_EVENT);
    DL_TimerA_enableInterrupt(TIMER_2_INST, DL_TIMERA_INTERRUPT_ZERO_EVENT);
    NVIC_ClearPendingIRQ(TIMER_2_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_2_INST_INT_IRQN);
    DL_TimerA_startCounter(TIMER_2_INST);
}

/* 10ms 中断 2 分频 -> 20ms (50Hz) IMU */
void TIMER_0_INST_IRQHandler(void)
{
    static uint8_t imu_div = 0U;

    switch (DL_TimerG_getPendingInterrupt(TIMER_0_INST)) {
        case DL_TIMERG_IIDX_ZERO:
            imu_div++;
            if (imu_div >= 2U) {
                imu_div = 0U;
                imuflag = 1;
            }
            break;
        default:
            break;
    }
}

/* 20ms -> 50Hz motor/task control */
void TIMER_1_INST_IRQHandler(void)
{
    switch (DL_TimerA_getPendingInterrupt(TIMER_1_INST)) {
        case DL_TIMERA_IIDX_ZERO:
            controlflag = 1;
            break;
        default:
            break;
    }
}

/* 50ms -> 20Hz VOFA telemetry */
void TIMER_2_INST_IRQHandler(void)
{
    switch (DL_TimerA_getPendingInterrupt(TIMER_2_INST)) {
        case DL_TIMERA_IIDX_ZERO:
            vofaflag = 1;
            break;
        default:
            break;
    }
}
