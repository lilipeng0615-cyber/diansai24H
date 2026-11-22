#include "headfile.h"

extern Car_t car;

#define KEY_PRESSED     1U
#define KEY_UNPRESSED   0U

/* 单位为毫秒，依次为首次松开后的双击等待时间、长按阈值和长按后的重复触发间隔。 */
#define KEY_TIME_DOUBLE 200U
#define KEY_TIME_LONG   2000U
#define KEY_TIME_REPEAT 100U

volatile uint8_t Key_Flag[KEY_COUNT];
static uint8_t Key_CurrState[KEY_COUNT];
static uint8_t Key_State[KEY_COUNT];
static uint16_t Key_Time[KEY_COUNT];

/* 清除待处理事件和计时。仍按住的键保留 HOLD，等松开后才识别下一次操作。 */
static void key_clear_all_events(void)
{
    uint8_t i;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    for (i = 0U; i < KEY_COUNT; i++) {
        Key_Flag[i] &= KEY_FLAG_HOLD;
        Key_Time[i] = 0U;
        Key_State[i] = (Key_CurrState[i] == KEY_PRESSED) ? 3U : 0U;
    }
    __set_PRIMASK(primask);
}

/* n 为实体按键编号 1～3（K1～K3）。低电平表示按下，无效编号按松开处理。 */
uint8_t read_pin(uint8_t n)
{
    if (n == 1U) {
        if (DL_GPIO_readPins(Key_PORT, Key_PIN_3_PIN) == 0U) {
            return KEY_PRESSED;
        }
    } else if (n == 2U) {
        if (DL_GPIO_readPins(Key_PORT, Key_PIN_4_PIN) == 0U) {
            return KEY_PRESSED;
        }
    } else if (n == 3U) {
        if (DL_GPIO_readPins(Key_PORT, Key_PIN_5_PIN) == 0U) {
            return KEY_PRESSED;
        }
    }

    return KEY_UNPRESSED;
}

/* n 为数组下标 0～2，对应 K1～K3。请求的标志中只要有一位置位就返回 1，否则返回 0。 */
uint8_t Key_check(uint8_t n, uint8_t flag)
{
    uint8_t hit = 0U;
    uint8_t clear_mask;
    uint32_t primask;

    if ((n >= KEY_COUNT) || (flag == 0U)) {
        return 0U;
    }

    /* 命中的事件标志读后清除。HOLD 表示最近一次采样的按住状态，可重复查询。 */
    clear_mask = (uint8_t)(flag & (uint8_t)(~KEY_FLAG_HOLD));

    /* 在同一临界区内读取并清除标志，避免漏掉 SysTick 中断新置位的事件。 */
    primask = __get_PRIMASK();
    __disable_irq();
    if ((Key_Flag[n] & flag) != 0U) {
        Key_Flag[n] &= (uint8_t)(~clear_mask);
        hit = 1U;
    }
    __set_PRIMASK(primask);
    return hit;
}

/* SysTick 每 1 ms 更新各键的倒计时，每隔 20 ms 采样并更新按键状态。 */
void Key_Tick(void)
{
    static uint8_t Count, i;
    static uint8_t PrevState[KEY_COUNT];

    for (i = 0U; i < KEY_COUNT; i++) {
        if (Key_Time[i] > 0U) {
            Key_Time[i]--;
        }
    }

    Count++;
    if (Count >= 20U) {
        Count = 0U;
        for (i = 0U; i < KEY_COUNT; i++) {
            PrevState[i] = Key_CurrState[i];
            Key_CurrState[i] = read_pin((uint8_t)(i + 1U));

            /* HOLD 跟随当前采样值，DOWN 和 UP 分别记录相邻两次采样间的按下和松开。 */
            if (Key_CurrState[i] == KEY_PRESSED) {
                Key_Flag[i] |= KEY_FLAG_HOLD;
            } else {
                Key_Flag[i] &= (uint8_t)(~KEY_FLAG_HOLD);
            }
            if ((Key_CurrState[i] == KEY_PRESSED) && (PrevState[i] == KEY_UNPRESSED)) {
                Key_Flag[i] |= KEY_FLAG_DOWN;
            }
            if ((Key_CurrState[i] == KEY_UNPRESSED) && (PrevState[i] == KEY_PRESSED)) {
                Key_Flag[i] |= KEY_FLAG_UP;
            }

            if (Key_State[i] == 0U) {
                /* 空闲时检测到首次按下，开始长按计时。 */
                if (Key_CurrState[i] == KEY_PRESSED) {
                    Key_Time[i] = KEY_TIME_LONG;
                    Key_State[i] = 1U;
                }
            } else if (Key_State[i] == 1U) {
                /* 首次按下后，松开就开始等待双击，持续按到阈值则产生一次长按事件。 */
                if (Key_CurrState[i] == KEY_UNPRESSED) {
                    Key_Time[i] = KEY_TIME_DOUBLE;
                    Key_State[i] = 2U;
                } else if (Key_Time[i] == 0U) {
                    Key_Time[i] = KEY_TIME_REPEAT;
                    Key_Flag[i] |= KEY_FLAG_LONG;
                    Key_State[i] = 4U;
                }
            } else if (Key_State[i] == 2U) {
                /* 再次按下即确认双击。等待期结束仍是松开状态，才确认单击。 */
                if (Key_CurrState[i] == KEY_PRESSED) {
                    Key_Flag[i] |= KEY_FLAG_DOUBLE;
                    Key_State[i] = 3U;
                } else if (Key_Time[i] == 0U) {
                    Key_Flag[i] |= KEY_FLAG_SINGLE;
                    Key_State[i] = 0U;
                }
            } else if (Key_State[i] == 3U) {
                /* 等待松开，避免把同一次按住识别为下一次操作。 */
                if (Key_CurrState[i] == KEY_UNPRESSED) {
                    Key_State[i] = 0U;
                }
            } else if (Key_State[i] == 4U) {
                /* 长按已触发，继续按住时每隔 KEY_TIME_REPEAT 毫秒产生一次重复事件。 */
                if (Key_CurrState[i] == KEY_UNPRESSED) {
                    Key_State[i] = 0U;
                } else if (Key_Time[i] == 0U) {
                    Key_Time[i] = KEY_TIME_REPEAT;
                    Key_Flag[i] |= KEY_FLAG_REPEAT;
                    Key_State[i] = 4U;
                }
            }
        }
    }
}

void Key_Task_Handle(void)
{
    if (car.task == NULL) {
        return;
    }

    /* 发车倒计时或运行期间，任一键的按下事件都会触发急停，无需等到松开。 */
    if (car.task->Carstartflag != 0) {
        if (Key_check(0U, KEY_FLAG_DOWN) ||
            Key_check(1U, KEY_FLAG_DOWN) ||
            Key_check(2U, KEY_FLAG_DOWN)) {
            car.task->Carstartflag = 0;
            car.task->StartDelayTicks = 0;
            car.task->TaskState = 0;
            key_clear_all_events();
            MotorStop(car.Motors);
            BuzzerBee(car.buzzer);
            LCD_ShowStatusLine("E-STOP!", LCD_RED);
        }
        return;
    }

    /* 空闲时长按 K1，先停电机，再依次采样白底和黑线，校准灰度传感器。 */
    if (Key_check(0U, KEY_FLAG_LONG)) {
        key_clear_all_events();
        MotorStop(car.Motors);
        GraySensor_RunCalibration(car.GraySensor_t, car.buzzer);
        return;
    }

    /* 单击 K1、K2、K3 分别选择任务 1、2、3，双击 K3 选择任务 4。选定后进入发车倒计时。 */
    if (Key_check(0U, KEY_FLAG_SINGLE)) {
        key_clear_all_events();
        car.task->TaskFlag = 1;
        car.task->TaskState = 0;
        car.task->StartDelayTicks = TASK_START_DELAY_TICKS;
        car.task->Carstartflag = 1;
        BuzzerBee(car.buzzer);
        LCD_ShowStatusLine("T1 ARMING...", LCD_YELLOW);
    } else if (Key_check(1U, KEY_FLAG_SINGLE)) {
        key_clear_all_events();
        car.task->TaskFlag = 2;
        car.task->TaskState = 0;
        car.task->StartDelayTicks = TASK_START_DELAY_TICKS;
        car.task->Carstartflag = 1;
        BuzzerBee(car.buzzer);
        LCD_ShowStatusLine("T2 ARMING...", LCD_YELLOW);
    } else if (Key_check(2U, KEY_FLAG_SINGLE)) {
        key_clear_all_events();
        car.task->TaskFlag = 3;
        car.task->TaskState = 0;
        car.task->StartDelayTicks = TASK_START_DELAY_TICKS;
        car.task->Carstartflag = 1;
        BuzzerBee(car.buzzer);
        LCD_ShowStatusLine("T3 ARMING...", LCD_YELLOW);
    } else if (Key_check(2U, KEY_FLAG_DOUBLE)) {
        key_clear_all_events();
        car.task->TaskFlag = 4;
        car.task->TaskState = 0;
        car.task->StartDelayTicks = TASK_START_DELAY_TICKS;
        car.task->Carstartflag = 1;
        BuzzerBee(car.buzzer);
        LCD_ShowStatusLine("T4 ARMING...", LCD_YELLOW);
    }
}
