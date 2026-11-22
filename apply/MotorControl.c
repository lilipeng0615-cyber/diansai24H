#include "headfile.h"


/* 三组参数依次用于轮速、灰度巡线和航向保持，每组按 Kp、Ki、Kd 排列。 */
fp32 BrushMotor_PID[3] = {BrushMotor_PID_KP , BrushMotor_PID_KI	,	BrushMotor_PID_KD	};
fp32 SelfTurn_PID[3] 	 = {SelfTurn_PID_KP , SelfTurn_PID_KI	,	SelfTurn_PID_KD	};
fp32 YawHold_PID[3]      = {YawHold_PID_KP , YawHold_PID_KI , YawHold_PID_KD };

/* 初始化单个有刷电机，将目标轮速 ExpectOutput 和驱动输出 Output 置零。 */
void BrushMotorInit (BrushMotor_t* BrushMotor , int ERF , int EL , float TR)
{
	BrushMotor->state = Disable; 

	/* ERF 为编码器读取频率（Hz），EL 为编码器线数，TR 为轮胎半径。 */
	BrushMotor->ERF= ERF;
	BrushMotor->EL=  EL;
	BrushMotor->TR=  TR;
	BrushMotor->ExpectOutput = 0.0f;
	BrushMotor->Output = 0;
}

/* 通过 Motors 返回电机组的地址。电机和 PID 对象均为静态变量，供后续控制周期使用。 */
void MotorInit (Motors_t** Motors)
{
 static Motors_t motors;
*Motors = &motors;
	/* 启动 PWM 定时器，将左右轮通道的初始比较值设为零。 */
	PWMStart(PWM_0_INST);
 PWM_OUTput(0,0);
 
 static BrushMotor_t motorLeftFront;
 static BrushMotor_t motorRightFront;
 static Pid_t pidselfturn;
 static Pid_t pidyaw;
 static Pid_t pidLeftFront;
 static Pid_t pidRightFront;
 
 motors.MotorLeft = &motorLeftFront;
 motors.MotorRight = &motorRightFront;
 
 motors.PidSelfTurn=&pidselfturn;
 motors.PidYaw=&pidyaw;
 
 motors.Left=&pidLeftFront;
 motors.Right=&pidRightFront;
 
 BrushMotorInit(motors.MotorLeft,EncoderReadingFrequency,EncoderLines,TireRadius);
 BrushMotorInit(motors.MotorRight,EncoderReadingFrequency,EncoderLines,TireRadius);
 
 /* 巡线和航向保持各用一个转向 PID。左右轮使用相同的速度环参数，各自保存积分和误差历史。 */
 PID_init(motors.PidSelfTurn,SelfTurn_PID_mode ,SelfTurn_PID ,SelfTurn_PID_Maxout,SelfTurn_PID_MaxIout);
 PID_init(motors.PidYaw,YawHold_PID_mode ,YawHold_PID ,YawHold_PID_Maxout,YawHold_PID_MaxIout);
 PID_init(motors.Left,BrushMotor_PID_mode,BrushMotor_PID,BrushMotor_PID_Maxout,BrushMotor_PID_MaxIout);
 PID_init(motors.Right,BrushMotor_PID_mode,BrushMotor_PID,BrushMotor_PID_Maxout,BrushMotor_PID_MaxIout);
 
 
}
 
/* 取驱动输出的绝对值作为 PWM 比较值，左轮写入通道 C0，右轮写入通道 C1。 */
void MotorsPWMSet(Motors_t* Motors)
{
	int pwm[2];
	
	pwm[0]=ABS(Motors->MotorLeft->Output);
	
	pwm[1]=ABS(Motors->MotorRight->Output);

   PWM_OUTput(pwm[0],pwm[1]);
}

/* 停车时清零两轮的目标速度和驱动输出，复位四个 PID，再将零输出写入硬件。 */
void MotorStop(Motors_t* Motors)
{
	if (Motors == NULL) {
		return;
	}
	Motors->MotorLeft->ExpectOutput=0.0f;
	Motors->MotorRight->ExpectOutput=0.0f;
	Motors->MotorLeft->Output=0;
	Motors->MotorRight->Output=0;
	PID_clear(Motors->Left);
	PID_clear(Motors->Right);
	PID_clear(Motors->PidSelfTurn);
	PID_clear(Motors->PidYaw);
	
	MotorDataUpdate(Motors);
	
}


// DRV8701E 单方向引脚接法的参考实现。
//void MotorDirectionSet(Motors_t* Motors)
//{
//	(Motors->MotorLeft->Output>0)?(DL_GPIO_setPins(Motor_PIN_7_PORT,Motor_PIN_7_PIN)):(DL_GPIO_clearPins(Motor_PIN_7_PORT,Motor_PIN_7_PIN));
//	(Motors->MotorRight->Output>0)?(DL_GPIO_setPins(Motor_PIN_9_PORT,Motor_PIN_9_PIN)):(DL_GPIO_clearPins(Motor_PIN_9_PORT,Motor_PIN_9_PIN));
//	
//}

/* 当前接法每个电机使用两个方向引脚，左轮为 PIN_7、PIN_8，右轮为 PIN_9、PIN_10。
 * 按上述顺序，输出为正时引脚为高、低，为负时为低、高，为零时两脚均为低电平。
 */
void MotorDirectionSet(Motors_t* Motors)
{
    (Motors->MotorLeft->Output > 0) ?
        DL_GPIO_setPins(Motor_PIN_7_PORT, Motor_PIN_7_PIN) :
        DL_GPIO_clearPins(Motor_PIN_7_PORT, Motor_PIN_7_PIN);

    (Motors->MotorLeft->Output < 0) ?
        DL_GPIO_setPins(Motor_PIN_8_PORT, Motor_PIN_8_PIN) :
        DL_GPIO_clearPins(Motor_PIN_8_PORT, Motor_PIN_8_PIN);

    (Motors->MotorRight->Output > 0) ?
        DL_GPIO_setPins(Motor_PIN_9_PORT, Motor_PIN_9_PIN) :
        DL_GPIO_clearPins(Motor_PIN_9_PORT, Motor_PIN_9_PIN);

    (Motors->MotorRight->Output < 0) ?
        DL_GPIO_setPins(Motor_PIN_10_PORT, Motor_PIN_10_PIN) :
        DL_GPIO_clearPins(Motor_PIN_10_PORT, Motor_PIN_10_PIN);
}

/* 灰度巡线时，将 TurnAngleSet 作为巡线偏差反馈，转向 PID 以偏差为零为目标。 */
void MotorPidCtrl (Motors_t* Motors,fp32 TurnAngleSet,fp32 AverageSpeedSet)
{
	float ExpectDifferentia=PID_Calc(Motors->PidSelfTurn,TurnAngleSet,0);
	/* 以 AverageSpeedSet 为两轮平均目标速度，左轮减去修正量，右轮加上修正量。 */
	Motors->MotorLeft->ExpectOutput=AverageSpeedSet-ExpectDifferentia;
	Motors->MotorRight->ExpectOutput=AverageSpeedSet+ExpectDifferentia;
	
	/* 各轮速度环用编码器实测线速度 V 跟踪本轮目标速度，得到带方向符号的驱动输出。 */
	Motors->MotorLeft->Output=PID_Calc(Motors->Left,Motors->EncoderLeft->V,Motors->MotorLeft->ExpectOutput);
	Motors->MotorRight->Output=PID_Calc(Motors->Right,Motors->EncoderRight->V,Motors->MotorRight->ExpectOutput);
	
}

/* 直行时，YawErrorRad 为当前航向减去基准航向（弧度）。航向 PID 以零偏差为目标，
 * 其输出修正左右轮目标速度，再由两轮各自的速度 PID 跟踪。
 */
void MotorYawPidCtrl (Motors_t* Motors,fp32 YawErrorRad,fp32 AverageSpeedSet)
{
	float ExpectDifferentia=PID_Calc(Motors->PidYaw,YawErrorRad,0);
	Motors->MotorLeft->ExpectOutput=AverageSpeedSet-ExpectDifferentia;
	Motors->MotorRight->ExpectOutput=AverageSpeedSet+ExpectDifferentia;
	
	Motors->MotorLeft->Output=PID_Calc(Motors->Left,Motors->EncoderLeft->V,Motors->MotorLeft->ExpectOutput);
	Motors->MotorRight->Output=PID_Calc(Motors->Right,Motors->EncoderRight->V,Motors->MotorRight->ExpectOutput);
}

/* 将本轮计算出的驱动输出写入硬件，先按符号设置方向引脚，再按幅值更新 PWM。 */
void MotorDataUpdate (Motors_t* Motors)
{
	MotorDirectionSet(Motors);
	MotorsPWMSet(Motors);
}
