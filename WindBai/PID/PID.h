#include "PrePosition.h"

#ifndef PID_H
#define PID_H

typedef struct
{
	float Kp;
	float Ki;
	float Kd;
	float Value_Target;
	float Value_Actual;
	float Error_Sum;
	float Error;
	float Error_Last;
	float Error_Prev;
}PID_Typedef;

extern PID_Typedef thisPID,speed_pid;

void PID_Init(PID_Typedef * PID_Variables, float Kp, float Ki, float Kd);
float PID_Control_Dir(PID_Typedef* PID_Variables, float Target, float Value_Feedback);
XY_Polar XY_PWMControll_Line(PID_Typedef PID_Variables, int* PWM_X, int* PWM_Y, nowXY Pos_XY, float tLine_theta);
XY_Polar XY_PWMControll_Point(PID_Typedef PID_Variables, int* PWM_X, int* PWM_Y, nowXY Pos_XY, float tX, float tY);
XY_Polar XY_PWMControll_Circle(PID_Typedef PID_Variables, int* PWM_X, int* PWM_Y, nowXY Pos_XY, float tR);

#endif