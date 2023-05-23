#include "main.h"
#include "stdio.h"
#include "pid.h"
#include "PrePosition.h"
#include "math.h"

PID_Typedef thisPID,speed_pid;

void PID_Init(PID_Typedef * PID_Variables, float Kp, float Ki, float Kd)
{
	PID_Variables->Kp=Kp;
	PID_Variables->Ki=Ki;
	PID_Variables->Kd=Kd;
	PID_Variables->Error=0;
	PID_Variables->Error_Last=0;
	PID_Variables->Error_Prev=0;
	PID_Variables->Error_Sum=0;
	PID_Variables->Value_Actual=0;
	PID_Variables->Value_Target=0;
}
 
float PID_Control_Dir(PID_Typedef* PID_Variables, float Target, float Value_Feedback)
{
//	PID_Variables->Error=(PID_Variables->Value_Target - Value_Feedback);
//	PID_Variables->Value_Actual+=(PID_Variables->Kp * (PID_Variables->Error-PID_Variables->Error_Last))
//													+(PID_Variables->Ki * PID_Variables->Error)
//													+(PID_Variables->Kd * (PID_Variables->Error + PID_Variables->Error_Prev - 2*PID_Variables->Error_Last));
//	PID_Variables->Error_Last = PID_Variables->Error;
//	PID_Variables->Error_Prev = PID_Variables->Error_Last;
	float outPut;
	PID_Variables->Error = (Target - Value_Feedback);
//	printf("ERROR %f\n",PID_Variables->Error);
	PID_Variables->Error_Sum = PID_Variables->Error_Sum + PID_Variables->Error;
	outPut = (PID_Variables->Kp) * (PID_Variables->Error)
				 + (PID_Variables->Ki) * (PID_Variables->Error_Sum)
				 + (PID_Variables->Kd) * (PID_Variables->Error - PID_Variables->Error_Last);
	PID_Variables->Error_Last = PID_Variables->Error;
	return outPut;
}

XY_Polar XY_PWMControll_Line(PID_Typedef PID_Variables, int* PWM_X, int* PWM_Y, nowXY Pos_XY, float tLine_theta)
{
	float theta = tLine_theta * PI / 180;
	float deltaX, deltaY;
	XY_Polar pol;
	deltaX = Pos_XY.x - Pos_XY.y * cos(theta)/sin(theta);
	deltaY = Pos_XY.y - Pos_XY.x * tan(theta);
	*PWM_X = PID_Control_Dir(&PID_Variables, 0, deltaX);
	*PWM_Y = PID_Control_Dir(&PID_Variables, 0, deltaY);
	
	if(deltaX > 0) pol.X_polar = 1;
	else pol.X_polar = -1;
	if(deltaY > 0) pol.Y_polar = 1;
	else pol.Y_polar = -1;
	return pol;
}

XY_Polar XY_PWMControll_Point(PID_Typedef PID_Variables, int* PWM_X, int* PWM_Y, nowXY Pos_XY, float tX, float tY)
{
	float deltaX, deltaY;
	XY_Polar pol;
	deltaX = Pos_XY.x - tX;
	deltaY = Pos_XY.y - tY;
	*PWM_X = PID_Control_Dir(&PID_Variables, tX, Pos_XY.x);
	*PWM_Y = PID_Control_Dir(&PID_Variables, tY, Pos_XY.y);
//	printf("tx:%f posx:%f\n",tX,Pos_XY.x);
	if(deltaX > 0) pol.X_polar = 1;
	else pol.X_polar = -1;
	if(deltaY > 0) pol.Y_polar = 1;
	else pol.Y_polar = -1;
	return pol;
}


XY_Polar XY_PWMControll_Circle(PID_Typedef PID_Variables, int* PWM_X, int* PWM_Y, nowXY Pos_XY, float tR)
{
	float cosTheta, sinTheta, l, deltaX, deltaY;
	XY_Polar pol;
	l = sqrt(Pos_XY.x * Pos_XY.x + Pos_XY.y * Pos_XY.y);
	cosTheta = Pos_XY.x / l;
	sinTheta = Pos_XY.y / l;
	deltaX = l * cosTheta;
	deltaY = l * sinTheta;
	*PWM_X = PID_Control_Dir(&PID_Variables, 0, deltaX);
	*PWM_Y = PID_Control_Dir(&PID_Variables, 0, deltaY);
	
	if(deltaX > 0) pol.X_polar = 1;
	else pol.X_polar = -1;
	if(deltaY > 0) pol.Y_polar = 1;
	else pol.Y_polar = -1;
	return pol;
}

