#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "PrePosition.h"
#include "PID.h"
#include "main.h"

#define Time 4
#define G 9.8
#define KM 400


void setPWM_X(int PWM_Left, int PWM_Right);
void setPWM_Y(int PWM_Left, int PWM_Right);
void gotoPoint(PID_Typedef PID_Variables, float tX, float tY, nowXY pos);
void setGraviPWM(float x, float y, int* PWM_X, int* PWM_Y);
void setVector_X(int PWM);
void setVector_Y(int PWM);
void graphControl(int mode_, nowXY pos, int32_t timeCNT, int length, float angle);

#endif