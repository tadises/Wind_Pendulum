#ifndef __PREPOSITION_H
#define __PREPOSITION_H

#define H 76.5     // 含义：高度，单位：cm
#define L 50

#define PITCH_OFFSET (-1.00)  // 修正角度偏移
#define ROLL_OFFSET (0.650)

#define PI 3.14159

typedef struct nowXY
{
	float x;
	float y;
}nowXY;

typedef struct XY_Polar
{
	int X_polar;
	int Y_polar;
}XY_Polar;

typedef struct targetAngle
{
	float targetPitch;
	float targetRoll;
}targetAngle;

int getXY_Position(nowXY * nxy, float pitch, float roll);
void getTargetAngle(targetAngle * target, float x, float y);
int getQuard(nowXY pos);
void getAngleSpeed(float * omegaX, float * omegaY);
int getTargetAngleSpeed(nowXY pos, float startRho, float startTheta, float * omegaX, float * omegaY);

#endif