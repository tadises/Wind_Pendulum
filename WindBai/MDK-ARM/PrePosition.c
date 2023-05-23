#include "PrePosition.h"
#include "math.h"
#include "mpu6050.h"
#include "stdio.h"
#include "Motor.h"

/*
@ brief: 通过输入mpu6050的pitch、roll来获得激光笔打在地面上的位置
*/
int getXY_Position(nowXY * nxy, float pitch, float roll)
{
	double alpha,beta,x0,y0,z0;
	alpha = (pitch + PITCH_OFFSET) * 2 * PI / 360;
	beta = (roll + ROLL_OFFSET) * 2 * PI / 360;
	nxy->x = -H * tan(beta);
	nxy->y = -H * tan(alpha);
	if(nxy->x > 0 && nxy->y > 0) return 1;
	if(nxy->x < 0 && nxy->y > 0) return 2;
	if(nxy->x < 0 && nxy->y < 0) return 3;
	if(nxy->x > 0 && nxy->y < 0) return 4;
	if(nxy->x == 0 && nxy->y == 0) return 0;
}

void getTargetAngle(targetAngle * target, float x, float y)
{
	target->targetPitch = 180 / PI * atan(y / H);
	target->targetRoll = 180 / PI * atan(x / H);
}
int getQuard(nowXY pos)
{
	if(pos.x > 0 && pos.y > 0) return 1;
	if(pos.x < 0 && pos.y > 0) return 2;
	if(pos.x < 0 && pos.y < 0) return 3;
	if(pos.x > 0 && pos.y < 0) return 4;
	if(pos.x == 0 && pos.y == 0) return 0;
}

void getAngleSpeed(float * omegaX, float * omegaY)
{
	short gyrox, gyroy, gyroz;
	MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz);
	*omegaX = gyrox/1.0 ;
	*omegaY = gyroy/1.0 ;
	printf("omegaX : %4f, omegaY : %4f\n",*omegaX, *omegaY);
}
int getTargetAngleSpeed(nowXY pos, float startRho, float startTheta, float * omegaX, float * omegaY)
{
	float nowRhoSquare, omega, theta, Px;
	int polar = 1, k, t = 0;
	nowRhoSquare = pos.x/100 * pos.x/100 + pos.y/100 * pos.y/100;
	startRho = startRho/100;
	Px = 2 * G / L/100 * (H/100 / sqrt(H/100 * H/100 + nowRhoSquare) - H/100 / sqrt(H/100 * H/100 + startRho * startRho));
//	if(Px > 0) polar = 1;
//	else polar = -1;
	if (nowRhoSquare > (startRho*startRho) && pos.y > 0) 
	{
		polar = -1;
	}
	if(nowRhoSquare > (startRho*startRho) && pos.y < 0)
	{
		polar = 1;
	}
//	printf("polar : %d\n",polar);
	theta = startTheta * PI / 180;
	omega = 100000*sqrt(fabs(Px));
	*omegaX = polar * omega * cos(theta);
	*omegaY = polar * omega * sin(theta);
	return polar;
}