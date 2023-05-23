#include "Motor.h"
#include "main.h"
#include "PrePosition.h"
#include "PID.h"
#include "math.h"
#include "stdio.h"

float T[] = {1554.685,0};
float A = 800, Mx = 25, vectorX = 0, vectorY = 0, tX = 0, tY = 0;
int pidPWM_X = 0, pidPWM_Y = 0;
//调这两个数组，第一个15cm,第二个20cm，第三个25cm，第四个30cm，第五个35cm（半径）
float len[6] = {172.3698*5,172.3698*6.4 , 172.3698*7.75, 172.3698*9 , 172.3698*10.1, 0};
float rad[6] = {172.3698*5,172.3698*6.4 , 172.3698*7.75, 172.3698*9.4 , 172.3698*10.5, 0};

float lastLength, curLength;

void setPWM_X(int PWM_LEFT, int PWM_RIGHT)
{
	if (PWM_LEFT > 800) PWM_LEFT = 800;
	if (PWM_RIGHT > 800) PWM_RIGHT = 800;
	TIM2->CCR2 = PWM_LEFT;
	TIM2->CCR4 = PWM_RIGHT;
//	printf("PWM_L: %d  PWM_R:%d\n",PWM_LEFT,PWM_RIGHT);
}
void setPWM_Y(int PWM_UP, int PWM_DOWN)
{
	if (PWM_DOWN > 800) PWM_DOWN = 800;
	if (PWM_UP > 800) PWM_UP = 800;
	TIM2->CCR1 = PWM_UP;
	TIM2->CCR3 = PWM_DOWN;
	
}
void setVector_X(int PWM)
{
	if(PWM > 0)
	{
		setPWM_X(PWM, 0);
	}
	else
	{
		setPWM_X(0, -PWM);
	}
}
void setVector_Y(int PWM)
{
	if(PWM > 0)
	{
		setPWM_Y(PWM, 0);
	}
	else
	{
		setPWM_Y(0, -PWM);
	}
}

void gotoPoint(PID_Typedef PID_Variables, float tX, float tY, nowXY pos)
{
	XY_Polar pol;
	float Omegax,Omegay;
	float pwm_omega_x,pwm_omega_y;
	int gravPWM_X, gravPWM_Y;
	getAngleSpeed(&Omegax,&Omegay);
	pwm_omega_x = PID_Control_Dir(&speed_pid,0,Omegax);
	pwm_omega_y = PID_Control_Dir(&speed_pid,0,Omegay);
//	printf("pwmOmegaX:%4f pwmOmegaY:%4f\n",pwm_omega_x, pwm_omega_y);
	int PWM_X = 0, PWM_Y = 0, pidPWM_X, pidPWM_Y;
	setGraviPWM(tX, tY, &gravPWM_X, &gravPWM_Y);
	printf("grav_x:%d,grav_y:%d",gravPWM_X, gravPWM_Y);
	pol=XY_PWMControll_Point(PID_Variables, &pidPWM_X, &pidPWM_Y, pos, tX, tY);
//	pidPWM_X *=pol.X_polar;
//	pidPWM_Y *=pol.Y_polar;
	PWM_X = (((pos.x-tX)*(pos.x-tX)/120)*pidPWM_X - 4/fabs(pos.x-tX + 1 * pol.X_polar)*pwm_omega_x) + gravPWM_X;
	PWM_Y = (((pos.y-tY)*(pos.y-tY)/120)*pidPWM_Y - 4/fabs(pos.y-tY + 1 * pol.Y_polar)*pwm_omega_y) + gravPWM_Y;
	printf("pidX: %d,pidY %d\n",PWM_X,PWM_Y);
	setVector_X((int)PWM_X);
	setVector_Y((int)PWM_Y);
}
void setGraviPWM(float x, float y, int* PWM_X, int* PWM_Y)
{
	float alpha, beta, g_x, g_y, pitch, roll;
	pitch = 180 / PI * atan(y / H);
	roll = 180 / PI * atan(x / H);
	alpha = PI / 2 - pitch * PI / 180;
	beta = PI / 2 - roll * PI / 180;
	g_y = G * cos(alpha);
	g_x = G* cos(beta);
	*PWM_X = KM * g_x;
	*PWM_Y = KM * g_y;
}
void graphControl(int mode_, nowXY pos, int32_t timeCNT, int indexLen, float angle)
{
	if(mode_ == 0) // 水平摆直线
	{
		float omegaX, omegaY, tOmegaX, tOmegaY, Vmax = 172.3698, PWM_X, PWM_Y;
			getAngleSpeed(&omegaX, &omegaY);
			tOmegaY = len[indexLen] * cos(2 * PI / T[0] * timeCNT);//越大线越长
			tOmegaX = 0;
			omegaX = - omegaX;
			omegaY = - omegaY;
			PWM_X = PID_Control_Dir(&speed_pid, tOmegaX, omegaX);
			PWM_Y = PID_Control_Dir(&speed_pid, tOmegaY, omegaY);
//			if(fabs(omegaX)>1000) PWM_Y = 0;	
			setVector_X(PWM_X);
			setVector_Y(PWM_Y);
		
//		timeCNT++;
		printf("timeCNT: %d\n", timeCNT);
	}
	if(mode_ == 1)	//	回零
	{		
		//int quard = 0;
			float omegaX,omegaY, tOmegaX, tOmegaY, Vmax = 172.3698, PWM_X, PWM_Y;
			getAngleSpeed(&omegaX, &omegaY);
			tOmegaY = 0;
			tOmegaX = 0;
			omegaX = - omegaX;
			omegaY = - omegaY;
			PWM_X = PID_Control_Dir(&speed_pid, tOmegaX, omegaX);
			PWM_Y = PID_Control_Dir(&speed_pid, tOmegaY, omegaY);
//			if(fabs(omegaX)>1000) PWM_Y = 0;	
			setVector_X(PWM_X);
			setVector_Y(PWM_Y);
		
//		timeCNT++;
		printf("time:%d\n",timeCNT);
			//printf("PWM_Y: %f\n", PWM_Y);
	}
	if(mode_ == 2) // 角度
	{
		float tOmegaX = 0, tOmegaY = 0, omegaX = 0, omegaY = 0, PWM_X = 0, PWM_Y = 0, rho = 0, tRho = 10, Vmax = 172.3698;
		if (timeCNT % 1 == 0)
		{
		getAngleSpeed(&omegaX, &omegaY);
		omegaX = - omegaX;
		omegaY = - omegaY;
		rho = sqrt(pos.x * pos.x + pos.y * pos.y);
		angle = angle * PI / 180;
//		getTargetAngleSpeed(pos, tRho, 90, &tOmegaX, &tOmegaY);
		tOmegaY = len[indexLen] * sin(angle) * cos(2 * PI / T[0] * timeCNT);
		tOmegaX = len[indexLen] * cos(angle) * cos(2 * PI / T[0] * timeCNT);
		printf("tOmegaY:%4f\n", tOmegaY);
		PWM_X = PID_Control_Dir(&speed_pid, tOmegaX, omegaX);
		PWM_Y = PID_Control_Dir(&speed_pid, tOmegaY, omegaY);
		setVector_X(PWM_X);
		setVector_Y(PWM_Y);
		}
	}
	if(mode_ == 3) // 画圆
	{
		float tOmegaX = 0, tOmegaY = 0, omegaX = 0, omegaY = 0, PWM_X = 0, PWM_Y = 0, Vmax = 172.3698;

			getAngleSpeed(&omegaX, &omegaY);
			omegaX = -omegaX;
			omegaY = -omegaY;
			tOmegaY = rad[indexLen] * cos(2 * PI / T[0] * timeCNT)+25;
			tOmegaX = rad[indexLen] * sin(2 * PI / T[0] * timeCNT)+50;
			PWM_X = PID_Control_Dir(&speed_pid, tOmegaX, omegaX);
			PWM_Y = PID_Control_Dir(&speed_pid, tOmegaY, omegaY);
			printf("tOmegaX:%4f,tOmegaY:%4f\n",tOmegaX, tOmegaY);
			setVector_X(PWM_X);
			setVector_Y(PWM_Y);
		
	}
	
}	




