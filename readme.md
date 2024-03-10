# Wind_Pendulum



windbai 倒立风力摆，主要使用空心杯电机作为动力，mpu6050作为主要传感器，stm32f401ccu6作为主控

![IO ports](https://github.com/tadises/Wind_Pendulum/blob/main/IO%20ports.png)

共二路串口蓝牙遥控和串口监视，一路I2C作为6050读取，两路GPIO输出作为激光控制和备用，两路GPIO输入作为按键，四路PWM输出控制空心杯电机。

电路版预览

![model_pcb](https://github.com/tadises/Wind_Pendulum/blob/main/model2.png)

模型预览

![model](https://github.com/tadises/Wind_Pendulum/blob/main/model1.png)
