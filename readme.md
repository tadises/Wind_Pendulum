# Wind_Pendulum



windbai 倒立风力摆，主要使用空心杯电机作为动力，mpu6050作为主要传感器，stm32f401ccu6作为主控

![image-20240310154844320](C:\Users\23885\AppData\Roaming\Typora\typora-user-images\image-20240310154844320.png)

共二路串口蓝牙遥控和串口监视，一路I2C作为6050读取，两路GPIO输出作为激光控制和备用，两路GPIO输入作为按键，四路PWM输出控制空心杯电机。

电路版预览

![image-20240310155641005](C:\Users\23885\AppData\Roaming\Typora\typora-user-images\image-20240310155641005.png)

模型预览

![image-20240310155604128](C:\Users\23885\AppData\Roaming\Typora\typora-user-images\image-20240310155604128.png)