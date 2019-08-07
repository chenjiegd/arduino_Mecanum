#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu; //实例化一个 MPU6050 对象，对象名称为 mpu
int16_t ax, ay, az, gx, gy, gz;

//********************angle data*********************//
float Gyro_y; //Y轴陀螺仪数据暂存
float Gyro_x;
float Gyro_z;
float angleAx;
float angle6;
float K1 = 0.05; // 对加速度计取值的权重
float Angle;	 //一阶互补滤波计算出的小车最终倾斜角度
float accelz = 0;

//********************angle data*********************//

//***************Kalman_Filter*********************//
float P[2][2] = {{1, 0},
				 {0, 1}};
float Pdot[4] = {0, 0, 0, 0};
float Q_angle = 0.001, Q_gyro = 0.005; //角度数据置信度,角速度数据置信度
float R_angle = 0.5, C_0 = 1;
float q_bias, angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
float timeChange = 5;		   //滤波法采样时间间隔毫秒
float dt = timeChange * 0.001; //注意：dt的取值为滤波器采样时间
//***************Kalman_Filter*********************//

void Angletest()
{
	//平衡参数
	Angle = atan2(ay, az) * 57.3; //角度计算公式
	Gyro_x = (gx - 128.1) / 131;  //角度转换
	Kalman_Filter(Angle, Gyro_x); //卡曼滤波
	//旋转角度Z轴参数
	if (gz > 32768)
		gz -= 65536;	//强制转换2g  1g
	Gyro_z = -gz / 131; //Z轴参数转换
	accelz = az / 16.4;

	angleAx = atan2(ax, az) * 180 / PI; //计算与x轴夹角
	Gyro_y = -gy / 131.00;				//计算角速度
	//一阶互补滤波
	angle6 = K1 * angleAx + (1 - K1) * (angle6 + Gyro_y * dt);
}

////////////////////////kalman/////////////////////////
float angle, angle_dot; //平衡角度值
void Kalman_Filter(double angle_m, double gyro_m)
{
	angle += (gyro_m - q_bias) * dt;
	angle_err = angle_m - angle;
	Pdot[0] = Q_angle - P[0][1] - P[1][0];
	Pdot[1] = -P[1][1];
	Pdot[2] = -P[1][1];
	Pdot[3] = Q_gyro;
	P[0][0] += Pdot[0] * dt;
	P[0][1] += Pdot[1] * dt;
	P[1][0] += Pdot[2] * dt;
	P[1][1] += Pdot[3] * dt;
	PCt_0 = C_0 * P[0][0];
	PCt_1 = C_0 * P[1][0];
	E = R_angle + C_0 * PCt_0;
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	t_0 = PCt_0;
	t_1 = C_0 * P[0][1];
	P[0][0] -= K_0 * t_0;
	P[0][1] -= K_0 * t_1;
	P[1][0] -= K_1 * t_0;
	P[1][1] -= K_1 * t_1;
	angle += K_0 * angle_err; //角度
	q_bias += K_1 * angle_err;
	angle_dot = gyro_m - q_bias; //角速度
}

void setup()
{
	Wire.begin();		//加入 I2C 总线序列
	Serial.begin(9600); //开启串口，设置波特率
	delay(1000);
	mpu.initialize(); //初始化MPU6050
}

void loop()
{
	mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); //IIC获取MPU6050六轴数据 ax ay az gx gy gz

	Angletest(); //获取angle 角度和卡曼滤波

	// Serial.print(ax);Serial.print(",");
	// Serial.print(ay);Serial.print(",");
	// Serial.print(az);Serial.print("---");
	Serial.print(angle);
	Serial.print(",");
	Serial.print(angle_dot);
	Serial.print(",");
	Serial.println(angle6);

	delay(50);
}
