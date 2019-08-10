/**
* @par Copyright (C): 2010-2019, Shenzhen Yahboom Tech
* @file         arduino_4WD_McNamee-wheel_WiFi-Camera
* @author       wusicaijuan
* @version      V1.0
* @date         2019.07.04
* @brief       	wifi摄像头控制arduino4WD麦克纳姆轮小车
* @details
* @par History  见如下说明
*
*/
#include <Arduino.h>
#include <MsTimer2.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_NeoPixel.h> //库文件
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"
#include <math.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL

#define INTERRUPT_PIN 2 // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13		// (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;		// return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;	// expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;		// count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;		 // [w, x, y, z]         quaternion container
VectorInt16 aa;		 // [x, y, z]            accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float ypr[3];		 // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float pre_yaw = 0;

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
	mpuInterrupt = true;
}

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
#define SERVOMIN 150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 600 // this is the 'maximum' pulse length count (out of 4096)

#define PIN 9	 //定义RGB灯的引脚
#define MAX_LED 4 //小车一共有1个RGB灯
Adafruit_NeoPixel strip = Adafruit_NeoPixel(MAX_LED, PIN, NEO_RGB + NEO_KHZ800);

#define run_car '1'		   //按键前
#define back_car '2'	   //按键后
#define left_car '3'	   //按键左
#define right_car '4'	  //按键右
#define spin_left_car '5'  //按键左旋
#define spin_right_car '6' //按键右旋
#define stop_car '7'	   //按键停
#define servoL 'L'		   //舵机左转
#define servoR 'R'		   //舵机右转
#define servoS 'S'		   //舵机停止

#define ON 1  //使能LED
#define OFF 0 //禁止LED

/*小车运行状态枚举*/
const typedef enum {
	enRUN = 1,
	enBACK,
	enLEFT,
	enRIGHT,
	enSPINLEFT,
	enSPINRIGHT,
	enSTOP
} enCarState;

/*====================================================================================================
PID Function
The PID (比例、积分、微分) function is used in mainly
control applications. PIDCalc performs one iteration of the PID
algorithm.
While the PID function works, main is just a dummy program showing
a typical usage.
=====================================================================================================*/
typedef struct
{
	float SetPoint;   // 设定目标Desired value
	float Proportion; // 比例常数Proportional Const
	float Integral;   // 积分常数Integral Const
	float Derivative; // 微分常数Derivative Const
	float LastError;  // Error[-1]
	float PrevError;  // Error[-2]
	float SumError;   // Sums of Errors
} PID;

/*====================================================================================================/
PID计算部分
=====================================================================================================*/
PID omega_PID = {0, 100.1, 0.001, 0.001, 0, 0, 0};
PID alpha_PID = {0, 0.1, 0.001, 0.001, 0, 0, 0};

struct car_omega
{
	/* data */
	float Kp = 0, Ki = 0, Kd = 0;
	float target_angle = 0;
	float error = 0, I = 0, D = 0, PID_value = 0;
	float previous_error = 0, previous_I = 0;
};

struct car_alpha
{
	/* data */
	float Kp = 0, Ki = 0, Kd = 0;
	float target_angle = 0;
	float error = 0, I = 0, D = 0, PID_value = 0;
	float previous_error = 0, previous_I = 0;
};

const int key = 8; //按键key

/*小车初始速度控制*/
const char wheel[4][2] = {{10, 11}, {13, 12}, {15, 14}, {8, 9}};
static int CarSpeedControl = 150;
float omega_Work = 0;

/*串口数据设置*/
int IncomingByte = 0;			 //接收到的 data byte
String InputString = "";		 //用来储存接收到的内容
boolean NewLineReceived = false; //前一次数据结束标志
boolean StartBit = false;		 //协议开始标志
/*电机状态*/
static int g_CarState = enSTOP; //1前2后3左4右5左旋6右旋7停止

/**
* Function       setup
* @author        wusicaijuan
* @date          2019.07.04
* @brief         初始化配置
* @param[in]     void
* @retval        void
* @par History   无
*/
void setup()
{
	// configure LED for output
	pinMode(LED_PIN, OUTPUT);

	//串口波特率设置
	// Serial.begin(9600);

	strip.begin();
	strip.show();
	PCB_RGB_OFF();

	pwm.begin();
	pwm.setPWMFreq(50); // Analog servos run at ~60 Hz updates
	Clear_All_PWM();


	Wire.begin();
	Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

	// Serial.begin(115200);
	Serial.begin(9600);
	// initialize device
	mpu.initialize();
	pinMode(INTERRUPT_PIN, INPUT);

	devStatus = mpu.dmpInitialize();

	// supply your own gyro offsets here, scaled for min sensitivity
	mpu.setXGyroOffset(51);
	mpu.setYGyroOffset(8);
	mpu.setZGyroOffset(21);
	mpu.setXAccelOffset(1150);
	mpu.setYAccelOffset(-50);
	mpu.setZAccelOffset(1060);
	// make sure it worked (returns 0 if so)
	if (devStatus == 0)
	{
		// Calibration Time: generate offsets and calibrate our MPU6050
		mpu.CalibrateAccel(6);
		mpu.CalibrateGyro(6);
		// Serial.println();
		mpu.PrintActiveOffsets();
		// turn on the DMP, now that it's ready
		mpu.setDMPEnabled(true);

		attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
		mpuIntStatus = mpu.getIntStatus();

		// set our DMP Ready flag so the main loop() function knows it's okay to use it
		dmpReady = true;

		// get expected DMP packet size for later comparison
		packetSize = mpu.dmpGetFIFOPacketSize();
	}

	// 中断设置函数，每 5ms 进入一次中断
	MsTimer2::set(5, flash);
	//开始计时
	MsTimer2::start();

	//舵机归位
	Servo180(75);

	pinMode(key, INPUT); //定义按键输入脚

	breathing_light(20, 1);
}

float PIDCalc(float NextPoint)
{
	float dError, Error;
	Error = omega_PID.SetPoint - NextPoint;				// 偏差
	omega_PID.SumError += Error;						// 积分
	dError = omega_PID.LastError - omega_PID.PrevError; // 当前微分
	omega_PID.PrevError = omega_PID.LastError;
	omega_PID.LastError = Error;

	// if (omega_PID.SumError > 900)
	// 	omega_PID.SumError = 900;
	// else if (omega_PID.SumError < -900)
	// 	omega_PID.SumError = -900;

	return (omega_PID.Proportion * Error			  // 比例项
			+ omega_PID.Integral * omega_PID.SumError // 积分项
			+ omega_PID.Derivative * dError			  // 微分项
	);
}

//中断处理函数，改变灯的状态
void flash()
{
    mpu6050_getdata();
	omega_Work = PIDCalc(ypr[0]);
}

/**
* Function       brake
* @author        wusicaijuan
* @date          2019.06.25
* @brief         小车刹车
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   无
*/
void brake()
{
	pwm.setPWM(8, 0, 0);
	pwm.setPWM(9, 0, 0);
	pwm.setPWM(11, 0, 0);
	pwm.setPWM(10, 0, 0);

	pwm.setPWM(12, 0, 0);
	pwm.setPWM(13, 0, 0);
	pwm.setPWM(14, 0, 0);
	pwm.setPWM(15, 0, 0);
}

/*
* Function      Servo180(num, degree)
* @author       wusicaijuan
* @date         2019.06.25
* @bried        180°舵机转动角度
                    180 Degree Steering Engine Rotation Angle
* @param[in1]   index
                    1: s1
                    2: s2
                    3: s3
                    4: s4
* @param[in2]   degree (0 <= degree <= 180)
* @retval       void
*/
void Servo180(int degree)
{
	long us = (degree * 1800 / 180 + 600); // 0.6 ~ 2.4
	long pwmvalue = us * 4096 / 20000;	 // 50hz: 20,000 us
	pwm.setPWM(7, 0, pwmvalue);
}

/**
* Function       PCB_RGB(R,G,B)
* @author        wusicaijuan
* @date          2019.06.26
* @brief         设置板载RGB灯
* @param[in1]	 R
* @param[in2]    G
* @param[in3]    B
* @param[out]    void
* @retval        void
* @par History   无
*/
void PCB_RGB(int R, int G, int B)
{
	R = map(R, 0, 255, 0, 10);
	G = map(G, 0, 255, 0, 10);
	B = map(B, 0, 255, 0, 10);
	uint32_t color = strip.Color(G, R, B);
	for (uint8_t i = 0; i < 4; i++)
	{
		strip.setPixelColor(i, color);
	}
	strip.show();
}

/**
* Function       PCB_RGB_OFF()
* @author        wusicaijuan
* @date          2019.06.26
* @brief         关闭板载RGB灯
* @param[in1]	 void
* @param[out]    void
* @retval        void
* @par History   无
*/
void PCB_RGB_OFF()
{
	uint32_t color = strip.Color(0, 0, 0);
	for (uint8_t i = 0; i < 4; i++)
	{
		strip.setPixelColor(i, color);
	}
	strip.show();
}

/**
* Function       serial_data_parse
* @author        wusicaijuan
* @date          2019.07.04
* @brief         串口数据解析并指定相应的动作
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   无
*/
void serial_data_parse()
{

	//解析上位机发来的通用协议指令,并执行相应的动作
	//$4WD,UD180#
	if (InputString.indexOf("4WD") > 0)
	{
		//解析上位机发来的舵机云台的控制指令并执行舵机旋转
		//$4WD,UD180# 舵机转动到180度
		if (InputString.indexOf("UD") > 0)
		{
			int i = InputString.indexOf("UD"); //寻找以PTZ开头,#结束中间的字符
			int ii = InputString.indexOf("#", i);
			if (ii > i)
			{
				String m_skp = InputString.substring(i + 2, ii);
				int m_kp = m_skp.toInt(); //将找到的字符串变成整型
				m_kp = map(m_kp, 0, 180, 60, 150);
				Servo180(180 - m_kp); //转动到指定角度m_kp
			}
			InputString = ""; //清空串口数据
			NewLineReceived = false;
			return; //直接跳出本次loop()
		}
		InputString = ""; //清空串口数据
		NewLineReceived = false;
		return;
	}
	//解析上位机发来的通用协议指令,并执行相应的动作
	//如:$1,0,0,0#    小车前进
	//检测长度以防误判
	if ((InputString.indexOf("4WD") == -1) && (InputString.length() == 9))
	{
		//小车加减速判断
		if (InputString[3] == '1') //加速，每次加50
		{
			CarSpeedControl += 50;
			if (CarSpeedControl > 150)
			{
				CarSpeedControl = 150;
			}
			InputString = ""; //清空串口数据
			NewLineReceived = false;
			return;
		}
		if (InputString[3] == '2') //减速，每次减50
		{
			CarSpeedControl -= 50;
			if (CarSpeedControl < 50)
			{
				CarSpeedControl = 50;
			}
			InputString = ""; //清空串口数据
			NewLineReceived = false;
			return;
		}

		switch (InputString[1])
		{
		case run_car:
			g_CarState = enRUN;
			break;
		case back_car:
			g_CarState = enBACK;
			break;
		case left_car:
			g_CarState = enLEFT;
			break;
		case right_car:
			g_CarState = enRIGHT;
			break;
		case spin_left_car:
			g_CarState = enSPINLEFT;
			break;
		case spin_right_car:
			g_CarState = enSPINRIGHT;
			break;
		case stop_car:
			g_CarState = enSTOP;
			break;
		default:
			g_CarState = enSTOP;
            omega_PID.SetPoint = ypr[0];
			break;
		}

		InputString = ""; //清空串口数据
		NewLineReceived = false;
	}
	InputString = ""; //清空串口数据
	NewLineReceived = false;
	return;
}
/**
* Function       loop
* @author        wusicaijuan
* @date          2019.07.08
* @brief         对串口发送过来的数据解析，并执行相应的指令
* @param[in]     void
* @retval        void
* @par History   无
*/

void loop()
{
	// mpu6050_getdata();
	// float an = PIDCalc(omega_PID, ypr[0]);
	serialEvent();
	if (NewLineReceived)
	{
		// 调试查看串口数据
		// Serial.println(InputString);
		serial_data_parse(); //调用串口解析函数
	}
	// InputString = ""; //清空串口数据
	//根据小车状态做相应的动作
	switch (g_CarState)
	{
	case enSTOP:
		brake();
		break;
	case enRUN:
		mecanum_run(0, CarSpeedControl, omega_Work, CarSpeedControl);
		break;
	case enLEFT:
		mecanum_run(-M_PI / 2, CarSpeedControl, omega_Work, CarSpeedControl);
		break;
	case enRIGHT:
		mecanum_run(M_PI / 2, CarSpeedControl, omega_Work, CarSpeedControl);
		break;
	case enBACK:
		mecanum_run(M_PI, CarSpeedControl, omega_Work, CarSpeedControl);
		break;
	case enSPINLEFT:
		mecanum_run(0, 0, M_PI / 2, CarSpeedControl);
		break;
	case enSPINRIGHT:
		mecanum_run(0, 0, -M_PI / 2, CarSpeedControl);
		break;
	default:
		brake();
		break;
	}
}

/**
* Function       serialEvent
* @author        liusen
* @date          2017.07.25
* @brief         串口解包
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   无
*/
void serialEvent()
{
	while (Serial.available())
	{
		//一个字节一个字节地读,下一句是读到的放入字符串数组中组成一个完成的数据包
		IncomingByte = Serial.read();
		if (IncomingByte == '$')
		{
			StartBit = true;
		}
		if (StartBit == true)
		{
			InputString += (char)IncomingByte;
		}
		if (IncomingByte == '#')
		{
			NewLineReceived = true;
			StartBit = false;
		}
	}
}

/*
* Function       Clear_All_PWM
* @author        wusicaijuan
* @date          2019.07.04
* @brief         关闭PCA9685所有PWM
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   无
*/
void Clear_All_PWM()
{
	for (int i = 0; i < 16; i++)
	{
		pwm.setPWM(i, 0, 0);
	}
}

/*
* Function       breathing_light
* @author        wusicaijuan
* @date          2019.07.04
* @brief         呼吸灯
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   无
*/
void breathing_light(int time, int increament)
{
	uint32_t color = strip.Color(0, 0, 0);
	for (int a = 0; a < 256; a += increament)
	{
		color = strip.Color(a, 0, 0);
		for (uint8_t i = 0; i < 4; i++)
		{
			strip.setPixelColor(i, color);
		}
		strip.show();
		delay(time);
	}
}

void mpu6050_getdata()
{
	// if programming failed, don't try to do anything
	if (!dmpReady)
		return;

	// wait for MPU interrupt or extra packet(s) available
	while (!mpuInterrupt && fifoCount < packetSize)
	{
		if (mpuInterrupt && fifoCount < packetSize)
		{
			// try to get out of the infinite loop
			fifoCount = mpu.getFIFOCount();
		}
	}

	// reset interrupt flag and get INT_STATUS byte
	mpuInterrupt = false;
	mpuIntStatus = mpu.getIntStatus();

	// get current FIFO count
	fifoCount = mpu.getFIFOCount();

	// check for overflow (this should never happen unless our code is too inefficient)
	if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024)
	{
		// reset so we can continue cleanly
		mpu.resetFIFO();
		fifoCount = mpu.getFIFOCount();
		// Serial.println(F("FIFO overflow!"));

		// otherwise, check for DMP data ready interrupt (this should happen frequently)
	}
	else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT))
	{
		// wait for correct available data length, should be a VERY short wait
		while (fifoCount < packetSize)
			fifoCount = mpu.getFIFOCount();

		// read a packet from FIFO
		mpu.getFIFOBytes(fifoBuffer, packetSize);

		// track FIFO count here in case there is > 1 packet available
		// (this lets us immediately read more without waiting for an interrupt)
		fifoCount -= packetSize;

		// display Euler angles in degrees
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        if((ypr[0]>pre_yaw+1/60)&&(ypr[0]<pre_yaw-1/60)){
            return;
        }else
        {
            pre_yaw = ypr[0];
        }
        
		// Serial.print("ypr\t");
		// Serial.print(cal_omega(ypr[0]));
		// Serial.print("\t");
		mpu.dmpGetAccel(&aa, fifoBuffer);
		// Serial.print("\tRaw Accl XYZ\t");
		// Serial.print(aa.x);
		// Serial.print("\t");
		// Serial.print(aa.y);
		// Serial.print("\t");
		// Serial.print(cal_angle(-aa.x, aa.y));
		// Serial.println();

		// blink LED to indicate activity
		blinkState = !blinkState;
		digitalWrite(LED_PIN, blinkState);
	}
}

//计算移动方向角(-aa.x, aa.y)
float cal_angle(float y, float x)
{
	return atan2(int(y / 200) * 200, int(x / 200) * 200);
}

//计算偏航角ypr[0]->弧度
float cal_omega(float a)
{
	return (a * 180 / M_PI); //角度度数
}

/**
* Function       mecanum_run
* @author        wusicaijuan
* @date          2019.06.25
* @brief         麦克纳姆移动
* @param[in]     Speed
* @param[out]    void
* @retval        void
* @par History   无
*/
void mecanum_run(float car_alpha, int speed_L, int car_omega, int speed_A)
{
	speed_L = speed_L * 16; //map 255 to 4096
	speed_A = speed_A * 16; //map 255 to 4096
	float speed_x = speed_L * sin(car_alpha);
	float speed_y = speed_L * cos(car_alpha);
	float speed_omega = speed_A * sin(car_omega);
	float wheel_speed[4];
	wheel_speed[0] = speed_y - speed_x + speed_omega;
	wheel_speed[1] = speed_y + speed_x - speed_omega;
	wheel_speed[2] = speed_y - speed_x - speed_omega;
	wheel_speed[3] = speed_y + speed_x + speed_omega;
	for (int i = 0; i < 4; i++)
	{
		if (wheel_speed[i] >= 0)
		{
			pwm.setPWM(wheel[i][0], 0, wheel_speed[i]);
			pwm.setPWM(wheel[i][1], 0, 0);
		}
		else
		{
			pwm.setPWM(wheel[i][0], 0, 0);
			pwm.setPWM(wheel[i][1], 0, -wheel_speed[i]);
		}
	}
}
