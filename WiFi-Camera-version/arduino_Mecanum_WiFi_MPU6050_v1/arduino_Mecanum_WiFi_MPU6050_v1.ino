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
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_NeoPixel.h> //库文件
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"

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

const int key = 8; //按键key

/*小车初始速度控制*/
const char wheel[4][2] = {{10, 11}, {13, 12}, {15, 14}, {8, 9}};
static int CarSpeedControl = 150;

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
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	Wire.begin();
	Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#endif
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
	//舵机归位
	Servo180(75);

	pinMode(key, INPUT); //定义按键输入脚

	breathing_light(20, 1);
}




/**
* Function       run
* @author        wusicaijuan
* @date          2019.06.25
* @brief         小车前进
* @param[in]     Speed
* @param[out]    void
* @retval        void
* @par History   无
*/
void run(int Speed)
{
	Speed = map(Speed, 0, 255, 0, 4095);
	pwm.setPWM(10, 0, Speed); //右前
	pwm.setPWM(11, 0, 0);
	pwm.setPWM(8, 0, Speed); //右后
	pwm.setPWM(9, 0, 0);

	pwm.setPWM(13, 0, Speed); //左前
	pwm.setPWM(12, 0, 0);
	pwm.setPWM(15, 0, Speed); //左后
	pwm.setPWM(14, 0, 0);
}

/**
* Function       back
* @author        wusicaijuan
* @date          2019.06.25
* @brief         小车后退
* @param[in]     Speed
* @param[out]    void
* @retval        void
* @par History   无
*/
void back(int Speed)
{
	Speed = map(Speed, 0, 255, 0, 4095);
	pwm.setPWM(10, 0, 0);
	pwm.setPWM(11, 0, Speed);
	pwm.setPWM(8, 0, 0);
	pwm.setPWM(9, 0, Speed);

	pwm.setPWM(13, 0, 0);
	pwm.setPWM(12, 0, Speed);
	pwm.setPWM(15, 0, 0);
	pwm.setPWM(14, 0, Speed);
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

/**
* Function       left
* @author        wusicaijuan
* @date          2019.06.26
* @brief         小车左移
* @param[in]     Speed
* @param[out]    void
* @retval        void
* @par History   无
*/
void left(int Speed)
{
	Speed = map(Speed, 0, 255, 0, 4095);
	pwm.setPWM(10, 0, Speed);
	pwm.setPWM(11, 0, 0);
	pwm.setPWM(8, 0, 0);
	pwm.setPWM(9, 0, Speed);

	pwm.setPWM(13, 0, 0);
	pwm.setPWM(12, 0, Speed);
	pwm.setPWM(15, 0, Speed);
	pwm.setPWM(14, 0, 0);
}

/**
* Function       right
* @author        wusicaijuan
* @date          2019.06.26
* @brief         小车右移
* @param[in]     Speed
* @param[out]    void
* @retval        void
* @par History   无
*/
void right(int Speed)
{
	Speed = map(Speed, 0, 255, 0, 4095);
	pwm.setPWM(10, 0, 0);
	pwm.setPWM(11, 0, Speed);
	pwm.setPWM(8, 0, Speed);
	pwm.setPWM(9, 0, 0);

	pwm.setPWM(13, 0, Speed);
	pwm.setPWM(12, 0, 0);
	pwm.setPWM(15, 0, 0);
	pwm.setPWM(14, 0, Speed);
}

/**
* Function       spin_left
* @author        wusicaijuan
* @date          2019.06.25
* @brief         小车原地左转(左轮后退，右轮前进)
* @param[in]     Speed
* @param[out]    void
* @retval        void
* @par History   无
*/
void spin_left(int Speed)
{
	Speed = map(Speed, 0, 255, 0, 4095);
	pwm.setPWM(10, 0, Speed);
	pwm.setPWM(11, 0, 0);
	pwm.setPWM(8, 0, Speed);
	pwm.setPWM(9, 0, 0);

	pwm.setPWM(13, 0, 0);
	pwm.setPWM(12, 0, Speed);
	pwm.setPWM(15, 0, 0);
	pwm.setPWM(14, 0, Speed);
}

/**
* Function       spin_right
* @author        wusicaijuan
* @date          2019.06.25
* @brief         小车原地右转(右轮后退，左轮前进)
* @param[in]     Speed
* @param[out]    void
* @retval        void
* @par History   无
*/
void spin_right(int Speed)
{
	Speed = map(Speed, 0, 255, 0, 4095);
	pwm.setPWM(10, 0, 0);
	pwm.setPWM(11, 0, Speed);
	pwm.setPWM(8, 0, 0);
	pwm.setPWM(9, 0, Speed);

	pwm.setPWM(13, 0, Speed);
	pwm.setPWM(12, 0, 0);
	pwm.setPWM(15, 0, Speed);
	pwm.setPWM(14, 0, 0);
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

void setServoPulse(uint8_t n, double pulse)
{
	double pulselength;

	pulselength = 1000000; // 1,000,000 us per second
	pulselength /= 60;	 // 60 Hz
	//Serial.print(pulselength); Serial.println(" us per period");
	pulselength /= 4096; // 12 bits of resolution
	//Serial.print(pulselength); Serial.println(" us per bit");
	pulse *= 1000;
	pulse /= pulselength;
	//Serial.println(pulse);
	pwm.setPWM(n, 0, pulse);
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
	mpu6050_getdata();
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
		mecanum_run(90 * 180 / M_PI, CarSpeedControl, 0);
		break;
	case enLEFT:
		mecanum_run(0 * 180 / M_PI, CarSpeedControl, 0);
		break;
	case enRIGHT:
		mecanum_run(a * 180 / M_PI, CarSpeedControl, 0);
		break;
	case enBACK:
		mecanum_run(a * 180 / M_PI, CarSpeedControl, 0);
		break;
	case enSPINLEFT:
		mecanum_run(a * 180 / M_PI, 0, M_PI/2);
		break;
	case enSPINRIGHT:
		mecanum_run(a * 180 / M_PI, 0, -M_PI/2);
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

#ifdef OUTPUT_READABLE_YAWPITCHROLL
		// display Euler angles in degrees
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
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

#endif

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

//计算偏航角ypr[0]
float cal_omega(float a)
{
	return (a * 180 / M_PI);
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
// void mecanum_run(float car_alpha, int speed, int car_omega)
// {
// 	float speed_x = speed * cos(car_alpha);
// 	float speed_y = speed * sin(car_alpha);
// 	float speed_omega = speed * sin(car_omega);
// 	float wheel1 = speed_y - speed_x + speed_omega;
// 	float wheel2 = speed_y + speed_x - speed_omega;
// 	float wheel3 = speed_y - speed_x - speed_omega;
// 	float wheel4 = speed_y + speed_x + speed_omega;
// 	wheel_run(1, wheel1);
// 	wheel_run(2, wheel2);
// 	wheel_run(3, wheel3);
// 	wheel_run(4, wheel4);
// }
// void wheel_run(int a, float speed)
// {
// 	speed = speed * 16; //map 255 to 4096
// 	if (speed >= 0)
// 	{
// 		pwm.setPWM(wheel[a - 1][0], 0, speed);
// 		pwm.setPWM(wheel[a - 1][1], 0, 0);
// 	}
// 	else
// 	{
// 		pwm.setPWM(wheel[a - 1][0], 0, 0);
// 		pwm.setPWM(wheel[a - 1][1], 0, -speed);
// 	}
// }
void mecanum_run(float car_alpha, int speed, int car_omega)
{
	speed = speed * 16; //map 255 to 4096
	float speed_x = speed * cos(car_alpha);
	float speed_y = speed * sin(car_alpha);
	float speed_omega = speed * sin(car_omega);
	float wheel_speed[4];
	wheel_speed[0] = speed_y - speed_x + speed_omega;
	wheel_speed[1] = speed_y + speed_x - speed_omega;
	wheel_speed[2] = speed_y - speed_x - speed_omega;
	wheel_speed[3] = speed_y + speed_x + speed_omega;
	for(int i = 0; i < 4; i++){
		if (wheel[i] >= 0)
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

struct car_omega
{
	/* data */
	float Kp = 0, Ki = 0, Kd = 0;
	float error = 0, P = 0, I = 0, D = 0, PID_value = 0;
	float previous_error = 0, previous_I = 0;
};

struct car_alpha
{
	/* data */
	float Kp = 0, Ki = 0, Kd = 0;
	float error = 0, P = 0, I = 0, D = 0, PID_value = 0;
	float previous_error = 0, previous_I = 0;
};

void calculate_pid(float Kp, float Ki, float Kd, float error, float P, float I, float D, float PID_value, float previous_error, float previous_I)
{
	P = error;
	I = I + previous_I;
	D = error - previous_error;

	PID_value = (Kp * P) + (Ki * I) + (Kd * D);
	// Serial.println(PID_value);

	previous_I = I;
	previous_error = error;
}
