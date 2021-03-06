// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps_V6_12.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

#define INTERRUPT_PIN 2 // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13		// (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;
int time_1 = 0;

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
VectorInt16 gy;		 // [x, y, z]            gyro sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float ypr[3];		 // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

float omega_Work = 0;

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

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
	mpuInterrupt = true;
}

void setup()
{

	Wire.begin();
	Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

	Serial.begin(115200);
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
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop()
{
	// if programming failed, don't try to do anything
	// if (!dmpReady)
	// 	return;



	// reset interrupt flag and get INT_STATUS byte
	// mpuInterrupt = false;
	// mpuIntStatus = mpu.getIntStatus();

	// // get current FIFO count
	// fifoCount = mpu.getFIFOCount();

	// // check for overflow (this should never happen unless our code is too inefficient)
	// if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024)
	// {
	// 	// reset so we can continue cleanly
	// 	mpu.resetFIFO();
	// 	fifoCount = mpu.getFIFOCount();
	// 	Serial.println(F("FIFO overflow!"));

	// 	// otherwise, check for DMP data ready interrupt (this should happen frequently)
	// }
	// else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT))
	// {
	// 	// wait for correct available data length, should be a VERY short wait
	// 	while (fifoCount < packetSize)
	// 		fifoCount = mpu.getFIFOCount();

		// read a packet from FIFO
		mpu.getFIFOBytes(fifoBuffer, packetSize);

		// track FIFO count here in case there is > 1 packet available
		// (this lets us immediately read more without waiting for an interrupt)
		// fifoCount -= packetSize;


		// display Euler angles in degrees
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
		Serial.print("ypr\t");
		Serial.print(cal_omega(ypr[0]));
		Serial.print("\t");
		Serial.print(ypr[1] * 180 / M_PI);
		Serial.print("\t");
		Serial.print(ypr[2] * 180 / M_PI);
		mpu.dmpGetAccel(&aa, fifoBuffer);
		Serial.print("\tRaw Accl XYZ\t");
		Serial.print(aa.x);
		Serial.print("\t");
		Serial.print(aa.y);
		Serial.print("\t");
		Serial.print(cal_angle(-aa.x, aa.y));
		Serial.print("\t");
		Serial.print(aa.z);
		mpu.dmpGetGyro(&gy, fifoBuffer);
		Serial.print("\tRaw Gyro XYZ\t");
		Serial.print(gy.x);
		Serial.print("\t");
		Serial.print(gy.y);
		Serial.print("\t");
		Serial.print(gy.z);
		Serial.print("\t");
		Serial.print(millis());
		Serial.print("\t");
		Serial.print(time_1);
		omega_Work = PIDCalc(ypr[0]);
		Serial.print("\t");
		Serial.print(millis());
		Serial.println();
		time_1++;


		// // blink LED to indicate activity
		// blinkState = !blinkState;
		// digitalWrite(LED_PIN, blinkState);
	// }
}

//计算移动方向角
float cal_angle(float y, float x)
{
	return atan2(int(y / 200) * 200, int(x / 200) * 200);
}

//计算偏航角
float cal_omega(float a)
{
	return (a * 180 / M_PI);
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
