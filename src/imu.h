//Arduino/Teensy Flight Controller - dRehmFlight
//Author: Nicholas Rehm
//Project Start: 1/6/2020
//Version: Beta 1.2

/*
 * 
 * If you are using this for an academic or scholarly project, please credit me in any presentations or publications:
 *
 * Nicholas Rehm
 * Department of Aerospace Engineering
 * University of Maryland
 * College Park 20742
 * Email: nrehm@umd.edu
 *
 */
 
//========================================================================================================================//

//CREDITS + SPECIAL THANKS
/*
Some elements inspired by:
http://www.brokking.net/ymfc-32_main.html
Skeleton code for reading and initializing MPU6050 borrowed from:
https://howtomechatronics.com/tutorials/arduino/arduino-and-mpu6050-accelerometer-and-gyroscope-tutorial/
Madgwick filter function adapted from:
https://github.com/arduino-libraries/MadgwickAHRS
MPU9250 implementation based on MPU9250 library by
brian.taylor@bolderflight.com
http://www.bolderflight.com
Thank you to:
RcGroups 'jihlein' - IMU implementation overhaul + SBUS implementation
*/



//========================================================================================================================//
//                                                 USER-SPECIFIED DEFINES                                                 //                                                                 
//========================================================================================================================//

//Uncomment only one receiver type
#define USE_PWM_RX
//#define USE_PPM_RX
//#define USE_SBUS_RX

//Uncomment only one IMU
#define USE_MPU6050_I2C //default
//#define USE_MPU9250_SPI

//Uncomment only one full scale gyro range (deg/sec)
#define GYRO_250DPS //default
//#define GYRO_500DPS
//#define GYRO_1000DPS
//#define GYRO_2000DPS

//Uncomment only one full scale accelerometer range (G's)
#define ACCEL_2G //default
//#define ACCEL_4G
//#define ACCEL_8G
//#define ACCEL_16G



//========================================================================================================================//



//REQUIRED LIBRARIES (included with download in main sketch folder)

// #include <Wire.h>     //I2c communication
// #include <SPI.h>      //SPI communication
// #include <PWMServo.h> //commanding any extra actuators, installed with teensyduino installer

// #if defined USE_SBUS_RX
//   #include "src/SBUS/SBUS.h"   //sBus interface
// #endif

// #if defined USE_MPU6050_I2C
//   #include "MPU6050.h"
//   MPU6050 mpu6050;
// #elif defined USE_MPU9250_SPI
//   #include "src/MPU9250/MPU9250.h"
//   MPU9250 mpu9250(SPI2,36);
// #else
//   #error No MPU defined... 
// #endif



//========================================================================================================================//



//Setup gyro and accel full scale value selection and scale factor

// #if defined USE_MPU6050_I2C
//   #define GYRO_FS_SEL_250    MPU6050_GYRO_FS_250
//   #define GYRO_FS_SEL_500    MPU6050_GYRO_FS_500
//   #define GYRO_FS_SEL_1000   MPU6050_GYRO_FS_1000
//   #define GYRO_FS_SEL_2000   MPU6050_GYRO_FS_2000
//   #define ACCEL_FS_SEL_2     MPU6050_ACCEL_FS_2
//   #define ACCEL_FS_SEL_4     MPU6050_ACCEL_FS_4
//   #define ACCEL_FS_SEL_8     MPU6050_ACCEL_FS_8
//   #define ACCEL_FS_SEL_16    MPU6050_ACCEL_FS_16
// #elif defined USE_MPU9250_SPI
//   #define GYRO_FS_SEL_250    mpu9250.GYRO_RANGE_250DPS
//   #define GYRO_FS_SEL_500    mpu9250.GYRO_RANGE_500DPS
//   #define GYRO_FS_SEL_1000   mpu9250.GYRO_RANGE_1000DPS                                                        
//   #define GYRO_FS_SEL_2000   mpu9250.GYRO_RANGE_2000DPS
//   #define ACCEL_FS_SEL_2     mpu9250.ACCEL_RANGE_2G
//   #define ACCEL_FS_SEL_4     mpu9250.ACCEL_RANGE_4G
//   #define ACCEL_FS_SEL_8     mpu9250.ACCEL_RANGE_8G
//   #define ACCEL_FS_SEL_16    mpu9250.ACCEL_RANGE_16G
// #endif
  
// #if defined GYRO_250DPS
//   #define GYRO_SCALE GYRO_FS_SEL_250
//   #define GYRO_SCALE_FACTOR 131.0
// #elif defined GYRO_500DPS
//   #define GYRO_SCALE GYRO_FS_SEL_500
//   #define GYRO_SCALE_FACTOR 65.5
// #elif defined GYRO_1000DPS
//   #define GYRO_SCALE GYRO_FS_SEL_1000
//   #define GYRO_SCALE_FACTOR 32.8
// #elif defined GYRO_2000DPS
//   #define GYRO_SCALE GYRO_FS_SEL_2000
//   #define GYRO_SCALE_FACTOR 16.4
// #endif

// #if defined ACCEL_2G
//   #define ACCEL_SCALE ACCEL_FS_SEL_2
//   #define ACCEL_SCALE_FACTOR 16384.0
// #elif defined ACCEL_4G
//   #define ACCEL_SCALE ACCEL_FS_SEL_4
//   #define ACCEL_SCALE_FACTOR 8192.0
// #elif defined ACCEL_8G
//   #define ACCEL_SCALE ACCEL_FS_SEL_8
//   #define ACCEL_SCALE_FACTOR 4096.0
// #elif defined ACCEL_16G
//   #define ACCEL_SCALE ACCEL_FS_SEL_16
//   #define ACCEL_SCALE_FACTOR 2048.0
// #endif



//========================================================================================================================//
//                                               USER-SPECIFIED VARIABLES                                                 //                           
//========================================================================================================================//


//Filter parameters - Defaults tuned for 2kHz loop rate; Do not touch unless you know what you are doing:
// float B_madgwick ;  //Madgwick filter parameter
// float B_accel ;     //Accelerometer LP filter paramter, (MPU6050 default: 0.14. MPU9250 default: 0.2)
// float B_gyro ;       //Gyro LP filter paramter, (MPU6050 default: 0.1. MPU9250 default: 0.17)
// float B_mag ;        //Magnetometer LP filter parameter

//Magnetometer calibration parameters - if using MPU9250, uncomment calibrateMagnetometer() in void setup() to get these values, else just ignore these
// float MagErrorX ;
// float MagErrorY ; 
// float MagErrorZ ;
// float MagScaleX ;
// float MagScaleY ;
// float MagScaleZ ;

//========================================================================================================================//
//                                                     DECLARE PINS                                                       //                           
//========================================================================================================================//                                          



//IMU:
// float AccX, AccY, AccZ;
// float AccX_prev, AccY_prev, AccZ_prev;
// float GyroX, GyroY, GyroZ;
// float GyroX_prev, GyroY_prev, GyroZ_prev;
// float MagX, MagY, MagZ;
// float MagX_prev, MagY_prev, MagZ_prev;
// float roll_IMU, pitch_IMU, yaw_IMU;
// float roll_IMU_prev, pitch_IMU_prev;
// float AccErrorX, AccErrorY, AccErrorZ, GyroErrorX, GyroErrorY, GyroErrorZ;
// float q0; //initialize quaternion for madgwick filter
// float q1;
// float q2;
// float q3;

struct ImuFiltered {
  float roll_IMU;
  float pitch_IMU;
  float yaw_IMU;
};

//========================================================================================================================//
//                                                      FUNCTIONS                                                         //                           
void IMUinit();
void getIMUdata(ImuFiltered *gyroState, float dt);
void calculate_IMU_error();
void calibrateAttitude();
void Madgwick(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float invSampleFreq, ImuFiltered *gyrostate);
void Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az, float invSampleFreq, ImuFiltered *gyroState);
float invSqrt(float x);