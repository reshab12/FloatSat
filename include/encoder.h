/*****************************************************************
Sensors.h

Original Created by: Atheel Redah @ University of W�rzburg
Original Creation Date: January 20, 2015

Development environment specifics:
	Software Platform: Rodos (Realtime Onboard Dependable Operating System).
	Hardware Platform: STM32F4 + W�rzburg Uni Informatik 8 Discovery AddOn Board Version 2.0.
*****************************************************************/

#ifndef Sensors_H_
#define Sensors_H_

/* Includes ------------------------------------------------------------------*/
#include "LSM9DS1.h"
#include "math.h"
#include "stdint.h"
//#include "topics.h"
#include "topics.hpp"
#include "stm32f4xx_conf.h"

/* ADC -------------------------------------------------------------*/
#define ADC01               ADC_IDX1       			// ADC1
#define ADCChannel          ADC_CH_001     			// PA1
#define ADCRes              4095                // ADC full scale resolution of 12 bits = 2^12-1 = 4095
#define ADCRef              3000.0              // ADC adc reference voltage  = 3V = 3000mV
#define CurrentVoltageRatio 100                 //Current-sense feedback voltage of approximately 500 mV per amp


/* Exported types ------------------------------------------------------------*/
enum AHRS_Mode
{
	Gyro_Cal = 1,
	Accel_Cal = 2,
	Mag_Cal = 3,
	Gyro_Update = 4,
	Gyro_Quaternion_Update = 5,
	Acc_Mag_Tilted_Compass_Update = 6,
	Madgwick_Quaternion_Update = 7,
	Mahony_Quaternion_Update = 8,
	Extended_Kalman_Update = 9,
};

/* Exported functions ------------------------------------------------------- */

void AHRSUpdate(AHRS_Mode mode);

void GyroUpdate(float gx, float gy, float gz);

void GyroQuaternionUpdate(float gx, float gy, float gz);

void AccMagUpdate(float ax, float ay, float az, float mx, float my, float mz);

void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);

void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);

void Quaternion2Euler();

void MagCal(float* magMax, float* magMin, float magCalTime);

void GyroCal(float* gbias);

void AccelCal(float* abias);

void MotorSpeedUpdate(motor_data* motor);

void ADCUpdate();

void EncoderInit();

#endif /* Sensors_H_ */
