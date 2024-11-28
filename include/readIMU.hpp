#include "rodos.h"
#include <stdio.h>
#include "hal.h"
#include "matlib.h"
#include "math.h"


#define LSM9DS1_AG 0x6B /* Accelerometer and Gyroscope 7-bit I2C address */
#define LSM9DS1_M 0x1E /* Magnetometer 7-bit I2C address */
// The Magnetometer Offset Registers.
#define OFFSET_X_REG_L_M	0x05
#define OFFSET_X_REG_H_M	0x06
#define OFFSET_Y_REG_L_M	0x07
#define OFFSET_Y_REG_H_M	0x08
#define OFFSET_Z_REG_L_M	0x09
#define OFFSET_Z_REG_H_M	0x0A

HAL_I2C imu(I2C_IDX1);

static HAL_UART bluetooth(UART_IDX2); // Tx: PD5, Rx: PD6
static LinkinterfaceUART link_name_not_imp(&bluetooth, 115200, 3, 10);
static Gateway gw_name_not_imp(&link_name_not_imp, true);

uint8_t LSM9DS1_WHO_AM_I[1] = {0x0F}; /* WHO_AM_I register address with a value of 0x68 in the AG sensor and 0x3D in the M sensor*/
uint8_t LSM9DS1_OUT_X_G_L[1] = {0x18}; /* The starting data register address for the Gyroscope */
uint8_t LSM9DS1_OUT_X_XL_L[1] = {0x28}; //Accelerometer
uint8_t LSM9DS1_OUT_X_L_M[1] = {0x28}; //Magnetometer
uint8_t LSM9DS1_OUT_X_TEMP_L[1] = {0x15}; //Temperatur

uint8_t LSM9DS1_CTRL_REG1_G[2] = {0x10,0b11011011}; /* {Gyroscope CTRL_REG1_G register address, Value needs to be written} */
uint8_t LSM9DS1_CTRL_REG6_XL[2] = {0x20,0b11000000};
uint8_t LSM9DS1_CTRL_REG1_M[2] = {0x20,0b01111100};
uint8_t LSM9DS1_CTRL_REG2_M[2] = {0x21,0b00000000};
uint8_t LSM9DS1_CTRL_REG3_M[2] = {0x22,0b00000000};
uint8_t LSM9DS1_CTRL_REG4_M[2] = {0x23,0b00001100};

struct Offsets{
    float gyro[3] = {0,0,0};
	double magnetoMin[3] = {9999999.9,9999999.9,9999999.9};
	double magnetoMax[3] = {-9999999.9,-9999999.9,-9999999.9};
	//float accel[3] = {0,0,0};
};

struct Attitude{
	double headingMagneto = 0;
	float headingGyro = 0;
};

struct SensorData{
	float accel[3] = {0,0,0};
	float magneto[3] = {0,0,0};
	float gyro[3] = {0,0,0};
};


void i2cerror();

void initialize();

void readAccel(int16_t* xyzCoordinates);

void calcAccel(SensorData* data);

float calcRoll(SensorData* data);

float calcPitch(SensorData* data);

//void offsetAccel(Offsets* offset);

void readGyro(int16_t* xyzCoordinates);

void offsetGyro(Offsets* offset);

void readMagneto(int16_t* xyzCoordinates);

void offsetMagneto(Offsets* offset);

void calibrateMagneto(Offsets* offsets, int16_t x, int16_t y, int16_t z, double calibratedMagneto[3]);

float computeDeltaTime();

void calcHeadingMagneto(Attitude* attitude, double magneto[3], Offsets* offset, float roll, float pitch);

void calcHeadingGyro(Attitude* attitude, int16_t gyro, Offsets* offset, float deltaTime);

/*class Sensor : public StaticThread<>{
public:
	Sensor(const char* name) : StaticThread<>(name){};
	void init();
	void run();
};*/