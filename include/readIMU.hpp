#include "rodos.h"
#include <stdio.h>
#include "hal.h"
#include "matlib.h"


#define LSM9DS1_AG 0x6B /* Accelerometer and Gyroscope 7-bit I2C address */
#define LSM9DS1_M 0x1E /* Magnetometer 7-bit I2C address */

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
	float magnetoMin[3] = {0,0,0};
	float magnetoMax[3] = {0,0,0};
	float accel[3] = {0,0,0};
};

struct Attitude{
	float headingMagneto = 0;
	float headingGyro = 0;
};


void i2cerror();

void initialize();

void readAccel(int16_t* xyzCoordinates);

void offsetAccel(Offsets* offset);

void readGyro(int16_t* xyzCoordinates);

void offsetGyro(Offsets* offset);

void readMagneto(int16_t* xyzCoordinates);

void offsetMagneto(Offsets* offset);

void calibrateMagneto(Offsets* offset, int16_t data, float* calibratedMagneto);

void calcHeadingMagneto(Attitude* attitude, int16_t magneto[3], Offsets * offset);

void calcHeadingGyro(Attitude* Gyro, int16_t gyro[3], Offsets * offset);

/*class Sensor : public StaticThread<>{
public:
	Sensor(const char* name) : StaticThread<>(name){};
	void init();
	void run();
};*/