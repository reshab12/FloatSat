#include "rodos.h"
#include <stdio.h>
#include "hal.h"
#include "math.h"

HAL_I2C imu(I2C_IDX1);

#define LSM9DS1_AG 0x6B /* Accelerometer and Gyroscope 7-bit I2C address */
#define LSM9DS1_M 0x1E /* Magnetometer 7-bit I2C address */

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
    float gyro[3];
	float accel[3];
	float magneto[3];
}offsets;

void i2cerror(){
	imu.reset();
	AT(NOW() + 5*MILLISECONDS);
	imu.init(400000);
}

void initialize(){
	imu.init(400000);
	imu.write(LSM9DS1_AG, LSM9DS1_CTRL_REG1_G, 2);
	//imu.write(LSM9DS1_AG, LSM9DS1_CTRL_REG6_XL, 2);
	imu.write(LSM9DS1_M, LSM9DS1_CTRL_REG1_M, 2);
	imu.write(LSM9DS1_M, LSM9DS1_CTRL_REG2_M, 2);
	imu.write(LSM9DS1_M, LSM9DS1_CTRL_REG3_M, 2);
	imu.write(LSM9DS1_M, LSM9DS1_CTRL_REG4_M, 2);
}

void readGyro(int16_t* xyzCoordinates){
	uint8_t DATA[6];
	if(imu.writeRead(LSM9DS1_AG, LSM9DS1_OUT_X_G_L, 1, DATA, 6) <= 0) i2cerror();

	xyzCoordinates[0] = (int16_t) ((DATA[1]<<8) | DATA[0]);
	xyzCoordinates[1] = (int16_t) ((DATA[3]<<8) | DATA[2]);
	xyzCoordinates[2] = (int16_t) ((DATA[5]<<8) | DATA[4]);

}

void offsetGyro(Offsets* offset){

	int16_t xyzCoordinates[3];
	for(int i = 0; i<10000; i++){
		readGyro(xyzCoordinates);
		offset->gyro[0] = offset->gyro[0] + xyzCoordinates[0];
		offset->gyro[1] = offset->gyro[1] + xyzCoordinates[1];
		offset->gyro[2] = offset->gyro[2] + xyzCoordinates[2];
	}
	offset->gyro[0] = offset->gyro[0]/10000;
	offset->gyro[1] = offset->gyro[1]/10000;
	offset->gyro[2] = offset->gyro[2]/10000;
}

void readMagneto(int16_t* xyzCoordinates){
	uint8_t DATA[6];
	if(imu.writeRead(LSM9DS1_AG, LSM9DS1_OUT_X_L_M, 1, DATA, 6) <= 0) i2cerror();

	xyzCoordinates[0] = (int16_t) ((DATA[1]<<8) | DATA[0]);
	xyzCoordinates[1] = (int16_t) ((DATA[3]<<8) | DATA[2]);
	xyzCoordinates[2] = (int16_t) ((DATA[5]<<8) | DATA[4]);

}

void offsetMagneto(Offsets* offset){
    //ToDo
}

class Sensor : public StaticThread<>{
public:

	Sensor(const char* name) : StaticThread<>(name) {
	}

	void init() {
		initialize();
	}

	void run() {
        /*Uncomment if you want to checkt for WHO AM I
            uint8_t data = 0;
            stat = imu.writeRead(LSM9DS1_AG, LSM9DS1_WHO_AM_I,1,data,1);
            if(stat <= 0) i2cerror();
            PRINTF("I am %d! \r\n", data);
        */
        int16_t xyzGyro[3];
        offsetGyro(&offsets);
        float gx, gy, gz;
        TIME_LOOP(1 * SECONDS, 10 * MILLISECONDS){
            readGyro(xyzGyro);
            gx = (xyzGyro[0] - offsets.gyro[0]) * 0.07;
            gy = (xyzGyro[1] - offsets.gyro[1]) * 0.07;
            gz = (xyzGyro[2] - offsets.gyro[2]) * 0.07;
            //PRINTF("gx: %f dps \r\n gy: %f dps \r\n gz: %f dps \r\n", gx, gy, gz);
        }
    }
};

Sensor sensorThread("readIMU");
