#include "readIMU.hpp"

struct Offsets offsets;
struct Attitude attitude;

void i2cerror(){
	imu.reset();
	AT(NOW() + 5*MILLISECONDS);
	imu.init(400000);
}

void initialize(){
	imu.init(400000);
	imu.write(LSM9DS1_AG, LSM9DS1_CTRL_REG1_G, 2);
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
	if(imu.writeRead(LSM9DS1_M, LSM9DS1_OUT_X_L_M, 1, DATA, 6) <= 0) i2cerror();

	xyzCoordinates[0] = (int16_t) ((DATA[1]<<8) | DATA[0]);
	xyzCoordinates[1] = (int16_t) ((DATA[3]<<8) | DATA[2]);
	xyzCoordinates[2] = (int16_t) ((DATA[5]<<8) | DATA[4]);

}

void offsetMagneto(Offsets* offset){
    int16_t xyzCoordinates[3];
	PRINTF("Rotate about x-Axis");
	for(int i = 0; i<1000; i++){
		readMagneto(xyzCoordinates);
		if(xyzCoordinates[0] < offset->magnetoMin[0]) offset->magnetoMin[0] = xyzCoordinates[0];
		else if(xyzCoordinates[0] > offset->magnetoMax[0]) offset->magnetoMax[0] = xyzCoordinates[0];
		AT(NOW()+ 10 * MILLISECONDS);
	}
	PRINTF("Rotate about y-Axis");
	for(int i = 0; i<1000; i++){
		readMagneto(xyzCoordinates);
		if(xyzCoordinates[1] < offset->magnetoMin[1]) offset->magnetoMin[1] = xyzCoordinates[1];
		else if(xyzCoordinates[1] > offset->magnetoMax[1]) offset->magnetoMax[1] = xyzCoordinates[1];
		AT(NOW()+ 10 * MILLISECONDS);
	}
	PRINTF("Rotate about z-Axis");
	for(int i = 0; i<1000; i++){
		readMagneto(xyzCoordinates);
		if(xyzCoordinates[2] < offset->magnetoMin[2]) offset->magnetoMin[2] = xyzCoordinates[2];
		else if(xyzCoordinates[2] > offset->magnetoMax[2]) offset->magnetoMax[2] = xyzCoordinates[2];
		AT(NOW()+ 10 * MILLISECONDS);
	}
}

void calcHeadingGyro(Attitude* attitude, int16_t gyro[3], Offsets * offset){
	attitude->headingGyro = gyro[2] - offset->gyro[2];
}

class Sensor : public StaticThread<>{
public:
	Sensor(const char* name) : StaticThread(name){};

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
		int16_t xyzMagneto[3];
        //offsetGyro(&offsets);
		//offsetMagneto(&offsets);
        float gx, gy, gz;
		float mx, my, mz;
        TIME_LOOP(1 * SECONDS, 500* MILLISECONDS){
			readGyro(xyzGyro);
			calcHeadingGyro(&attitude, xyzGyro, &offsets);
            //readGyro(xyzGyro);
			//readMagneto(xyzMagneto);

    		gx = (xyzGyro[0] - offsets.gyro[0]) * 0.07;
            gy = (xyzGyro[1] - offsets.gyro[1]) * 0.07;
            gz = (xyzGyro[2] - offsets.gyro[2]) * 0.07;

			//mx = ((xyzMagneto[0] - offsets.magnetoMin[0])/(offsets.magnetoMax[0] - offsets.magnetoMin[0]) * 2 - 1) * 0.00014;
			//my = ((xyzMagneto[1] - offsets.magnetoMin[1])/(offsets.magnetoMax[1] - offsets.magnetoMin[1]) * 2 - 1) * 0.00014;
			//mz = ((xyzMagneto[2] - offsets.magnetoMin[2])/(offsets.magnetoMax[2] - offsets.magnetoMin[2]) * 2 - 1) * 0.00014;
            PRINTF("gx: %d Gauss  gy: %d Gauss gz: %d Gauss \r\n", gx, gy, gz);
			//MW_PRINTF("gx: %f Gauss  gy: %f Gauss  gz: %f Gauss \r\n", (xyzMagneto[0]*0.00014), (xyzMagneto[1]*0.00014), (xyzMagneto[2]*0.00014));
        }
    }
};

Sensor sensorThread("readIMU");
