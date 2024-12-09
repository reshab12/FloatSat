#include "readIMU.hpp"

struct Offsets offsets;
struct Attitude attitude;
struct SensorData sensorData;

float lastTime = 0;
float timeNow = 0;
float deltaTime = 0;

void i2cerror(){
	imu.reset();
	AT(NOW() + 5*MILLISECONDS);
	imu.init(400000);
}

void initialize(){
	imu.init(400000);
	imu.write(LSM9DS1_AG, LSM9DS1_CTRL_REG1_G, 2);
	imu.write(LSM9DS1_AG, LSM9DS1_CTRL_REG6_XL, 2);
	imu.write(LSM9DS1_M, LSM9DS1_CTRL_REG1_M, 2);
	imu.write(LSM9DS1_M, LSM9DS1_CTRL_REG2_M, 2);
	imu.write(LSM9DS1_M, LSM9DS1_CTRL_REG3_M, 2);
	imu.write(LSM9DS1_M, LSM9DS1_CTRL_REG4_M, 2);
}

float computeDeltaTime(){
	timeNow = NOW();
	deltaTime = (lastTime - timeNow)/(double)SECONDS;
	lastTime = timeNow;
	return deltaTime;
}

void readAccel(int16_t* xyzCoordinates){
	uint8_t DATA[6];
	if(imu.writeRead(LSM9DS1_AG, LSM9DS1_OUT_X_XL_L, 1, DATA, 6) <= 0) i2cerror();

	xyzCoordinates[0] = (int16_t) ((DATA[1]<<8) | DATA[0]);
	xyzCoordinates[1] = (int16_t) ((DATA[3]<<8) | DATA[2]);
	xyzCoordinates[2] = (int16_t) ((DATA[5]<<8) | DATA[4]);
}

void calcAccel(SensorData* data){
	int16_t xyzCoordinates[3];
	readAccel(xyzCoordinates);
	data->accel[0] = xyzCoordinates[0] * 0.000061;
	data->accel[1] = xyzCoordinates[1] * 0.000061;
	data->accel[2] = xyzCoordinates[2] * 0.000061;
}

/*void offsetAccel(Offsets* offset, int axis){
	int16_t xyzCoordinates[3];

	switch(axis){
	case 2:
		for(int i = 0; i<50; i++){
			readAccel(xyzCoordinates);
			offset->accel[0] = offset->accel[0] + xyzCoordinates[0];
			offset->accel[1] = offset->accel[1] + xyzCoordinates[1];
		}
		break;
	case 0:
		for(int i = 0; i<50; i++){
			readAccel(xyzCoordinates);
			offset->accel[2] = offset->accel[2] + xyzCoordinates[2];
			offset->accel[1] = offset->accel[1] + xyzCoordinates[1];
		}
		break;
	case 1:
		for(int i = 0; i<50; i++){
			readAccel(xyzCoordinates);
			offset->accel[0] = offset->accel[0] + xyzCoordinates[0];
			offset->accel[2] = offset->accel[2] + xyzCoordinates[2];
		}
		break;
	}
}*/

float calcPitch(SensorData* data){
	float numerator = -(data->accel[1]);
	float denominator = sqrt(data->accel[0] * data->accel[0] + data->accel[2] * data->accel[2]);
	return atan2(numerator, denominator);
}

float calcRoll(SensorData* data){
	float numerator = data->accel[0];
	float denominator = sqrt(data->accel[1] * data->accel[1] + data->accel[2] * data->accel[2]);
	return atan2(numerator, denominator);
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
	offset->gyro[0] = (offset->gyro[0]/10000)*0.07;
	offset->gyro[1] = (offset->gyro[1]/10000)*0.07;
	offset->gyro[2] = (offset->gyro[2]/10000)*0.07;
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
	uint8_t magOffsetMin = 0;
	uint8_t magOffsetMax = 0;
	int16_t offsetMag = 0;
	PRINTF("Rotate about x-Axis");
	for(int i = 0; i<1000; i++){
		readMagneto(xyzCoordinates);
		if(xyzCoordinates[0] < offset->magnetoMin[0]) offset->magnetoMin[0] = xyzCoordinates[0];
		else if(xyzCoordinates[0] > offset->magnetoMax[0]) offset->magnetoMax[0] = xyzCoordinates[0];
		AT(NOW()+ 10 * MILLISECONDS);
	}
	PRINTF("Max: %f \r\n Min: %f \r\n", offset->magnetoMax[0], offset->magnetoMin[0]);
	PRINTF("Rotate about y-Axis");
	for(int i = 0; i<1000; i++){
		readMagneto(xyzCoordinates);
		if(xyzCoordinates[1] < offset->magnetoMin[1]) offset->magnetoMin[1] = xyzCoordinates[1];
		else if(xyzCoordinates[1] > offset->magnetoMax[1]) offset->magnetoMax[1] = xyzCoordinates[1];
		AT(NOW()+ 10 * MILLISECONDS);
	}
	PRINTF("Max: %f \r\n Min: %f \r\n", offset->magnetoMax[1], offset->magnetoMin[1]);
	PRINTF("Rotate about z-Axis");
	for(int i = 0; i<1000; i++){
		readMagneto(xyzCoordinates);
		if(xyzCoordinates[2] < offset->magnetoMin[2]) offset->magnetoMin[2] = xyzCoordinates[2];
		else if(xyzCoordinates[2] > offset->magnetoMax[2]) offset->magnetoMax[2] = xyzCoordinates[2];
		AT(NOW()+ 10 * MILLISECONDS);
	}
	PRINTF("Max: %f \r\n Min: %f \r\n", offset->magnetoMax[2], offset->magnetoMin[2]);
}

void calibrateMagneto(Offsets* offsets, int16_t x, int16_t y, int16_t z, double calibratedMagneto[3]){
	calibratedMagneto[0] = ((x - offsets->magnetoMin[0])/(offsets->magnetoMax[0] - offsets->magnetoMin[0]) * 2 - 1);
	calibratedMagneto[1] = ((y - offsets->magnetoMin[1])/(offsets->magnetoMax[1] - offsets->magnetoMin[1]) * 2 - 1);
	calibratedMagneto[2] = ((z - offsets->magnetoMin[2])/(offsets->magnetoMax[2] - offsets->magnetoMin[2]) * 2 - 1);
	//calibratedMagneto[0] = x - (abs(offsets->magnetoMax[0]) + abs(offsets->magnetoMin[0]))/2;
	//calibratedMagneto[1] = y - (abs(offsets->magnetoMax[1]) + abs(offsets->magnetoMin[1]))/2;
	//calibratedMagneto[2] = z - (abs(offsets->magnetoMax[2]) + abs(offsets->magnetoMin[2]))/2;
}

void calcHeadingGyro(Attitude* attitude, int16_t gyro, Offsets* offset, float deltaTime){
	float temp;
	temp = attitude->headingGyro + (gyro * 0.07 - offset->gyro[2]) * deltaTime;
	if(temp >= 360) temp -= 360;
	attitude->headingGyro = temp;
}

void calcHeadingMagneto(Attitude* attitude, double magneto[3], Offsets* offset, float roll, float pitch){
	/*Todo figure out how to get pitch and roll
	* Then use mxh and myh in atan2 */
	float mxh = magneto[0] * cos(pitch) + magneto[2] * sin(pitch);
	float myh = magneto[0] * sin(roll) * sin(pitch) + magneto[1] * cos(roll) - magneto[2] * sin(roll) * cos(pitch);
	
	attitude->headingMagneto = atan2(magneto[1],magneto[0]) * 180/M_PI + 180;
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
		int16_t xyzAccel[3];
        offsetGyro(&offsets);
		offsetMagneto(&offsets);
        float gx, gy, gz;
		double calibratedMagneto[3];
        TIME_LOOP(1 * SECONDS, 500 * MILLISECONDS){
			readGyro(xyzGyro);
			readMagneto(xyzMagneto);
			readAccel(xyzAccel);
			calcAccel(&sensorData);
			deltaTime = computeDeltaTime();
			calcHeadingGyro(&attitude, xyzGyro[2], &offsets, deltaTime);
			calibrateMagneto(&offsets, xyzMagneto[0], xyzMagneto[1], xyzMagneto[2], calibratedMagneto);
			float pitch = calcPitch(&sensorData);
			float roll = calcRoll(&sensorData);
			calcHeadingMagneto(&attitude, calibratedMagneto, &offsets, roll, pitch);

    		//gx = (xyzGyro[0] - offsets.gyro[0]) * 0.07;
            //gy = (xyzGyro[1] - offsets.gyro[1]) * 0.07;
            //gz = (xyzGyro[2] - offsets.gyro[2]) * 0.07;

            PRINTF("heading: %f degrees \r\n", attitude.headingMagneto);
			//MW_PRINTF("gx: %f Gauss  gy: %f Gauss  gz: %f Gauss \r\n", (xyzMagneto[0]*0.00014), (xyzMagneto[1]*0.00014), (xyzMagneto[2]*0.00014));
        }
    }
};

Sensor sensorThread("readIMU");
