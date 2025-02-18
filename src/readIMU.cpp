#include "readIMU.hpp"


HAL_I2C imu(I2C_IDX1);

HAL_GPIO pin_green_led(GPIO_060);
HAL_GPIO pin_blue_led(GPIO_063);

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
	pin_green_led.init(true,1,0);
	pin_blue_led.init(true,1,0);
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


float calcPitch(SensorData* data){
	float numerator = -(data->accel[1]);
	float denominator = sqrt(data->accel[0] * data->accel[0] + data->accel[2] * data->accel[2]);
	return atan2(numerator, denominator);
}

float calcRoll(SensorData* data){
	float numerator = data->accel[2];
	float denominator = sqrt(data->accel[1] * data->accel[1] + data->accel[0] * data->accel[0]);
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
	pin_green_led.setPins(1);
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
	pin_green_led.setPins(0);
}

void readMagneto(int16_t* xyzCoordinates){
	uint8_t DATA[6];
	if(imu.writeRead(LSM9DS1_M, LSM9DS1_OUT_X_L_M, 1, DATA, 6) <= 0) i2cerror();

	xyzCoordinates[0] = (int16_t) ((DATA[1]<<8) | DATA[0]);
	xyzCoordinates[1] = (int16_t) ((DATA[3]<<8) | DATA[2]);
	xyzCoordinates[2] = (int16_t) ((DATA[5]<<8) | DATA[4]);

}

void offsetMagneto(Offsets* offset){
	pin_blue_led.setPins(1);
    int16_t xyzCoordinates[3];

	//MW_PRINTF("Rotate about z-Axis");
	for(int i = 0; i<1000; i++){
		readMagneto(xyzCoordinates);
		if(xyzCoordinates[2] < offset->magnetoMin[2]) offset->magnetoMin[2] = xyzCoordinates[2];
		else if(xyzCoordinates[2] > offset->magnetoMax[2]) offset->magnetoMax[2] = xyzCoordinates[2];
		if(xyzCoordinates[1] < offset->magnetoMin[1]) offset->magnetoMin[1] = xyzCoordinates[1];
		else if(xyzCoordinates[1] > offset->magnetoMax[1]) offset->magnetoMax[1] = xyzCoordinates[1];
		if(xyzCoordinates[0] < offset->magnetoMin[0]) offset->magnetoMin[0] = xyzCoordinates[0];
		else if(xyzCoordinates[0] > offset->magnetoMax[0]) offset->magnetoMax[0] = xyzCoordinates[0];
		AT(NOW()+ 10 * MILLISECONDS);
	}
	//MW_PRINTF("Max: %f \r\n Min: %f \r\n", offset->magnetoMax[0], offset->magnetoMin[0]);
	//MW_PRINTF("Max: %f \r\n Min: %f \r\n", offset->magnetoMax[1], offset->magnetoMin[1]);
	pin_blue_led.setPins(0);
}

void calibrateMagneto(Offsets* offsets, int16_t x, int16_t y, int16_t z, float calibratedMagneto[3]){
	calibratedMagneto[0] = ((x - offsets->magnetoMin[0])/(offsets->magnetoMax[0] - offsets->magnetoMin[0]) * 2 - 1);
	calibratedMagneto[1] = ((y - offsets->magnetoMin[1])/(offsets->magnetoMax[1] - offsets->magnetoMin[1]) * 2 - 1);
	calibratedMagneto[2] = ((z - offsets->magnetoMin[2])/(offsets->magnetoMax[2] - offsets->magnetoMin[2]) * 2 - 1);
	//calibratedMagneto[0] = x - (abs(offsets->magnetoMax[0]) + abs(offsets->magnetoMin[0]))/2;
	//calibratedMagneto[1] = y - (abs(offsets->magnetoMax[1]) + abs(offsets->magnetoMin[1]))/2;
	//calibratedMagneto[2] = z - (abs(offsets->magnetoMax[2]) + abs(offsets->magnetoMin[2]))/2;
}

void calcHeadingGyro(Attitude* attitude, int16_t gyro, Offsets* offset, float deltaTime){
	float temp;
	temp = attitude->headingGyro + (gyro * 0.07 - offset->gyro[1]) * deltaTime;
	if(temp >= 360) temp -= 360;
	attitude->headingGyro = temp;
}

void calcHeadingMagnetoPitchRoll(Attitude* attitude, float magneto[3], float roll, float pitch){
	/*Todo figure out how to get pitch and roll
	* Then use mxh and myh in atan2 */
	float mxh = magneto[0] * cos(pitch) + magneto[2] * sin(pitch);
	float myh = magneto[0] * sin(roll) * sin(pitch) + magneto[1] * cos(roll) - magneto[2] * sin(roll) * cos(pitch);
	
	attitude->headingMagneto = atan2(mxh,myh) * 180/M_PI + 180;
}

void calcHeadingMagneto(Attitude* attitude, float magneto[3]){
	attitude->headingMagneto = atan2(magneto[0],magneto[1]) * 180/M_PI + 180;
}


Sensor::Sensor(const char* name, int32_t priority) : StaticThread(name, priority){};

void Sensor::updateDt(){
	dt= (RODOS::NOW()-t)/ SECONDS;
	t = RODOS::NOW();

	//A.r[0][1]=dt;
	G.r[0][0]= dt;
	//G.r[0][1]= 0;

	

	Q.r[0][0]= dt*dt * R_GYROX;
	Q.r[0][1]= dt*R_GYROX;
	Q.r[1][0]= dt*R_GYROX;
	Q.r[1][1]= R_GYROX;

}

float mod(float in){
	if( in < -180)
		in += 360;
	else if( in > 180)
		in -= 360;
	return in;
}


void Sensor::update(const Matrix_<1,1,float> & y, float u){

	if(!initialized)
		RODOS::PRINTF("NOT INITIALIZED!!!!! WATCH OUT FOR SHARKS!!!!!");

	x_hat_new = A * x_hat + G * u;

	x_hat_new.r[0][0] = mod(x_hat_new.r[0][0]);

	P = A*P*A.transpose() + Q;
	K = P*C.transpose()*(C*P*C.transpose() + R).invert();

	Matrix_<1,1,float> error = (y - C*x_hat_new);

	error.r[0][0] = mod(error.r[0][0]);

	//MW_PRINTF("%f %f %f %f %f\n",(C*x_hat_new).r[0][0],x_hat_new.r[0][0],y.r[0][0] , error.r[0][0]);

	//K.r[0][0] = 1;
	//K.r[0][1] = 1;

	x_hat_new = x_hat_new + K * error;


	x_hat_new.r[0][0] = mod(x_hat_new.r[0][0]); 

	P = (I - K*C)*P;
	x_hat = x_hat_new;
}

void Sensor::init() {
	initialize();

	//R_Gyro = 0.01;
	dt = 100 * MILLISECONDS/100000000;

	updateDt();
	 
   	// dt = dt/10;
	//dt=0.02;

	A.r[0][0]=1;							
	A.r[0][1]=0;
	A.r[1][0]=0;
	A.r[1][1]=0;

	G.r[0][0]=dt;					
	G.r[0][1]=1;

	C.r[0][0]=1;
	C.r[0][1]=0;

	Q.r[0][0]=dt*dt * R_GYROX;		
	Q.r[0][1]=dt*R_GYROX;
	Q.r[1][0]=dt*R_GYROX;
	Q.r[1][1]=R_GYROX;

	R.r[0][0]=10;							

	P.r[0][0]=1.;							
	P.r[1][1]=1.;

	I.r[0][0]=1.;							
	I.r[1][1]=1.;

	/*RODOS::PRINTF("dt: %f \n",dt);
	RODOS::PRINTF("Rgyro: %f \n",R_Gyro);
	RODOS::PRINTF("A: %f %f\n   %f %f\n",A.r[0][0],A.r[0][1],A.r[1][0],A.r[1][1]);
	RODOS::PRINTF("G: %f \n   %f \n",G.r[0][0],G.r[0][1]);
	RODOS::PRINTF("C: %f \n   %f \n",C.r[0][0],C.r[0][1]);
	RODOS::PRINTF("Q: %f %f\n   %f %f\n",Q.r[0][0],Q.r[0][1],Q.r[1][0],Q.r[1][1]);
	RODOS::PRINTF("R: %f \n",R.r[0][0]);
	RODOS::PRINTF("P: %f %f\n   %f %f\n",P.r[0][0],P.r[0][1],P.r[1][0],P.r[1][1]);*/
	initialized=true;						//all initiialized

}

void Sensor::run() {
	/*Uncomment if you want to checkt for WHO AM I
		uint8_t data = 0;
		stat = imu.writeRead(LSM9DS1_AG, LSM9DS1_WHO_AM_I,1,data,1);
		if(stat <= 0) i2cerror();
		PRINTF("I am %d! \r\n", data);
	*/
	int16_t xyzGyro[3];
	float xyzGyro_buffer[10][3];
	int16_t xyzMagneto[3];
	//int16_t xyzAccel[3];
	//AT(NOW()+ 2 * SECONDS);
	offsetGyro(&offsets);
	offsetMagneto(&offsets);
	float calibratedMagneto[3];
	float pitch;
	float roll;

	float test=0;
	int counter = 0;
	int buffer_size = 10;
	imu_data data;
	satellite_mode start_mode;
	start_mode.control_mode = control_mode_vel;
	start_mode.mission_mode = mission_mode_standby;
	start_mode.pose_estimation_mode = pose_estimation_mode_imu;
	topic_satellite_mode.publish(start_mode);
	requested_conntrol rcontrol;
	rcontrol.requested_angle = 0;
	rcontrol.requested_rot_speed = 0;
	topic_user_requested_conntrol.publish(rcontrol);
	TIME_LOOP(1 * SECONDS, 10 * MILLISECONDS){
		readGyro(xyzGyro);
		for (size_t i = 0; i < 3; i++)
		{
			xyzGyro_buffer[counter][i] = (xyzGyro[i] * 0.07 - offsets.gyro[i]);;
		}
		
		/*pitch = calcPitch(&sensorData);
		roll = calcRoll(&sensorData);
		calcHeadingMagnetoPitchRoll(&attitude, calibratedMagneto, roll, pitch);*/
		//calcHeadingMagneto(&attitude, calibratedMagneto);

		counter++;
		if(counter >= buffer_size){
			counter = 0;

			readMagneto(xyzMagneto);

			updateDt();

			//calcHeadingGyro(&attitude, xyzGyro[0], &offsets, dt);
			calibrateMagneto(&offsets, xyzMagneto[0], xyzMagneto[1], xyzMagneto[2], calibratedMagneto);
			data.w[0]=0;
			data.w[1]=0;
			data.w[2]=0;
			for (size_t i = 0; i < buffer_size; i++)
			{
				for (size_t j = 0; j < 3; j++)
				{
					data.w[j] += xyzGyro_buffer[i][j];
				}
			}
			
			for (size_t i = 0; i < 3; i++)
			{
				data.w[i] /= buffer_size;
				data.m[i] = calibratedMagneto[i];
			}

			topic_imu_data.publish(data);

			//Kalman
			float r2 = data.w[2];
			float p2 = (data.m[0]);
			float y2 = (data.m[1]);
			float cz = mod(atan2(p2,y2) * 180/M_PI);
			
			
			Matrix_<1,1,float> peter;
			peter.r[0][0]=cz;

			float last_pose = mod(x_hat.r[0][0]);
			
			update(peter,r2);
			test += r2 * dt;
			test = mod(test);
			
			
			position_data pose;
			pose.heading = mod(x_hat.r[0][0]);
			pose.headingMagneto = cz;
			pose.headingGyro = test;
			pose.moving = mod(pose.heading-last_pose)/dt;

			topic_position_data.publish(pose);

			//PRINTF("heading: %f degrees \r\n", attitude.headingMagneto);
			//MW_PRINTF("gx: %f Gauss  gy: %f Gauss  gz: %f Gauss \r\n", (xyzMagneto[0]*0.00014), (xyzMagneto[1]*0.00014), (xyzMagneto[2]*0.00014));
		}
	}
}


//Sensor sensorThread("readIMU");
