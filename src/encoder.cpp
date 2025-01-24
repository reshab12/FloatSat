/*****************************************************************
Sensors.cpp

Original Created by: Atheel Redah @ University of Würzburg
Original Creation Date: January 20, 2019

Development environment specifics:
	Software Platform: Rodos (Realtime Onboard Dependable Operating System).
	Hardware Platform: Würzburg Uni Informatik 8 SKITH Board.
*****************************************************************/

#include "encoder.h"
#include "LSM9DS1.h"

static Application module01("LSM9DS1 AHRS", 2001);


//HAL_ADC CurrentADC(ADC01);

//CommBuffer<sTelecommandData> SensorsTelecommandDataBuffer;
//Subscriber SensorsTelecommandDataSubscriber(TelecommandDataTopic, SensorsTelecommandDataBuffer);

/*sSensorData sensorDataLib = {0, 0, 0,	   // int16_t RawDataGx, RawDataGy, RawDataGz
						  0, 0, 0,	   // int16_t RawDataAx, RawDataAy, RawDataAz
						  0, 0, 0,     // int16_t RawDataMx, RawDataMy, RawDataMz
						  0, 0, 0,     // float gx, gy, gz
						  0, 0, 0,     // float ax, ay, az
              0, 0, 0,     // float mx, my, mz
              0,           // float temperature
						  0, 0, 0,     // float pitch, yaw, roll;
						  {1, 0, 0, 0},// float q[4];
						  0, 		   // float motorSpeed;
						  0            // double deltaTime;
};*/

// Create an instance of the LSM9DS1 library called `imu`.
LSM9DS1 imuLib;

HAL_GPIO LSM9DS1_CSAG(GPIO_006); //PA6
HAL_GPIO LSM9DS1_CSM(GPIO_041);  //PC9
HAL_I2C  LSM9DS1_I2C(I2C_IDX1);
//HAL_SPI  LSM9DS1_SPI(SPI_IDX1);
/*
AHRS_Mode AHRSMode = Gyro_Update;
AHRS_Mode AHRSModeLast;

uint64_t SensorsPeriod = 5; // Sensors period in ms

uint64_t lastTime = 0;
uint64_t timeNow = 0;
uint64_t startTime = 0;

uint8_t MagCalState=0;


float abias[3] = {0, 0, 0}, gbias[3] = {0, 0, 0}, magData[3] = {0, 0, 0};

float magMax[3] = {1000, 1000, 1000}, magMin[3] = {-1000, -1000, -1000};


void AHRSUpdate(AHRS_Mode mode)
{
	switch (mode)
	{

	case Gyro_Cal:
		GyroCal(gbias);

		break;

	case Accel_Cal:

		break;

	case Mag_Cal:

		break;

	case Gyro_Update:

		imu.readGyro();    // Read raw gyro data
	  sensorData.gx =round(imu.calcGyro(imu.gx - gbias[0]));   // Convert to degrees per seconds, remove gyro biases
		sensorData.gy =round(imu.calcGyro(imu.gy - gbias[1]));
		sensorData.gz =round(imu.calcGyro(imu.gz - gbias[2]));

		timeNow = NOW();
		sensorData.deltaTime = ((timeNow - lastTime) / (double)SECONDS);
		lastTime = timeNow;

		GyroUpdate(sensorData.gx, sensorData.gy, sensorData.gz);
		break;

	case Gyro_Quaternion_Update:

		break;

	case Acc_Mag_Tilted_Compass_Update:

		break;

	case Madgwick_Quaternion_Update:

		break;

	case Mahony_Quaternion_Update:

		break;
	}
}

// This function will calculate your LSM9DS0' orientation angles based on the gyroscope data.
void GyroUpdate(float gx, float gy, float gz)
{
	sensorData.pitch = sensorData.pitch + gx * sensorData.deltaTime;
	sensorData.roll  = sensorData.roll  + gy * sensorData.deltaTime;
	sensorData.yaw   = sensorData.yaw   + gz * sensorData.deltaTime;
}


void GyroCal(float* gbias)
{
  uint8_t data[6] = {0, 0, 0, 0, 0, 0};
  float gyro_bias[3] = {0, 0, 0};
  float samples= 1000;

  PRINTF("Please make sure that the gyroscope is at standstill, the calibration will take about 5 seconds. \r\n");

  for(int i = 0; i < samples ; i++) {

	imu.readGyro();    // Read raw gyro data
    gyro_bias[0] += imu.gx;
    gyro_bias[1] += imu.gy;
    gyro_bias[2] += imu.gz;
    AT(NOW()+5*MILLISECONDS);
  }

  gbias[0] = round(gyro_bias[0] / samples); // average the data
  gbias[1] = round(gyro_bias[1] / samples);
  gbias[2] = round(gyro_bias[2] / samples);

  PRINTF("gxbias = %f, gybias = %f, gzbias = %f \r\n", gbias[0], gbias[1], gbias[2]);
}
*/

/* Private variables ---------------------------------------------------------*/
__IO uint32_t IC4ReadValue1 = 0, IC4ReadValue2 = 0, Capture = 0;
__IO uint8_t CaptureNumber = 0;
__IO uint32_t TIM2Freq = 0;
__IO uint8_t EncoderB;
__IO double CaptureTime;

void EncoderInit()
{
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  TIM_ICInitTypeDef  TIM_ICInitStructure;

  /* TIM2 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  /* GPIOA clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  /* TIM2 channel 4 pin (PA3) configuration for Encoder A (Yellow)*/
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Connect TIM pins to AF2 */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM2);

  /* Configure (PA5) pin as input floating for Encoder B (White)*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* -----------------------------------------------------------------------
     TIM2 Configuration:

     In this example TIM2 input clock (TIM2CLK) is set to 2 * APB1 clock (PCLK1):
     	 TIM2CLK = SystemCoreClock / 2 = 84000000 Hz

     To get TIM2 counter clock at X Hz, the prescaler is computed as follows:
     	 Prescaler = (TIM3CLK / TIM3 counter clock) - 1
     	 Prescaler = ((SystemCoreClock /2) / X Hz) - 1

     Note:
     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.
    ----------------------------------------------------------------------- */
  //TIM_PrescalerConfig(TIM2, (uint16_t) (((SystemCoreClock/2) / X) - 1), TIM_PSCReloadMode_Immediate);

  /* Enable the TIM2 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* TIM2 configuration: Input Capture mode ---------------------
     The external signal is connected to TIM2 CH4 pin (PA3)
     The Rising edge is used as active edge,
     The TIM2 CCR4 is used to compute the frequency value
  ------------------------------------------------------------ */
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV8;		/*!< Capture performed once every 8 events. */
  TIM_ICInitStructure.TIM_ICFilter = 0x0;

  TIM_ICInit(TIM2, &TIM_ICInitStructure);

  /* TIM enable counter */
  TIM_Cmd(TIM2, ENABLE);

  /* Enable the CC2 Interrupt Request */
  TIM_ITConfig(TIM2, TIM_IT_CC4, ENABLE);
}

extern "C" {
/**
  * @brief  This function handles TIM2 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM2_IRQHandler(void)
{
  if(TIM_GetITStatus(TIM2, TIM_IT_CC4) == SET)
  {
    /* Clear TIM2 Capture compare interrupt pending bit */
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC4);
    CaptureTime = NOW();
    if(CaptureNumber == 0)
    {
      /* Get the Input Capture value */
      IC4ReadValue1 = TIM_GetCapture4(TIM2);
      CaptureNumber = 1;
    }
    else if(CaptureNumber == 1)
    {
      EncoderB = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5);
      /* Get the Input Capture value */
      IC4ReadValue2 = TIM_GetCapture4(TIM2);

      /* Capture computation */
      if (IC4ReadValue2 > IC4ReadValue1)
      {
        Capture = (IC4ReadValue2 - IC4ReadValue1);
      }
      else if (IC4ReadValue2 < IC4ReadValue1)
      {
        Capture = ((0xFFFFFFFF - IC4ReadValue1) + IC4ReadValue2);
      }
      /* Frequency computation */
      TIM2Freq = (uint32_t) ((SystemCoreClock/2)) * 8 / Capture;
      CaptureNumber = 0;
    }
  }
}
}

void MotorSpeedUpdate(motor_data* motor)
{
	double SensorTime = ((NOW()-CaptureTime)/(double)MILLISECONDS);
	if (SensorTime>250) //minimum measured speed is 2 RPS(120 RPM). This can give us 250ms of minimum interval between interrupts (2 interrupts every one revolution).
	{
		TIM2Freq=0;
	}

	if (EncoderB)
	{
		motor->motorSpeed  = -1*((float)TIM2Freq / 16) * 60;  //CCW
	}
	else {motor->motorSpeed  = ((float)TIM2Freq / 16) * 60;}  //CW
}

/*
void ADCUpdate()
{
	float ADCVaule = CurrentADC.read(ADCChannel);

	sensorData.motorCurrent = (ADCVaule / ADCRes) * (3000 / CurrentVoltageRatio) * 1000;

}*/


/*class Sensors: public Thread {

	uint64_t periode;
	sTelecommandData TelecommandDataReceiver;

public:

	Sensors(const char* name, uint64_t periode) : Thread(name) {
		this->periode = periode;
	}

	void init() {

	}

	void run(){

		LSM9DS1_I2C.init(400000);

		CurrentADC.init(ADCChannel);

		EncoderInit();

		if (imu.begin() == false)
			      {
					 PRINTF("Failed to communicate with LSM9DS1.\r\n");
				  }

		GyroCal(gbias);


		TIME_LOOP(0,periode){

	    AHRSUpdate(AHRSMode);

	    MotorSpeedUpdate();

	    ADCUpdate();

        SensorDataTopic.publish(sensorData);

		}
	}
};

Sensors Sensors("Sensors", SensorsPeriod * MILLISECONDS);*/


