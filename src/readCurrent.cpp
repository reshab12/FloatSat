#include "readCurrent.hpp"

HAL_ADC mainCurrent(ADC_IDX2);
HAL_ADC voltage(ADC_IDX1);

HAL_GPIO safetyPin(GPIO_062);

void initADCPins(){
    mainCurrent.init(ADC_CH_000);
    //mainCurrent.init(ADC_CH_004);
    mainCurrent.init(ADC_CH_010);
    mainCurrent.init(ADC_CH_002);
    voltage.init(ADC_CH_012);
}

void readADCPins(additional_sensor_data* data){
    uint16_t voltageADC = voltage.read(ADC_CH_012);
    data->boardVoltage = (voltageADC/ADCRes) * ADCRef;
    data->batterieVoltage = data->boardVoltage/0.105;
    AT(NOW() + 10 * MILLISECONDS);

    uint16_t motorADCValue = mainCurrent.read(ADC_CH_000);
	data->motorCurrent = (((motorADCValue / ADCRes) * ADCRef))/ NewCurrentVoltage;
    AT(NOW() + 10*MILLISECONDS);

    //uint16_t magADCValue = mainCurrent.read(ADC_CH_004);
	//data->magTorquerCurrent = ((magADCValue / ADCRes) * ADCRef -2.5)/ CurrentVoltageRatio;
    //AT(NOW() + 10*MILLISECONDS);

    uint16_t boardADCValue = mainCurrent.read(ADC_CH_010);
	data->boardCurrent = ((boardADCValue / ADCRes) * ADCRef -2.5) /CurrentVoltageRatio;
    AT(NOW() + 10 * MILLISECONDS);

    uint16_t solarPanel = mainCurrent.read(ADC_CH_002);
    //data->solarPanel = ((solarPanel / ADCRes) * ADCRef -2.5) /CurrentVoltageRatio;
}

ReadADCPins::ReadADCPins(const char* name, int32_t priority):StaticThread(name, priority){}

void ReadADCPins::init(){
    initializeMotor();
    safetyPin.init(true, 1,0);
    initADCPins();
}

void ReadADCPins::run(){
    safetyPin.setPins(0);
    motor_control_value motor;
    additional_sensor_data data;
    TIME_LOOP(0, 500 * MILLISECONDS){
        readADCPins(&data);
        topic_additional_sensor_data.publish(data);
        //if(data.batterieVoltage < 11.0) safetyPin.setPins(1);
        //PRINTF("Motor Current: %f A\n Board Current: %f A\n Torquer Current: %f A\n Board Voltage: %f V\n Batterie Voltage: %f V\n", data.motorCurrent, data.boardCurrent, data.magTorquerCurrent, data.boardVoltage, data.batterieVoltage);
    }
}
