#include "readCurrent.hpp"

HAL_ADC mainCurrent(ADC_IDX0);

void adcUpdate(additional_sensor_data* data){
	uint16_t ADCValue = mainCurrent.read(ADC_CH_000);

	data->mainCurrent = (ADCValue / ADCRes) * (ADCRef / CurrentVoltageRatio) * 1000;
}

class TestADC : StaticThread<>{
public:
    TestADC(const char* name):StaticThread(name){}

    void init(){}

    void run(){
        additional_sensor_data data;
        adcUpdate(&data);
        PRINTF("Current: %f A", data.mainCurrent);
    }
};

TestADC test("TestADC");