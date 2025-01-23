#include "readCurrent.hpp"

HAL_ADC mainCurrent(ADC_IDX0);

void adcUpdate(additional_sensor_data* data){
	uint16_t ADCValue = mainCurrent.read(ADC_CH_004);

	data->mainCurrent = (ADCValue / ADCRes) * (ADCRef / CurrentVoltageRatio) * 1000;
}

class TestADC : StaticThread<>{
public:
    TestADC(const char* name,int32_t priority):StaticThread(name,priority){}

    void init(){}

    void run(){
        additional_sensor_data data;
        TIME_LOOP(0, 100 * MILLISECONDS)
        {
            adcUpdate(&data);
            PRINTF("Current: %f A\n", data.mainCurrent);
        }
    }
};

//TestADC test("TestADC");