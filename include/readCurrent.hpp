#pragma once

#include "main.hpp"

extern HAL_ADC mainCurrent;
extern HAL_ADC voltage;


void initADCPins();

void readADCPins(additional_sensor_data* data);

class ReadADCPins: StaticThread<>{
private:
    HAL_GPIO safetyPin;
public:
    ReadADCPins(const char* name, int32_t priority);

    void init();

    void run();
};

