#pragma once

#include "main.hpp"

HAL_ADC mainCurrent(ADC_IDX2);
HAL_ADC voltage(ADC_IDX1);

HAL_GPIO safetyPin(GPIO_062);

void initADCPins();

void readADCPins(additional_sensor_data* data);

class ReadADCPins: StaticThread<>{
public:
    ReadADCPins(const char* name, int32_t priority);

    void init();

    void run();
};

