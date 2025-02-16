#pragma once

#include "main.hpp"

extern HAL_ADC mainCurrent;
extern HAL_ADC voltage;


void initADCPins();

void readADCPins(additional_sensor_data* data);

class ReadADCPins:public Subscriber,public StaticThread<>{
private:
    HAL_GPIO safetyPin;
    CommBuffer<bool> safetyPinMsgBuffer;
public:
    ReadADCPins(const char* name, int32_t priority);

    void init();

    void run();

    uint32_t put(const uint32_t topicId, const size_t len, void *data, [[gnu::unused]] const NetMsgInfo &netMsgInfo);
};



