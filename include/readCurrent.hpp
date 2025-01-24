#pragma once

#include "main.hpp"

//void readMotorCurrent(additional_sensor_data* data);

//void readMagTorquerCurrent(additional_sensor_data* data);

//void readBoardCurrent(additional_sensor_data* data);

//void readVoltage(additional_sensor_data* data);

void initADCPins();

void readADCPins(additional_sensor_data* data);

class ReadADCPins: StaticThread<>{
public:
    ReadADCPins(const char* name, int32_t priority);

    void init();

    void run();
};

