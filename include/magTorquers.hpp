#pragma once
#include "main.hpp"

void initializeTorquers();

void driveTorquers(uint16_t value);

class MagTorquer:StaticThread<>{
public:
    MagTorquer(const char* name, int32_t priority);
    void init();

    void run();
};