#pragma once

#include "main.hpp"


#define MOTROINCREMENTS 5000
#define MOTORFREQUENCY 4800
#define MOTORCONTROLTIME 5 * MILLISECONDS

void initializeMotor();

void driveMotor(control_value* control);

class MotorControler : StaticThread<>{
public: 
    MotorControler(const char* name);

    void init();

    void run();
};