#include "magTorquers.hpp"

HAL_PWM pwmT1(PWM_IDX13);
HAL_PWM pwmT2(PWM_IDX14);

void initializeTorquers(){
    pwmT1.init(FREQUENCY,INCREMENTS);
    pwmT2.init(FREQUENCY,INCREMENTS);
}

void driveTorquers(){
    pwmT1.write(INCREMENTS);
    pwmT2.write(INCREMENTS);
}