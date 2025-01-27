#include "magTorquers.hpp"

HAL_PWM pwmT1(PWM_IDX13);
HAL_PWM pwmT2(PWM_IDX14);

void initializeTorquers(){
    pwmT1.init(MOTORFREQUENCY,MOTROINCREMENTS);
    pwmT2.init(MOTORFREQUENCY,MOTROINCREMENTS);
}

void driveTorquers(uint16_t value){
    pwmT1.write(value);
    pwmT2.write(value);
}

MagTorquer::MagTorquer(const char* name, int32_t priority):StaticThread(name,priority){}

void MagTorquer::init(){
    initializeTorquers();
}

void MagTorquer::run(){
    driveTorquers(5000);
    PRINTF("Torquers are on! \n");
    AT(NOW() + 5 * SECONDS);
    driveTorquers(0);
    PRINTF("They are off!");
}
