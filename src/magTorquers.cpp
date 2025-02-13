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

/*void desaturate(motor_data* motor, controller_errors* errors, control_value* control, motor_control_value* motor_control, double deltaT){
    motor->motorSpeed = 1000;
    driveTorquers(5000);
    while(abs(motor->motorSpeed)-3000>100){
        MotorSpeedUpdate(motor);
        calcPIDMotor(errors, control, motor_control, motor, deltaT);
    }
}*/

MagTorquer::MagTorquer(const char* name, int32_t priority):StaticThread(name,priority){}

void MagTorquer::init(){
    initializeTorquers();
    initializeMotor();
}

void MagTorquer::run(){
    control_value motor;
    /*while(1){

        AT(END_OF_TIME);
        motor.desiredMotorSpeed = 1000;
        driveTorquers(5000);
        topic_control_value.publish(motor);
    }*/
    AT(20*SECONDS);
    driveTorquers(5000);
    //PRINTF("Torquers are on! \n");
    AT(NOW() + 8 * SECONDS);
    driveTorquers(0);
    //PRINTF("They are off!");
}
