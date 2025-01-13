#include "driveMotor.hpp"

HAL_PWM pwm1(PWM_IDX01);
HAL_PWM pwm2(PWM_IDX02);

void initializeMotor(){
    pwm1.init(FREQUENCY, INCREMENTS);
    pwm2.init(FREQUENCY, INCREMENTS);
}

void driveMotor(control_value* control, additional_sensor_data* data){
    switch(control->turnDirection){
        case FORWARD:    
            pwm1.write(control->increments);
            pwm2.write(0);
            break;
        case BACKWARD:
            pwm1.write(0);
            pwm2.write(control->increments);
            break;
    }
}