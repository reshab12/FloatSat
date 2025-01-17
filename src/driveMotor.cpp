#include "driveMotor.hpp"

HAL_PWM pwm1(PWM_IDX01);
HAL_PWM pwm2(PWM_IDX02);

void initializeMotor(){
    pwm1.init(MOTORFREQUENCY, MOTROINCREMENTS);
    pwm2.init(MOTORFREQUENCY, MOTROINCREMENTS);
}

void driveMotor(control_value* control){
    switch(control->turnDirection){
        case BACKWARD:    
            pwm1.write(control->increments);
            pwm2.write(0);
            break;
        case FORWARD:
            pwm1.write(0);
            pwm2.write(control->increments);
            break;
    }
}

CommBuffer<control_value> cb_control_value_motor_thread;
Subscriber sub_topic_control_value_motor_thread(topic_control_value, cb_control_value_motor_thread);


MotorControler::MotorControler(const char* name):StaticThread(name){}

void MotorControler::init(){
    EncoderInit();
    initializeMotor();
}

void MotorControler::run(){
    motor_data motor;
    controller_errors errors;
    control_value control;
    TIME_LOOP(0, 5 * MILLISECONDS)
    {
        cb_control_value_motor_thread.get(control);
        MotorSpeedUpdate(&motor);
        topic_motor_data.publish(motor);
        PRINTF("MotorSpeed: %d \n", motor.motorSpeed);
        calcPIDMotor(&errors, &control, &motor);
        driveMotor(&control);
    }
}