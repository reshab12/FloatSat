#include "driveMotor.hpp"

HAL_PWM pwm1(PWM_IDX01);
HAL_PWM pwm2(PWM_IDX02);

void initializeMotor(){
    pwm1.init(MOTORFREQUENCY, MOTROINCREMENTS);
    pwm2.init(MOTORFREQUENCY, MOTROINCREMENTS);
}

void driveMotor(motor_control_value* control){
    switch(control->turnDirection){
        case FORWARD:
            pwm1.write(0);
            pwm2.write(control->increments);
            break;
        case BACKWARD:
            pwm1.write(control->increments);
            pwm2.write(0);
            break;
        case BREAK:
            pwm1.write(5000);
            pwm2.write(5000);
    }
}

CommBuffer<control_value> cb_control_value_motor_thread;
Subscriber sub_topic_control_value_motor_thread(topic_control_value, cb_control_value_motor_thread);


MotorControler::MotorControler(const char* name,int32_t priority):StaticThread(name, priority){}

void MotorControler::init(){
    EncoderInit();
    initializeMotor();
}

void MotorControler::run(){
    motor_data motor;
    controller_errors errors;
    control_value control;
    motor_control_value motor_control;
    controller_errors_s mot_errors;
    double deltaT;
    double time = 1.0*NOW()/SECONDS;
    //control.desiredMotorSpeed = 3000;
    TIME_LOOP(0, 5 * MILLISECONDS)
    {
        double tempT = 1.0*NOW()/SECONDS;
        deltaT = tempT - time; 
        time = tempT;
        cb_control_value_motor_thread.getOnlyIfNewData(control);
        MotorSpeedUpdate(&motor);
        calcPIDMotor(&errors, &control,&motor_control, &motor, deltaT);
        mot_errors.error = errors.merror;
        mot_errors.error_change = errors.merror_change;
        mot_errors.Ierror = errors.mIerror;
        mot_errors.Last_error = errors.mLast_error;
        topic_mot_errors.publish(mot_errors);
        topic_motor_data.publish(motor);
        topic_motor_control_value.publish(motor_control);
        driveMotor(&motor_control);
    }
}