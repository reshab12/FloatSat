#include "rodos.h"
#include "pidPos.hpp"

#include "hal_pwm.h"

HAL_PWM pwmMotor(PWM_IDX00);

void initialize(){
    pwmMotor.init(FREQUENCY,INCREMENTS);
}

class PWM : StaticThread<>{
public:
    PWM(const char* name):StaticThread(name){};

    void init(){
        initialize();
    }

    void run(){
        pwmMotor.write(3000);
        AT(NOW() + 5 * SECONDS);
        pwmMotor.write(3500);
        AT(NOW() + 3 * SECONDS);
        pwmMotor.write(3000);
        AT(NOW() + 2 * SECONDS);
        pwmMotor.write(2500);
        AT(NOW() + 3 * SECONDS);
        pwmMotor.write(3000);
    }
};

PWM pwm("pwm");

/*
float calcPID(float desiredAngle, float currentAngle, float omega_wheel){
    float error = 0;
    float int_error = 0;
    float error_change = 0;
    float last_error = 0;
    int torque = 0;
    float dot_omega_wheel = 0;
    error = desiredAngle - currentAngle;
    int_error += error * CONTROLETIME;
    error_change = (error - last_error) / CONTROLETIME;
    last_error = error;
    pwmMotor.init(FREQUENCY, INCREMENTS);
    pwmMotor.write(20);
    torque = KP * error + KI * int_error + KD * error_change;

    dot_omega_wheel = - torque/I_WHEEL;
    omega_wheel += dot_omega_wheel * CONTROLETIME;

    return omega_wheel;
    //dot_omega_sat = -dot_omega_wheel * I_WHEEL / I_SATELLITE; 
    //omega_sat += dot_omega_sat * dt;
    //angles += omega_sat * dt;

}

*/
