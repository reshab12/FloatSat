#include "rodos.h"
#include "pidControl.hpp"

void calcPIDMotor(int16_t desiredVelocity, controller_errors* errors, control_value* control, additional_sensor_data* data){
    MotorSpeedUpdate(&data);  //Get the current motor speed.
    errors->merror = desiredVelocity - data->motorSpeed;
    errors->mIerror += errors->mIerror * MOTORCONTROLTIME;
    errors->merror_change = (errors->mLast_error - errors->merror)/MOTORCONTROLTIME;
    errors->mLast_error = errors->merror;

    data->omega_wheel = KP_M * errors->merror + KI_M * errors->mIerror + KD_M * errors->merror_change;

    control->increments += (omega_wheel / MAX_RPM) * INCREMENTS;   
}

void calcPIDPos(float desiredAngle, imu_data* imu, position_data* pos, additional_sensor_data* data, controller_errors* errors){
    int torque = 0;
    float dot_omega_wheel = 0;
    float omega_wheel_temp  = 0;
    errors->perror = desiredAngle - pos->heading;
    errors->pIerror += errors->perror * CONTROLTIME;
    errors->perror_change = (errors->perror - errors->pLast_error) / CONTROLTIME;
    errors->pLast_error = errors->perror;

    torque = KP_P * errors->perror + KI_P * errors->pIerror + KD_P * errors->pLast_error;

    dot_omega_wheel = - torque/I_WHEEL;
    omega_wheel_temp += dot_omega_wheel * CONTROLTIME;

    if(abs(omega_wheel_temp) > MAX_RAD_PER_SEC){
        if(omega_wheel_temp > MAX_RAD_PER_SEC){
            data->motorSpeed = MAX_RAD_PER_SEC; //Saturate the speed
        }else{
            data->motorSpeed = -MAX_RAD_PER_SEC;
        }
        dot_omega_wheel = 0;  //Stop further acceleration
    }else{
        data->motorSpeed = omega_wheel_temp; //Update normally if within limits
    }
}

void calcPIDVel(float desiredSpeed, imu_data* imu, additional_sensor_data* data, controller_errors* errors){
    int torque = 0;
    float dot_omega_wheel = 0;
    float omega_wheel_temp  = 0;
    errors->verror = desiredSpeed - data->motorSpeed;
    errors->vIerror += errors->verror * CONTROLTIME;
    errors->verror_change = (errors->verror - errors->vLast_error) / CONTROLTIME;
    errors->vLast_error = errors->verror;

    torque = KP_V * errors->verror + KI_V * errors->vIerror + KD_V * errors->vLast_error;

    dot_omega_wheel = - torque/I_WHEEL;
    omega_wheel_temp += dot_omega_wheel * CONTROLTIME;

    if(abs(omega_wheel_temp) > MAX_RAD_PER_SEC){
        if(omega_wheel_temp > MAX_RAD_PER_SEC){
            data->motorSpeed = MAX_RAD_PER_SEC; //Saturate the speed
        }else{
            data->motorSpeed = -MAX_RAD_PER_SEC;
        }
        dot_omega_wheel = 0;  //Stop further acceleration
    }else{
        data->motorSpeed = omega_wheel_temp; //Update normally if within limits
    }
}

/*
class PIDPos : StaticThread<>{
public:
    PIDPos(const char* name):StaticThread(name){};

    void init(){
        initialize();
    }

    void run(){
        int i = 3;
        TIME_LOOP(2 * SECONDS, 5 * MILLISECONDS){
            i++;
            if(i == 4){
                speed = calcPIDPos();
                i = 0;
            }
            calcPIDMotor(speed);
        }
    }
};*/
//PIDPos pidPos("pidPos");