#include "rodos.h"
#include "pidControl.hpp"

void calcPIDMotor(controller_errors* errors, control_value* control, additional_sensor_data* data){
    MotorSpeedUpdate(data);  //Get the current motor speed.
    errors->merror = control->desiredMotorSpeed - data->motorSpeed;
    errors->mIerror += errors->mIerror * MOTORCONTROLTIME;
    errors->merror_change = (errors->mLast_error - errors->merror)/MOTORCONTROLTIME;
    errors->mLast_error = errors->merror;

    data->omega_wheel = KP_M * errors->merror + KI_M * errors->mIerror + KD_M * errors->merror_change;

    control->increments += (data->omega_wheel / MAX_RPM) * INCREMENTS;
    
    if(control->increments < 0) control->turnDirection = BACKWARD;
    else control->turnDirection = FORWARD;   
}

void calcPIDPos(float desiredAngle, position_data* pos, additional_sensor_data* data, controller_errors* errors, control_value* control){
    float velocity = 0;
    float dot_omega_wheel = 0;
    float omega_wheel_temp  = 0;
    errors->perror = desiredAngle - pos->heading;
    errors->pIerror += errors->perror * CONTROLTIME;
    errors->perror_change = (errors->perror - errors->pLast_error) / CONTROLTIME;
    errors->pLast_error = errors->perror;

    control->satVelocity = KP_P * errors->perror + KI_P * errors->pIerror + KD_P * errors->pLast_error;
}

void calcPIDVel(control_value* control, additional_sensor_data* data, controller_errors* errors, imu_data* imu){
    float torque = 0;
    float dot_omega_wheel = 0;
    float omega_wheel_temp  = 0;
    errors->verror = control->satVelocity - imu->wy;
    errors->vIerror += errors->verror * CONTROLTIME;
    errors->verror_change = (errors->verror - errors->vLast_error) / CONTROLTIME;
    errors->vLast_error = errors->verror;

    torque = KP_V * errors->verror + KI_V * errors->vIerror + KD_V * errors->vLast_error;

    dot_omega_wheel = - torque/I_WHEEL;
    omega_wheel_temp += dot_omega_wheel * CONTROLTIME;



    if(abs(omega_wheel_temp) > MAX_RAD_PER_SEC){
        if(omega_wheel_temp > MAX_RAD_PER_SEC){
            control->desiredMotorSpeed = MAX_RPM; //Saturate the speed
        }else{
            control->desiredMotorSpeed = -MAX_RPM;
        }
        dot_omega_wheel = 0;  //Stop further acceleration
    }else{
        control->desiredMotorSpeed = (int)floor(omega_wheel_temp * 9.549297); //Update normally if within limits
    }
}