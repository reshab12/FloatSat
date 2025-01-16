#include "rodos.h"
#include "pidControl.hpp"
#include "driveMotor.hpp"

void calcPIDMotor(controller_errors* errors, control_value* control, additional_sensor_data* data){
    int16_t increments_temp;
    errors->merror = control->desiredMotorSpeed - data->motorSpeed;
    //PRINTF("error: %f \n", errors->merror);
    errors->mIerror += errors->merror * 0.005;
    //PRINTF("Integral Error: %f \n", errors->mIerror);
    errors->merror_change = (errors->mLast_error - errors->merror)/ 0.005;
    errors->mLast_error = errors->merror;

    data->omega_wheel = KP_M * errors->merror + KI_M * errors->mIerror + KD_M * errors->merror_change;
    //PRINTF("omega_wheel: %f \n", data->omega_wheel);
    increments_temp = (data->omega_wheel / MAX_RPM) * MOTROINCREMENTS;
    
    if(increments_temp > MOTROINCREMENTS) control->increments = MOTROINCREMENTS;
    else if(increments_temp < -MOTROINCREMENTS) control->increments = MOTROINCREMENTS;
    else{control->increments = increments_temp;}
    PRINTF("Increments: %d \n", control->increments);

    if(increments_temp < 0) control->turnDirection = BACKWARD;
    else control->turnDirection = FORWARD;   
}

void calcPIDPos(requested_conntrol* request, position_data* pos, additional_sensor_data* data, controller_errors* errors, control_value* control){
    errors->perror = request->requested_angle - pos->heading;
    errors->pIerror += errors->perror * CONTROLTIME;
    errors->perror_change = (errors->perror - errors->pLast_error) / CONTROLTIME;
    errors->pLast_error = errors->perror;

    request->requested_rot_speed = KP_P * errors->perror + KI_P * errors->pIerror + KD_P * errors->pLast_error;
}

void calcPIDVel(requested_conntrol* request, additional_sensor_data* data, controller_errors* errors, imu_data* imu, control_value* control){
    float torque = 0;
    float dot_omega_wheel = 0;
    float omega_wheel_temp  = 0;
    errors->verror = request->requested_rot_speed - imu->wy;
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