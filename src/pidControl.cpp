#include "rodos.h"
#include "pidControl.hpp"
#include "driveMotor.hpp"

void calcPIDMotor(controller_errors* errors, control_value* control,motor_control_value* motor_control, motor_data* data){
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
    
    if(increments_temp > MOTROINCREMENTS) motor_control->increments = MOTROINCREMENTS;
    else if(increments_temp < -MOTROINCREMENTS) motor_control->increments = MOTROINCREMENTS;
    else{motor_control->increments = abs(increments_temp);}
    //PRINTF("Increments: %d \n", control->increments);

    if(increments_temp < 0) motor_control->turnDirection = BACKWARD;
    else motor_control->turnDirection = FORWARD;   
}

void calcPIDPos(requested_conntrol* request, position_data* pos, controller_errors* errors){
    errors->perror = request->requested_angle - pos->heading;
    errors->pIerror += errors->perror * CONTROLTIME;
    errors->perror_change = (errors->perror - errors->pLast_error) / CONTROLTIME;
    errors->pLast_error = errors->perror;

    request->requested_rot_speed = KP_P * errors->perror + KI_P * errors->pIerror + KD_P * errors->pLast_error;
}

float calcPIDVel(requested_conntrol* request, controller_errors* errors, imu_data* imu){
    float torque;
    errors->verror = request->requested_rot_speed - imu->wy;
    errors->vIerror += errors->verror * CONTROLTIME;
    errors->verror_change = (errors->verror - errors->vLast_error) / CONTROLTIME;
    errors->vLast_error = errors->verror;

    torque = KP_V * errors->verror + KI_V * errors->vIerror + KD_V * errors->vLast_error;

    return torque;
}

void calcVel_with_torque(motor_data* motor_data, float torque, control_value* control){
    float dot_omega_wheel = 0;
    float omega_wheel_temp  = 0;
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

CommBuffer<imu_data> cb_imu_data_VelocityControler_thread;
Subscriber sub_imu_data_VelocityControler_thread(topic_imu_data, cb_imu_data_VelocityControler_thread);

CommBuffer<motor_data> cb_motor_data_VelocityControler_thread;
Subscriber sub_motor_data_VelocityControler_thread(topic_motor_data, cb_motor_data_VelocityControler_thread);

CommBuffer<requested_conntrol> cb_requested_conntrol_VelocityControler_thread;
Subscriber sub_requested_conntrol_VelocityControler_thread(topic_requested_conntrol, cb_requested_conntrol_VelocityControler_thread);

CommBuffer<requested_conntrol> cb_user_requested_conntrol_VelocityControler_thread;
Subscriber sub_user_requested_conntrol_VelocityControler_thread(topic_user_requested_conntrol, cb_user_requested_conntrol_VelocityControler_thread);

CommBuffer<satellite_mode> cb_satellite_mode_VelocityControler;
Subscriber sub_satellite_mode_VelocityControler(topic_satellite_mode, cb_satellite_mode_VelocityControler);

CommBuffer<float> cb_raspberry_control_value_VelocityController;
Subscriber sub_raspberry_control_value_VelocityController(topic_raspberry_control_value,cb_raspberry_control_value_VelocityController);

VelocityControler::VelocityControler(const char* name, int32_t priority):StaticThread(name, priority){}

void VelocityControler::init(){
    initializeMotor();
}

void VelocityControler::run(){
    requested_conntrol requested_conntrol;
    controller_errors errors;
    control_value control;
    imu_data data;
    satellite_mode mode;
    motor_data motor_data;
    float torque;
    TIME_LOOP(0, 5 * MILLISECONDS)
    {
        cb_satellite_mode_VelocityControler.get(mode);
        if( (mode.control_mode==control_mode_pos)||
            (mode.control_mode==control_mode_vel)||
            (mode.control_mode==control_mode_ai_pos)||
            (mode.control_mode==control_mode_ai_vel))
        {
            cb_imu_data_VelocityControler_thread.get(data);
            if(mode.control_mode==control_mode_pos)
                cb_requested_conntrol_VelocityControler_thread.get(requested_conntrol);
            else if(mode.control_mode==control_mode_vel)
                cb_user_requested_conntrol_VelocityControler_thread.get(requested_conntrol);
            
            cb_motor_data_VelocityControler_thread.get(motor_data);
            if((mode.control_mode==control_mode_ai_pos)||(mode.control_mode==control_mode_ai_vel))
                cb_raspberry_control_value_VelocityController.get(torque);
            else
                torque = calcPIDVel(&requested_conntrol, &errors, &data);
            
            calcVel_with_torque(&motor_data, torque, &control);
            topic_control_value.publish(control);
            //PRINTF("Sat Speed: %f", data.wy);
        }
    }
}



CommBuffer<position_data> cb_position_data_PositionControler;
Subscriber sub_position_data_PositionControler(topic_position_data, cb_position_data_PositionControler);

CommBuffer<requested_conntrol> cb_user_requested_conntrol_PositionControler;
Subscriber sub_user_requested_conntrol_PositionControler(topic_user_requested_conntrol, cb_user_requested_conntrol_PositionControler);

CommBuffer<satellite_mode> cb_satellite_mode_PositionControler;
Subscriber sub_satellite_mode_PositionControler(topic_satellite_mode, cb_satellite_mode_PositionControler);


PositionControler::PositionControler(const char* name, int32_t priority):StaticThread(name, priority){}

void PositionControler::init(){
    initializeMotor();
}

void PositionControler::run(){
    controller_errors errors;
    position_data position;
    requested_conntrol requested_conntrol;
    satellite_mode mode;
    TIME_LOOP(0, 5 * MILLISECONDS)
    {
        cb_satellite_mode_PositionControler.get(mode);
        if(mode.control_mode==control_mode_pos){
            cb_position_data_PositionControler.get(position);
            cb_user_requested_conntrol_PositionControler.get(requested_conntrol);
            calcPIDPos(&requested_conntrol, &position, &errors);
            topic_requested_conntrol.publish(requested_conntrol);
        }
    }
}

