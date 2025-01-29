#include "rodos.h"
#include "pidControl.hpp"
#include "driveMotor.hpp"

void calcPIDMotor(controller_errors* errors, control_value* control,motor_control_value* motor_control, motor_data* data, double deltaT){
    int16_t increments_temp;
    float eb = 0;
    errors->merror = control->desiredMotorSpeed - data->motorSpeed;
    //PRINTF("error: %f \n", errors->merror);
    errors->mIerror += errors->merror * deltaT + eb;
    //PRINTF("Integral Error: %f \n", errors->mIerror);
    errors->merror_change = (errors->mLast_error - errors->merror)/ deltaT;
    errors->mLast_error = errors->merror;
    if(errors->mIerror >= MAX_RAD_PER_SEC) eb += 1* (MAX_RAD_PER_SEC - errors->mIerror);

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

void calcPIDPos(requested_conntrol* request, position_data* pos, controller_errors* errors, double deltaT){
    float eb = 0;
    errors->pLast_error = errors->perror;
    errors->perror = request->requested_angle - pos->heading;
    errors->pIerror += errors->perror * deltaT + eb;
    errors->perror_change = (errors->perror - errors->pLast_error) / deltaT;
    if(errors->pIerror >= MAX_RAD_PER_SEC) eb += 1* (MAX_RAD_PER_SEC - errors->pIerror);

    request->requested_rot_speed = KP_P * errors->perror + KI_P * errors->pIerror + KD_P * errors->pLast_error;
}

float calcPIDVel(requested_conntrol* request, controller_errors* errors, position_data* pose,float last_heading, double deltaT){
    float torque = 0;
    float eb = 0;
    errors->vLast_error = errors->verror;
    errors->verror = request->requested_rot_speed - (pose->heading-last_heading) / deltaT;
    errors->vIerror += errors->verror * deltaT + eb;
    if(errors->vIerror >= MAX_RAD_PER_SEC) eb += 1* (MAX_RAD_PER_SEC - errors->vIerror);
    errors->verror_change = (errors->verror - errors->vLast_error) / deltaT;
    
    torque = KP_V * errors->verror + KI_V * errors->vIerror + KD_V * errors->vLast_error;

    return torque;
}

void calcVel_with_torque(motor_data* motor_data, float torque, control_value* control, double deltaT){
    float dot_omega_wheel = 0;
    float omega_wheel_temp  = motor_data->motorSpeed;
    dot_omega_wheel = - torque/I_WHEEL;
    omega_wheel_temp += dot_omega_wheel * deltaT;



    if(abs(omega_wheel_temp) > MAX_RAD_PER_SEC){
        if(omega_wheel_temp > MAX_RAD_PER_SEC){
            control->desiredMotorSpeed = MAX_RPM; //Saturate the speed
        }else{
            control->desiredMotorSpeed = -MAX_RPM;
        }
        dot_omega_wheel = 0;  //Stop further acceleration
    }else if(abs(omega_wheel_temp) < MIN_RAD_PER_SECOND){
        if(omega_wheel_temp > 0){
            control->desiredMotorSpeed = MIN_RPM;
        }else{
            control->desiredMotorSpeed = -MIN_RPM;
        }
    }else{
        control->desiredMotorSpeed = (int)floor(omega_wheel_temp * 9.549297); //Update normally if within limits
    }
}

CommBuffer<imu_data> cb_imu_data_VelocityControler_thread;
Subscriber sub_imu_data_VelocityControler_thread(topic_imu_data, cb_imu_data_VelocityControler_thread);

CommBuffer<position_data> cb_position_data_VelocityControler;
Subscriber sub_position_data_VelocityControler(topic_position_data, cb_position_data_VelocityControler);

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
}

void VelocityControler::run(){
    requested_conntrol requested_conntrol;
    controller_errors errors;
    control_value control;
    imu_data data;
    position_data pose;
    satellite_mode mode;
    motor_data motor_data;
    float torque;
    float last_heading = 0;
    double time = 1.0*NOW()/SECONDS;
    double deltaT;
    TIME_LOOP(0, 25 * MILLISECONDS)
    {
        double tempT = 1.0*NOW()/SECONDS;
        deltaT = tempT - time; 
        time = tempT;
        cb_satellite_mode_VelocityControler.get(mode);
        if( (mode.control_mode==control_mode_pos)||
            (mode.control_mode==control_mode_vel)||
            (mode.control_mode==control_mode_ai_pos)||
            (mode.control_mode==control_mode_ai_vel))
        {
            cb_imu_data_VelocityControler_thread.get(data);
            last_heading = pose.heading;
            cb_position_data_VelocityControler.get(pose);
            if(mode.control_mode==control_mode_pos)
                cb_requested_conntrol_VelocityControler_thread.get(requested_conntrol);
            else if(mode.control_mode==control_mode_vel)
                cb_user_requested_conntrol_VelocityControler_thread.get(requested_conntrol);
            
            cb_motor_data_VelocityControler_thread.get(motor_data);

            if((mode.control_mode==control_mode_ai_pos)||(mode.control_mode==control_mode_ai_vel))
                cb_raspberry_control_value_VelocityController.get(torque);
            else{
                torque = calcPIDVel(&requested_conntrol, &errors, &pose, last_heading, deltaT);
                controller_errors_s vel_errors;
                vel_errors.error = errors.verror;
                vel_errors.error_change = errors.verror_change;
                vel_errors.Ierror = errors.vIerror;
                vel_errors.Last_error = errors.vLast_error;
                topic_vel_errors.publish(vel_errors);
            }
            calcVel_with_torque(&motor_data, torque, &control, deltaT);
            topic_control_value.publish(control);
            //PRINTF("Sat Speed: %f", data.wy);
        }else{
            control.desiredMotorSpeed = 0.0;
            topic_control_value.publish(control);
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
    double deltaT;
    double time = 1.0*NOW()/SECONDS;
    TIME_LOOP(0, 100 * MILLISECONDS)
    {
        double tempT = 1.0*NOW()/SECONDS;
        deltaT = tempT - time; 
        time = tempT;
        cb_satellite_mode_PositionControler.get(mode);
        if(mode.control_mode==control_mode_pos){
            cb_position_data_PositionControler.get(position);
            cb_user_requested_conntrol_PositionControler.get(requested_conntrol);
            calcPIDPos(&requested_conntrol, &position, &errors, deltaT);
            topic_requested_conntrol.publish(requested_conntrol);
        }
    }
}

