#include "rodos.h"
#include "pidControl.hpp"
#include "driveMotor.hpp"

void calcPIDMotor(controller_errors* errors, control_value* control,motor_control_value* motor_control, motor_data* data, double deltaT){
    int16_t increments_temp;
    if(control->desiredMotorSpeed <= MIN_RPM && control->desiredMotorSpeed >= -MIN_RPM){
        motor_control->turnDirection = BREAK;
        motor_control->increments = 0;
        errors->mIerror = 0;
        errors->meb = 0;
    }else{
        errors->merror = control->desiredMotorSpeed - data->motorSpeed;
        errors->mIerror += errors->merror * deltaT + errors->meb;
        errors->merror_change = (errors->merror - errors->mLast_error)/ deltaT;
        errors->mLast_error = errors->merror;
        if((errors->mIerror >= MAX_RPM)){ 
            errors->meb = 1* (MAX_RPM - errors->mIerror);
        }else if((errors->mIerror <= -MAX_RPM)){
            errors->meb = 1* (-MAX_RPM - errors->mIerror);
        }else
            errors->meb = 0;

        //data->omega_wheel = KP_M * errors->merror + KI_M * errors->mIerror + KD_M/10 * errors->merror_change + data->omega_wheel;
        data->omega_wheel = KP_M * errors->merror + KI_M * errors->mIerror + KD_M * errors->merror_change ;

        increments_temp = (data->omega_wheel / MAX_RPM) * MOTROINCREMENTS;
        if(increments_temp > MOTROINCREMENTS) 
            motor_control->increments = MOTROINCREMENTS;
        else if(increments_temp < -MOTROINCREMENTS) 
            motor_control->increments = MOTROINCREMENTS;
        else{
            motor_control->increments = abs(increments_temp);
        }
        if(data->omega_wheel < 0){
            motor_control->turnDirection = BACKWARD;
        }else{
            motor_control->turnDirection = FORWARD;   
        }
        if((data->omega_wheel < 0 && data->motorSpeed > MIN_RPM) || (data->omega_wheel > 0 && data->motorSpeed < -MIN_RPM)){
            motor_control->turnDirection = BREAK;
        }
    }
}

void calcPIDPos(requested_conntrol* request, position_data* pos, controller_errors* errors, double deltaT){
    errors->perror = mod(request->requested_angle - pos->heading);
    errors->pIerror += errors->perror * deltaT + errors->peb;
    errors->perror_change = (errors->perror - errors->pLast_error) / deltaT;
    errors->pLast_error = errors->perror;
    if((errors->pIerror >= max_sat_dps/KI_P)){ 
        errors->peb = 1* (max_sat_dps/KI_P - errors->pIerror);
    }else if(errors->pIerror <= -max_sat_dps/KI_P){
        errors->peb = 1* (-max_sat_dps/KI_P - errors->pIerror);
    }else
        errors->peb = 0;

    if((errors->perror < 1 && errors->perror > -1) && (errors->vIerror > 500 || errors->vIerror < -500)){
        errors->pIerror = 0;
    }

    request->requested_rot_speed = KP_P * errors->perror + KI_P * errors->pIerror + KD_P * errors->pLast_error;
}

float calcPIDVel(requested_conntrol* request, controller_errors* errors, position_data* pose,float last_heading, double deltaT){
    float torque = 0;
    if(request->requested_rot_speed >= 0)
        request->requested_rot_speed = min(request->requested_rot_speed,max_sat_dps);
    else
        request->requested_rot_speed = max(request->requested_rot_speed,-max_sat_dps);
    
    errors->verror = request->requested_rot_speed - pose->moving;
    
    errors->vIerror += errors->verror * deltaT + errors->veb;
    if((errors->vIerror >= max_dot_omega_wheel*I_WHEEL))
        errors->veb = 1* (max_dot_omega_wheel*I_WHEEL - errors->vIerror);
    else if(errors->vIerror <= -max_dot_omega_wheel*I_WHEEL)
        errors->veb = 1* (-max_dot_omega_wheel*I_WHEEL - errors->vIerror);
    else
        errors->veb = 0;
    if((errors->verror < 1 && errors->verror > -1) && (errors->vIerror > 50 || errors->vIerror < -50)){
        errors->vIerror = 0;
    }

    errors->verror_change = (errors->verror - errors->vLast_error) / deltaT;
    errors->vLast_error = errors->verror;

    torque = (KP_V * errors->verror + KI_V * errors->vIerror + KD_V * errors->vLast_error);

    return torque;
}

void calcVel_with_torque(motor_data* motor_data, float torque, control_value* control, double deltaT){
    
    float omega_wheel_temp = motor_data->motorSpeed*rpm2radps;

    if(( motor_data->motorSpeed < MIN_RPM) && (motor_data->motorSpeed > -MIN_RPM)){
        omega_wheel_temp = control->desiredMotorSpeed*rpm2radps;
    }

    float dot_omega_wheel = torque/I_WHEEL;

    //MW_PRINTF("%f, %f, %f\n",omega_wheel_temp,dot_omega_wheel, dot_omega_wheel*deltaT);

    if(dot_omega_wheel >= 0)
        dot_omega_wheel = min(dot_omega_wheel,max_dot_omega_wheel);
    else
        dot_omega_wheel = max(dot_omega_wheel,-max_dot_omega_wheel);
    omega_wheel_temp += dot_omega_wheel * deltaT;

    if(omega_wheel_temp > max_rad_ps_contr){
        control->desiredMotorSpeed = max_rpm_contr; //Saturate the speed
    }else if(omega_wheel_temp < -max_rad_ps_contr){
        control->desiredMotorSpeed = -max_rpm_contr;
    }else
        control->desiredMotorSpeed = omega_wheel_temp / rpm2radps; //Update normally if within limits
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
    int64_t time = NOW();
    double deltaT;
    controller_errors_s vel_errors;
    TIME_LOOP(0, 25 * MILLISECONDS)
    {
        int64_t tempT = NOW();
        deltaT = (tempT - time)*1.0/SECONDS; 
        time = tempT;
        cb_satellite_mode_VelocityControler.get(mode);
        if( (mode.control_mode==control_mode_pos)||
            (mode.control_mode==control_mode_vel)||
            (mode.control_mode==control_mode_ai_pos))
        {
            cb_imu_data_VelocityControler_thread.get(data);
            last_heading = pose.heading;
            cb_position_data_VelocityControler.get(pose);
            if(mode.control_mode==control_mode_pos)
                cb_requested_conntrol_VelocityControler_thread.get(requested_conntrol);
            else if(mode.control_mode==control_mode_vel)
                cb_user_requested_conntrol_VelocityControler_thread.get(requested_conntrol);
            
            cb_motor_data_VelocityControler_thread.get(motor_data);

            if((mode.control_mode==control_mode_ai_pos)){
                cb_raspberry_control_value_VelocityController.get(torque);
                calcVel_with_torque(&motor_data, torque, &control, deltaT);
            }else{
                torque = calcPIDVel(&requested_conntrol, &errors, &pose, last_heading, deltaT);
                vel_errors.error = errors.verror;
                vel_errors.error_change = errors.verror_change;
                vel_errors.Ierror = errors.vIerror;
                //vel_errors.Last_error = requested_conntrol.requested_rot_speed;
                topic_vel_errors.publish(vel_errors);
                calcVel_with_torque(&motor_data, torque, &control, deltaT);
                topic_raspberry_control_value.publish(torque);
            }
            topic_control_value.publish(control);
        }else if(mode.control_mode == control_mode_stop_wheel){
            control.desiredMotorSpeed = 0.0;
            topic_control_value.publish(control);
        }
    }
}



CommBuffer<position_data> cb_position_data_PositionControler;
Subscriber sub_position_data_PositionControler(topic_position_data, cb_position_data_PositionControler);

CommBuffer<requested_conntrol> cb_user_requested_conntrol_PositionControler;
Subscriber sub_user_requested_conntrol_PositionControler(topic_user_requested_conntrol, cb_user_requested_conntrol_PositionControler);

CommBuffer<requested_conntrol> cb_requested_conntrol_PositionControler;
Subscriber sub_requested_conntrol_PositionControler(topic_requested_conntrol, cb_requested_conntrol_PositionControler);

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
    controller_errors_s pos_errors;
    double deltaT;
    double time = 1.0*NOW()/SECONDS;
    TIME_LOOP(0, 100 * MILLISECONDS)
    {
        double tempT = 1.0*NOW()/SECONDS;
        deltaT = tempT - time; 
        time = tempT;
        cb_satellite_mode_PositionControler.getOnlyIfNewData(mode);
        if(mode.control_mode==control_mode_pos){
            cb_position_data_PositionControler.getOnlyIfNewData(position);
            /*if(mode.mission_mode == mission_mode_star_mapper)
                cb_requested_conntrol_PositionControler.getOnlyIfNewData(requested_conntrol);
            else*/
            cb_user_requested_conntrol_PositionControler.getOnlyIfNewData(requested_conntrol);
            calcPIDPos(&requested_conntrol, &position, &errors, deltaT);
            topic_requested_conntrol.publish(requested_conntrol);
            pos_errors.error = errors.perror;
            pos_errors.error_change = errors.perror_change;
            pos_errors.Ierror = errors.pIerror;
            //pos_errors.Last_error = requested_conntrol.requested_rot_speed;
            topic_pos_errors.publish(pos_errors);
        }
    }
}

