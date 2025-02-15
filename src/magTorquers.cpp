#include "magTorquers.hpp"

HAL_PWM pwmT1(PWM_IDX13);
HAL_PWM pwmT2(PWM_IDX14);

void initializeTorquers(){
    pwmT1.init(MOTORFREQUENCY,MOTROINCREMENTS);
    pwmT2.init(MOTORFREQUENCY,MOTROINCREMENTS);
}

void driveTorquers1(uint16_t value){
    pwmT1.write(value);
    pwmT2.write(0);
}

void driveTorquers2(uint16_t value){
    pwmT1.write(0);
    pwmT2.write(value);
}

MagTorquer::MagTorquer(const char* name, int32_t priority):StaticThread(name,priority),Subscriber(topic_satellite_mode,name){}

void MagTorquer::init(){
    initializeTorquers();
    initializeMotor();
}

CommBuffer<satellite_mode> cb_satellite_mode_torquers;
Subscriber sub_satellite_mode_torquers(topic_satellite_mode, cb_satellite_mode_torquers);

CommBuffer<motor_data> cb_motor_data_torquers;
Subscriber sub_motor_data_torquer(topic_motor_data, cb_motor_data_torquers);

CommBuffer<position_data> cb_position_data_torquer;
Subscriber sub_position_data_torquer(topic_position_data, cb_position_data_torquer);


void MagTorquer::run(){
    satellite_mode mode;
    requested_conntrol pose;
    position_data position;
    motor_data motor;
    while (true)
    {
        int64_t start_time = NOW();
        inputMsgBuffer.getOnlyIfNewData(mode);
        if(mode.mission_mode == mission_mode_mag_torquers){
            cb_motor_data_torquers.getOnlyIfNewData(motor);
            cb_position_data_torquer.getOnlyIfNewData(position);
            if(motor.motorSpeed > 1000){
                pose.requested_angle = -90 + 45;
                topic_user_requested_conntrol.publish(pose);
                if(position.heading > 0){
                    driveTorquers1(5000);
                }else{
                    driveTorquers2(5000);
                }
            }else if(motor.motorSpeed < -1000){
                pose.requested_angle = 90 + 45;
                topic_user_requested_conntrol.publish(pose);
                if(position.heading < 0){
                    driveTorquers1(5000);
                }else{
                    driveTorquers2(5000);
                }
            }else{
                pose.requested_angle = 45;
                topic_user_requested_conntrol.publish(pose);
                driveTorquers(0);

                inputMsgBuffer.getOnlyIfNewData(mode);
                mode.mission_mode = mission_mode_standby;
                topic_satellite_mode.publish(mode);
            }   
            AT(start_time + 100 * MILLISECONDS);      
        }else{
            driveTorquers(0);
            suspendCallerUntil();
        }
    }
}


uint32_t MagTorquer::put(const uint32_t topicId, const size_t len, void *data, [[gnu::unused]] const NetMsgInfo &netMsgInfo) {
    inputMsgBuffer.put(*(satellite_mode *) data);
    this->resume();                         // not to publish from interrupt, call a thread to do it
    return 1;
}


