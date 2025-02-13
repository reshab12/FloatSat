#include "sunSensor.hpp"

CommBuffer<additional_sensor_data> cb_sunSensor_additional_sensor_data;
Subscriber sub_sunSensor_additional_sensor_data(topic_additional_sensor_data, cb_sunSensor_additional_sensor_data);

SunSensor::SunSensor(const char* name, int32_t priority):StaticThread(name,priority){}

void SunSensor::init(){
    initializeMotor();
}

void SunSensor::run(){
    float sunArray[1][1];
    requested_conntrol msg;
    msg.requested_rot_speed = 20;
    msg.requested_angle = 0;
    topic_requested_conntrol.publish(msg);
}