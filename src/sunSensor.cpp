#include "sunSensor.hpp"

CommBuffer<additional_sensor_data> cb_sunSensor_additional_sensor_data;
Subscriber sub_sunSensor_additional_sensor_data(topic_additional_sensor_data, cb_sunSensor_additional_sensor_data);

CommBuffer<position_data> cb_pose;
Subscriber sub_pose(topic_position_data, cb_pose);

CommBuffer<satellite_mode> cb_satellite_mode_sun_sensor;
Subscriber sub_satellite_mode_sun_sensor(topic_satellite_mode, cb_satellite_mode_sun_sensor);

void readSolarPanel(float voltage){
    uint16_t solarPanel = mainCurrent.read(ADC_CH_002);
    voltage = ((solarPanel / ADCRes) * ADCRef);
}

uint32_t SunSensor::put(const uint32_t topicId, const size_t len, void *data, [[gnu::unused]] const NetMsgInfo &netMsgInfo) {
    sunMsgBuffer.put(*(satellite_mode *) data);
    this->resume();                         // not to publish from interrupt, call a thread to do it
    return 1;
}

SunSensor::SunSensor(const char* name, int32_t priority):StaticThread(name,priority),Subscriber(topic_satellite_mode,name){}

void SunSensor::init(){
    //initializeMotor();
}

void SunSensor::run(){
    suspendCallerUntil();
    satellite_mode mode;
    float sunArray[2] = {0,0};
    float voltage = 0.0;
    requested_conntrol msg;
    additional_sensor_data data;
    position_data pose;
    
    sunMsgBuffer.get(mode);

    if(mode.mission_mode == mission_mode_object_detection){
        for(int i = 0; i < 5000; i++){
            msg.requested_rot_speed = 30;
            msg.requested_angle = 0;
            topic_user_requested_conntrol.publish(msg);
            cb_sunSensor_additional_sensor_data.getOnlyIfNewData(data);
            cb_pose.getOnlyIfNewData(pose);
            readSolarPanel(voltage);
            if(voltage > sunArray[0]){
                sunArray[0] = voltage;
                sunArray[1] = pose.heading;
            } 
            AT(NOW() + 5 * MILLISECONDS);
        }
    }

    sunMsgBuffer.get(mode);
    msg.requested_angle = sunArray[1];
    topic_requested_conntrol.publish(msg);
    mode.control_mode = control_mode_pos;
    mode.mission_mode = mission_mode_standby;
    topic_satellite_mode.publish(mode);
}