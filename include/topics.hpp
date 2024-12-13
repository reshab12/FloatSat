#ifndef __topics_hpp__
#define __topics_hpp__

#include "rodos.h"
/* ~~~~~ Topic definition ~~~~~ */

const uint32_t topic_id_telecommand = 1002;
const uint32_t topic_id_telemetry = 1003;
const uint32_t topic_id_imu_data = 1010;
const uint32_t topic_id_position_data = 1011;
const uint32_t topic_id_control_value = 1012;
const uint32_t topic_id_additional_sensor_data = 1013;
const uint32_t topic_id_satellite_mode = 1014;

// telecommand 
//from Python Middleware to STM32
struct telecommand 
{
  uint8_t command_id;
  uint32_t command_variable;
};

// telemetryframe 
//from STM32 to Python Middleware
struct __attribute__((packed)) telemetry
{
  int64_t number;
};

// starmap 
//from STM to Python Middleware
struct __attribute__((packed)) star_map
{
    /* data */
};

//IMU data 
//from IMU to sensorfusion thread
struct imu_data
{
    float wx,wy,wz,mx,my,mz,ax,ay,az = 0;
};

//angular position data 
//from sensorfusion to controller
struct position_data
{
    float x,y,z = 0;
};

//the value send to the motor-controller
struct control_value
{
    /* data */
};

//
struct additional_sensor_data
{
    /* data */
};

//
struct satellite_mode
{
    uint8_t pose_estimation_mode = 0;
    uint8_t control_mode = 0;
    uint8_t mission_mode = 0;
};

extern Topic<telecommand> topic_telecommand_uplink;
extern Topic<telemetry> topic_telemetry_downlink;
extern Topic<imu_data> topic_imu_data;
extern Topic<position_data> topic_position_data;
extern Topic<control_value> topic_control_value;
extern Topic<additional_sensor_data> topic_additional_sensor_data;
extern Topic<satellite_mode> topic_satellite_mode;


#endif
