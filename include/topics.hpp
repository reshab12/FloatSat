#pragma once

#include "rodos.h"
/* ~~~~~ Topic definition ~~~~~ */

const uint32_t topic_id_telecommand = 1002;
const uint32_t topic_id_telemetry = 1003;
const uint32_t topic_id_failed_command = 1004;
const uint32_t topic_id_imu_data = 1010;
const uint32_t topic_id_position_data = 1011;
const uint32_t topic_id_control_value = 1012;
const uint32_t topic_id_additional_sensor_data = 1013;
const uint32_t topic_id_satellite_mode = 1014;
const uint32_t topic_id_start_mission = 1015;

// telecommand 
//from Python Middleware to STM32
struct telecommand 
{
  uint8_t command_id;
  uint32_t command_variable;
};

struct failed_telecommand 
{
  bool failed;
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
    float pitch, roll, yaw;
    float headingMagneto = 0;
	float headingGyro = 0;
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

// telemetryframe 
//from STM32 to Python Middleware
/*struct __attribute__((packed)) telemetry
{
    int64_t number; //l

   //satellite_mode
    uint8_t pose_estimation_mode = 0;   //B
    uint8_t control_mode = 0;
    uint8_t mission_mode = 0;
      

    //imu_data
    float wx,wy,wz = 0.0f; //f

};*/
//struct __attribute__((packed)) telemetry
struct telemetry
{
    int64_t time; //l

   //satellite_mode
    satellite_mode satellite_modes; // 3B
      
    //imu_data
    imu_data imu;  //9f

    //position_data
    position_data position; //5f
};

struct start_mission
{
    uint8_t mission;
};

extern Topic<telecommand> topic_telecommand_uplink;
extern Topic<telemetry> topic_telemetry_downlink;
extern Topic<failed_telecommand> topic_failed_telecommand_downlink;
extern Topic<imu_data> topic_imu_data;
extern Topic<position_data> topic_position_data;
extern Topic<control_value> topic_control_value;
extern Topic<additional_sensor_data> topic_additional_sensor_data;
extern Topic<satellite_mode> topic_satellite_mode;
extern Topic<start_mission> topic_start_mission;

