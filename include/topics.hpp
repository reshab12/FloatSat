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
const uint32_t topic_id_variables = 1015;

// telecommand 
//from Python Middleware to STM32
struct telecommand 
{
  uint8_t command_id;
  uint32_t command_variable;
};

enum direction
{
    FORWARD,
    BACKWARD
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
    float wx,wy,wz,mx,my,mz,ax,ay,az = 0;   //W is in dps
};

//angular position data 
//from sensorfusion to controller
struct position_data
{
    float headingMagneto = 0;   //-180 to 180 degrees
	float headingGyro = 0;      //-180 to 180 degrees
    float heading = 0;          //-180 to 180 degrees
    float pitch, roll, yaw;     // in radian
};

//the value send to the motor-controller
struct control_value
{
    uint16_t increments;            //increments set for the motor PWM
    direction turnDirection;        //CW or CCW
    float satVelocity;              //Desired Velocity from the position controler
    uint16_t desiredMotorSpeed;     //Desired motor RPM from the velocity controler
};

//
struct additional_sensor_data
{
    uint16_t motorSpeed;    //RPM of the motor
    float omega_wheel;      //From motor PID calulated change to omega wheel.
};

//
struct satellite_mode
{
    uint8_t pose_estimation_mode = 0;
    uint8_t control_mode = 0;
    uint8_t mission_mode = 0;
};

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

struct variables
{
    uint32_t requested_angle;
    uint32_t requested_rot_speed;
};

struct controller_errors
{
    float merror, mIerror, merror_change, mLast_error = 0;  //errors for the motor controler
    float verror, vIerror, verror_change, vLast_error = 0;  //errors for the velocity controler
    float perror, pIerror, perror_change, pLast_error = 0;  //errors for the position controler
};


extern Topic<telecommand> topic_telecommand_uplink;
extern Topic<telemetry> topic_telemetry_downlink;
extern Topic<failed_telecommand> topic_failed_telecommand_downlink;
extern Topic<imu_data> topic_imu_data;
extern Topic<position_data> topic_position_data;
extern Topic<control_value> topic_control_value;
extern Topic<additional_sensor_data> topic_additional_sensor_data;
extern Topic<satellite_mode> topic_satellite_mode;
extern Topic<variables> topic_variables;

