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
const uint32_t topic_id_requested_conntrol = 1015;
const uint32_t topic_id_motor_data = 1016;
const uint32_t topic_id_user_requested_conntrol = 1017;

const uint32_t topic_id_raspberry_command = 1020;
const uint32_t topic_id_raspberry_receive = 1021;
const uint32_t topic_id_raspberry_settings = 1022;
const uint32_t topic_id_raspberry_control_mode = 1023;
const uint32_t topic_id_raspberry_control_value = 1024;

// telecommand 
//from Python Middleware to STM32
struct telecommand 
{
  uint8_t command_id;
  int32_t command_variable;
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
    uint8_t moving = 0;
};

//the value send to the motor-controller
struct control_value
{
    float desiredMotorSpeed;     //Desired motor RPM from the velocity controler
};

//the value used by the motor-controller
struct motor_control_value
{
    uint16_t increments;            //increments set for the motor PWM
    direction turnDirection;        //CW or CCW
};

//
struct additional_sensor_data
{
    float mainCurrent;      //Current consumed by the enire satellite.
};

struct motor_data
{
    float motorSpeed;    //RPM of the motor
    float omega_wheel;      //From motor PID calulated change to omega wheel.
};

//
struct satellite_mode{
    uint8_t pose_estimation_mode = 0;
    uint8_t control_mode = 0;
    uint8_t mission_mode = 0;
};

struct requested_conntrol
{
    float requested_angle;          //degree -180-180
    float requested_rot_speed;      //dps
};

struct raspberry_command{
    // 0=standby; 1=ready;
    uint8_t status; 
};

struct raspberry_receive{
    //0=standby; 1=ready;
    uint8_t status;

};

struct raspberry_settings{
    uint8_t number_of_pictures;
};

struct controller_errors
{
    float merror, mIerror, merror_change, mLast_error = 0;  //errors for the motor controler
    float verror, vIerror, verror_change, vLast_error = 0;  //errors for the velocity controler
    float perror, pIerror, perror_change, pLast_error = 0;  //errors for the position controler
};

struct telemetry
{
    int64_t time; //l

   //satellite_mode
    satellite_mode satellite_modes; // 3B
      
    //imu_data
    imu_data imu;  //9f

    //position_data
    position_data position; //3fB

    motor_data motor_dat; //2f

    //control_value ccontrol_value;

    requested_conntrol req_conntrol; //2f

    float f;
};


extern Topic<telecommand> topic_telecommand_uplink;
extern Topic<telemetry> topic_telemetry_downlink;
extern Topic<failed_telecommand> topic_failed_telecommand_downlink;

extern Topic<imu_data> topic_imu_data;
extern Topic<position_data> topic_position_data;
extern Topic<control_value> topic_control_value;
extern Topic<additional_sensor_data> topic_additional_sensor_data;
extern Topic<satellite_mode> topic_satellite_mode;
extern Topic<requested_conntrol> topic_requested_conntrol;
extern Topic<requested_conntrol> topic_user_requested_conntrol;
extern Topic<motor_data> topic_motor_data;

extern Topic<raspberry_command> topic_raspberry_command;
extern Topic<raspberry_receive> topic_raspberry_receive;
extern Topic<raspberry_settings> topic_raspberry_settings;
extern Topic<float> topic_raspberry_control_value;

