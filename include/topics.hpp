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
const uint32_t topic_id_vel_errors = 1018;
const uint32_t topic_id_mot_errors = 1019;
const uint32_t topic_id_motor_control_value = 1020;

const uint32_t topic_id_raspberry_command = 1120;
const uint32_t topic_id_raspberry_receive = 1121;
const uint32_t topic_id_raspberry_settings = 1122;
const uint32_t topic_id_raspberry_attitude = 1123;
const uint32_t topic_id_raspberry_control_value = 1124;

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
    BACKWARD,
    BREAK
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
    float w[3] = {0,0,0};
    float m[3] = {0,0,0};
    float a[3] = {0,0,0};   //W is in dps
};

//angular position data 
//from sensorfusion to controller
struct position_data
{
    float headingMagneto = 0;   //-180 to 180 degrees
	float headingGyro = 0;      //-180 to 180 degrees
    float heading = 0;          //-180 to 180 degrees
    float moving = 0;           //dps
};

//the value send to the motor-controller
struct control_value
{
    int32_t desiredMotorSpeed = 0;     //Desired motor RPM from the velocity controler
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
    float motorCurrent;         //Current consumed by the motor.
    float magTorquerCurrent;    //Current consumed by the mag torquers
    float boardCurrent;         //Current consumed by the boards and LEDs.
    float batterieVoltage;      //Voltage in the batteries.
    float boardVoltage;         //Voltage after the voltage devider.
    //float solarPanel;           //Current at the solar panel.
    //float allCurrent;
};

struct motor_data
{
    int32_t motorSpeed;    //RPM of the motor
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
    uint8_t new_folder_number;
};

struct controller_errors
{
    float merror, merror_change, mLast_error = 0;  //errors for the motor controler
    float verror, verror_change, vLast_error = 0;  //errors for the velocity controler
    float perror, perror_change, pLast_error = 0;  //errors for the position controler
    float mIerror = 0;
    float vIerror = 0;
    float pIerror = 0;
    float meb = 0;
    float veb = 0;
    float peb = 0;
};

struct controller_errors_s
{
    float error, Ierror, error_change, Last_error = 0;  //errors
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

    motor_data motor_dat; //lf

    control_value control;   //l

    requested_conntrol req_conntrol; //2f
    requested_conntrol user_req_conntrol; //2f

    controller_errors_s vel_errors;//4f

    controller_errors_s mot_errors;//4f

    additional_sensor_data sensor_data;//5f

    motor_control_value motor_control;

    float raspberry_attitude;

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
extern Topic<controller_errors_s> topic_vel_errors;
extern Topic<controller_errors_s> topic_mot_errors;
extern Topic<motor_control_value> topic_motor_control_value;

extern Topic<raspberry_command> topic_raspberry_command;
extern Topic<raspberry_receive> topic_raspberry_receive;
extern Topic<raspberry_settings> topic_raspberry_settings;
extern Topic<float> topic_raspberry_attitude;
extern Topic<float> topic_raspberry_control_value;

