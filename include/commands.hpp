#pragma once

#include "rodos.h"

//abort mission
const uint8_t command_id_abort_mission =            0xf0;

//mission modes
const uint8_t command_id_mission_mode =             0x00;

//controll modes
const uint8_t command_id_control_mode =             0x01;

//pose estimation modes
const uint8_t command_id_pose_estimation_mode =     0x02;

//set pose that should be moved to with PID
const uint8_t command_id_move_to =                  0x03; 

//set speed that should be reached with PID
const uint8_t command_id_accel_to =                 0x04; 


