#pragma once

#include "rodos.h"


/* ~~~~~ Modes definition ~~~~~ */

//pose estimation mode ids
const uint8_t pose_estimation_mode_imu = 0x00;
const uint8_t pose_estimation_mode_mag_interference = 0x01;
const uint8_t pose_estimation_mode_star_mapper = 0x02;

//pose control mode ids
const uint8_t control_mode_standby = 0x00;
const uint8_t control_mode_pos = 0x01;
const uint8_t control_mode_vel = 0x02;
const uint8_t control_mode_ai = 0x03;

//pose mission mode ids
const uint8_t mission_mode_standby = 0x00;
const uint8_t mission_mode_hibernation = 0x01;
const uint8_t mission_mode_star_mapper = 0x02;
const uint8_t mission_mode_object_detection = 0x03;
