
#ifndef __sateliteModes_hpp__
#define __sateliteModes_hpp__

#include "rodos.h"


/* ~~~~~ Modes definition ~~~~~ */

//pose estimation modes
const uint8_t pose_estimation_mode_imu = 0x01;
const uint8_t pose_estimation_mode_mag_interference = 0x02;
const uint8_t pose_estimation_mode_star_mapper = 0x03;

//pose control modes
const uint8_t control_mode_standby = 0x01;
const uint8_t control_mode_pos = 0x02;
const uint8_t control_mode_vel = 0x03;
const uint8_t control_mode_ai = 0x04;

//pose mission modes
const uint8_t mission_mode_standby = 0x01;
const uint8_t mission_mode_hibernation = 0x02;
const uint8_t mission_mode_star_mapper = 0x03;
const uint8_t mission_mode_object_detection = 0x04;

#endif