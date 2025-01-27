#pragma once

//#include "readIMU.hpp"
#include "math.h"

#include "topics.hpp"
#include "encoder.h"

#define I_WHEEL 0.0001175
#define I_SATELLITE 0.00834099434
#define KP_P 1
#define KI_P 0
#define KD_P 0.001
#define KP_V 0.03
#define KI_V 0.003
#define KD_V 0.002

#define KP_M 8
#define KI_M 0.9
#define KD_M 0.5

#define INCREMENTS 4000
#define FREQUENCY 50
#define CONTROLTIME 0.005
#define MAX_RPM  7000 // 4100
#define MIN_RPM 200
#define MAX_RAD_PER_SEC (MAX_RPM * 2 * M_PI) / 60
#define MIN_RAD_PER_SECOND (MIN_RPM * 2 * M_PI) / 60
#define MAX_VOLTS 5

void calcPIDMotor(controller_errors* errors, control_value* control,motor_control_value* motor_control, motor_data* data);

void calcPIDPos(requested_conntrol* request, position_data* pos, controller_errors* errors);

float calcPIDVel(requested_conntrol* request, controller_errors* errors, position_data* pose,float last_heading);

void calcVel_with_torque(motor_data* motor_data, float torque, control_value* control);

class VelocityControler: StaticThread<>{
public: 
    VelocityControler(const char* name, int32_t priority);

    void init();

    void run();
};

class PositionControler : StaticThread<>{
public:
    PositionControler(const char* name, int32_t priority);

    void init();

    void run();
};