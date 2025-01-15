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
#define KP_M 10
#define KI_M 5
#define KD_M 0
#define INCREMENTS 4000
#define FREQUENCY 500
#define MOTORFREQUENCY 200
#define MOTORCONTROLTIME 1/MOTORFREQUENCY * SECONDS
#define CONTROLTIME 1/FREQUENCY * SECONDS
#define MAX_RPM  9000 // 4100
#define MIN_RPM 200
#define MAX_RAD_PER_SEC  (MAX_RPM * 2 * M_PI) / 60
#define MAX_VOLTS 5

void calcPIDMotor(controller_errors* errors, control_value* control, additional_sensor_data* data);

void calcPIDPos(float desiredAngle, position_data* pos, additional_sensor_data* data, controller_errors* errors, control_value* control);

void calcPIDVel(control_value* control, additional_sensor_data* data, controller_errors* errors, imu_data* imu);