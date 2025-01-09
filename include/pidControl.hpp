//#include "readIMU.hpp"
#include "math.h"

#include "topics.hpp"
#include "encoder.hpp"

#define I_WHEEL 0.0000391
#define I_SATELLITE 0.00783661541
#define KP_P 20000
#define KI_P 5000
#define KD_P 35000
#define KP_V 200
#define KI_V 50
#define KD_V 350
#define KP_M 10
#define KI_M 5
#define KD_M 15
#define INCREMENTS 40000
#define FREQUENCY 50
#define CONTROLTIME 1/FREQUENCY * SECONDS
#define MAX_RPM  4100 // 9000
#define MIN_RPM 200
#define MAX_RAD_PER_SEC  (MAX_RPM * 2 * M_PI) / 60
#define MAX_VOLTS 5

void calcPIDMotor(float desiredVelocity, controller_errors* controler_errors, control_value* control, additional_sensor_data* data);

void calcPIDPos(float desiredAngle, imu_data* imu, position_data* pos, additional_sensor_data* data, controller_errors* errors);

void calcPIDVel(float desiredSpeed, imu_data* imu, additional_sensor_data* data, controller_errors* errors);