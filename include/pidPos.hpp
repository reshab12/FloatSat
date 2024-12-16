//#include "readIMU.hpp"

#define I_WHEEL 303.0
#define I_SATELLITE 7610.2
#define KP 20000
#define KI 5000
#define KD 35000
#define INCREMENTS 40000
#define FREQUENCY 50
#define CONTROLETIME 1/FREQUENCY * SECONDS

//float calcPID(float desiredAngle, float currentAngle, float dt, float omega_wheel);