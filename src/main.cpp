#include "main.hpp"


static Transmitter transmitter(190);
static Receiver receiver;
static Sensor sensorThread("readIMU",200);
static MotorControler motorControllerThread("motorControllerThread",210);
//static Commander raspberryCommanderThread(220);
//static VelocityControler velocityControler("VelocityControlerThread",230);
//static PositionControler positionControler("PositionControler",240);
