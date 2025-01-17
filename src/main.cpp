#include "main.hpp"


static Transmitter transmitter;
static Receiver receiver;
static Sensor sensorThread("readIMU");
static MotorControler motorControllerThread("motorControllerThread");
static Commander raspberryCommanderThread;
