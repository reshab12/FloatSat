#include "main.hpp"

static ReadADCPins readPins("ReadPins", 120);
static Transmitter transmitter(250);
static Receiver receiver;
static Sensor sensorThread("readIMU",200);
static MotorControler motorControllerThread("motorControllerThread",240);
static Commander raspberryCommanderThread(220);
static VelocityControler velocityControler("VelocityControlerThread",230);
static PositionControler positionControler("PositionControler",245);
static MagTorquer magTorquer("magT",130);
