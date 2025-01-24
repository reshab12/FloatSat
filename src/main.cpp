#include "main.hpp"


//static Transmitter transmitter(250);
//static Receiver receiver;
//static Sensor sensorThread("readIMU",200);
//static MotorControler motorControllerThread("motorControllerThread",240);
//static Commander raspberryCommanderThread(220);
//static VelocityControler velocityControler("VelocityControlerThread",230);
//static PositionControler positionControler("PositionControler",240);

static ReadADCPins readPins("ReadPins", 250);
static MagTorquer magTorquer("Torquer", 200);