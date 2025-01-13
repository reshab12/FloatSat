#include "rodos.h"
#include "hal_pwm.h"

#include "encoder.h"
#include "readIMU.hpp"
#include "main.hpp"

additional_sensor_data motor;
controller_errors errors;
control_value control;


float calcVar(){
    int16_t xyzGyro[3];
    Offsets offset;
    offsetGyro(&offset);
    float sum;
    for(int i = 0; i < 5000; i++){
        readGyro(xyzGyro);
        sum += (xyzGyro[2] * 0.07 - offset.gyro[2]) * (xyzGyro[2] * 0.07 - offset.gyro[2]);
        AT(NOW() + 5 * MILLISECONDS);
    }
    return sum/5000;
}

class EncoderTest : StaticThread<>{
public:
    EncoderTest(const char* name):StaticThread(name){}

    void init(){
        EncoderInit();
        initializeMotor();
    }

    void run(){
        TIME_LOOP(NOW() + 2 * SECONDS, 1 * SECONDS){
            MotorSpeedUpdate(&motor);
            PRINTF("MotorSpeed: %d \n", motor.motorSpeed);
        }
    }
};

class VarianzGyro : StaticThread<>{
public:
    VarianzGyro(const char* name):StaticThread(name){}

    void init(){
        initialize();
    }

    void run(){
        float var;
        var = calcVar();
        PRINTF("Varianz Gyro: %f", var);
    }
};

class MotorControlerTest : StaticThread<>{
public: 
    MotorControlerTest(const char* name):StaticThread(name){}

    void init(){
        initializeMotor();
    }

    void run(){
        int rpm = 300; 
        calcPIDMotor(rpm, &errors, &control, &motor);
        PRINTF("MotorSpeed: %d \n", motor.motorSpeed);
    }
};

MotorControlerTest test("Test");
//VarianzGyro varianzTest("VarianzTest");
//EncoderTest encoderTest("EncoderTest");
//MotorTest motorTest("MotorTest");