#include "rodos.h"
#include "hal_pwm.h"

#include "encoder.h"
#include "readIMU.hpp"
#include "main.hpp"

additional_sensor_data motor;
controller_errors errors;
control_value control;
imu_data data;
position_data position;

/*float calcVar(){
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
*/
class MotorControlerTest : StaticThread<>{
public: 
    MotorControlerTest(const char* name):StaticThread(name){}

    void init(){
        initializeMotor();
    }

    void run(){
        control.desiredMotorSpeed = 300; //Change thist to test different speeds.

        while(1){
            control.turnDirection = FORWARD; //Change this to change turn direction.
            calcPIDMotor(&errors, &control, &motor);
            PRINTF("MotorSpeed: %d \n", motor.motorSpeed);
            AT(NOW() + 5 * MILLISECONDS);
        }
    }
};

class VelocityControlerTest : StaticThread<>{
public: 
    VelocityControlerTest(const char* name):StaticThread(name){}

    void init(){
        initializeMotor();
    }

    void run(){
        control.satVelocity = 5.0;
        int i = 0;
        while(1){
            if(i == 2){
                calcPIDVel(&control, &motor, &errors, &data);
                PRINTF("Sat Speed: %f", data.wy);
                i = 0;
            }
            calcPIDMotor(&errors, &control, &motor);
            i++;
            AT(NOW() + 5 * MILLISECONDS);
        }
    }
};

class PositionControlerTest : StaticThread<>{
public:
    PositionControlerTest(const char* name):StaticThread(name){}

    void init(){
        initializeMotor();
    }

    void run(){
        float pos = 45.0;
        int i = 0;
        while(1){
            if(i == 4){
                calcPIDPos(pos, &position, &motor, &errors, &control);
                calcPIDVel(&control, &motor, &errors, &data);
                i = 0;
                PRINTF("Sat Angle: %f", position.heading);
            }else if(i == 2){
                calcPIDVel(&control, &motor, &errors, &data);
            }
            calcPIDMotor(&errors, &control, &motor);
            AT(NOW() + 5 * MILLISECONDS);
        }
    }
};

MotorControlerTest test("Test");
//VelocityControlerTest vTest("vTest");
//PositionControlerTest pTest("pTest");