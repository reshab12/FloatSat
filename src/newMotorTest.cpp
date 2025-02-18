#include "rodos.h"
#include "hal_pwm.h"

#include "encoder.h"
#include "readIMU.hpp"
#include "main.hpp"

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

class MotorTest : StaticThread<>{
private:
    motor_data motor;
    control_value control;
public:
    MotorTest(const char* name):StaticThread(name){}

    void init(){
        EncoderInit();
        initializeMotor();
    }

    void run(){
        control.increments = 0;
        control.turnDirection = FORWARD;
        driveMotor(&control);
        TIME_LOOP(1 * SECONDS, 200 * MILLISECONDS){
            MotorSpeedUpdate(&motor);
            PRINTF("RPM %d \n", motor.motorSpeed);
        }
    }
};

class MotorControlerTest : StaticThread<>{
private:
    motor_data motor;
    controller_errors errors;
    control_value control;
public: 
    MotorControlerTest(const char* name):StaticThread(name){}

    void init(){
        EncoderInit();
        initializeMotor();
    }

    void run(){
        control.desiredMotorSpeed = 8000;    //Change thist to test different speeds.
        control.turnDirection = BACKWARD;    //Change this to change turn direction.
        while(1){
            MotorSpeedUpdate(&motor);
            PRINTF("MotorSpeed: %d \n", motor.motorSpeed);
            calcPIDMotor(&errors, &control, &motor);
            driveMotor(&control);
            AT(NOW() + 5 * MILLISECONDS);
        }
    }
};

class VelocityControlerTest : StaticThread<>{
private:
    motor_data motor;
    controller_errors errors;
    control_value control;
    imu_data data;
    position_data position;
public: 
    VelocityControlerTest(const char* name):StaticThread(name){}

    void init(){
        initializeMotor();
    }

    void run(){
        requested_conntrol requested_conntrol;
        requested_conntrol.requested_rot_speed = 5;
        int i = 0;
        while(1){
            if(i == 2){
                calcPIDVel(&requested_conntrol, &errors, &data, &control);
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
        motor_data motor;
        controller_errors errors;
        control_value control;
        imu_data data;
        position_data position;
        requested_conntrol requested_conntrol;
        requested_conntrol.requested_angle = 45;
        float pos = 45.0;
        int i = 0;
        while(1){
            i++;
            if(i == 4){
                calcPIDPos(&requested_conntrol, &position, &errors);
                calcPIDVel(&requested_conntrol, &errors, &data, &control);
                i = 0;
                PRINTF("Sat Angle: %f", position.heading);
            }else if(i == 2){
                calcPIDVel(&requested_conntrol, &errors, &data, &control);
            }
            calcPIDMotor(&errors, &control, &motor);
            AT(NOW() + 5 * MILLISECONDS);
        }
    }
};

//MotorTest motorTest("MotorTest");
//MotorControlerTest test("Test");
//VelocityControlerTest vTest("vTest");
//PositionControlerTest pTest("pTest");