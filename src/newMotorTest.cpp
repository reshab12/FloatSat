#include "rodos.h"
#include "hal_pwm.h"

#include "encoder.h"


HAL_PWM pwmMotor(PWM_IDX00);
//additional_sensor_data data;

void initializeMotor(){
    pwmMotor.init(5000,400);
}


/*float getMaxMotorSpeed()
{
    float speed = 0.0;
    for (int i = 0; i < 1000; i++)
    {
        getMotorSpeed(&data);
        speed += data.motorSpeed;
        AT(NOW() + 20 * MILLISECONDS);
    }
    return speed/1000.0;
}


class MotorTest : StaticThread<>{
public:
    MotorTest(const char* name):StaticThread(name){};

    void init(){
        initializeMotor();
    }

    void run(){
        float speed;
        uint16_t velocity = 200;
        pwmMotor.write(velocity);
        PRINTF("Speed is %d \r\n", velocity);
        AT(NOW() + 1 * SECONDS);
        velocity += 200;
        pwmMotor.write(velocity);
        PRINTF("Speed is %d \r\n", velocity);
        AT(NOW() + 1 * SECONDS);
        velocity += 200;
        pwmMotor.write(velocity);
        PRINTF("Speed is %d \r\n", velocity);
        AT(NOW() + 1 * SECONDS);
        velocity += 200;
        pwmMotor.write(velocity);
        PRINTF("Speed is %d \r\n", velocity);
        AT(NOW() + 1 * SECONDS);
        velocity += 200;
        pwmMotor.write(velocity);
        PRINTF("Speed is %d \r\n", velocity);
        AT(NOW() + 1 * SECONDS);
        velocity += 200;
        pwmMotor.write(velocity);
        PRINTF("Speed is %d \r\n", velocity);
        AT(NOW() + 1 * SECONDS);
        velocity += 200;
        pwmMotor.write(velocity);
        PRINTF("Speed is %d \r\n", velocity);
        AT(NOW() + 1 * SECONDS);
        velocity += 200;
        pwmMotor.write(velocity);
        PRINTF("Speed is %d \r\n", velocity);
        AT(NOW() + 1 * SECONDS);
        velocity += 200;
        pwmMotor.write(velocity);
        PRINTF("Speed is %d \r\n", velocity);
        AT(NOW() + 1 * SECONDS);
        velocity += 200;
        pwmMotor.write(velocity);
        PRINTF("Speed is %d \r\n", velocity);
        AT(NOW() + 1 * SECONDS);
        velocity += 200;
        pwmMotor.write(velocity);
        PRINTF("Speed is %d \r\n", velocity);
        AT(NOW() + 1 * SECONDS);
        velocity += 200;
        pwmMotor.write(velocity);
        PRINTF("Speed is %d \r\n", velocity);
        AT(NOW() + 1 * SECONDS);
        velocity += 200;
        pwmMotor.write(velocity);
        PRINTF("Speed is %d \r\n", velocity);
        AT(NOW() + 1 * SECONDS);
        velocity += 200;
        pwmMotor.write(velocity);
        PRINTF("Speed is %d \r\n", velocity);
        AT(NOW() + 1 * SECONDS);
        velocity += 200;
        pwmMotor.write(velocity);
        PRINTF("Speed is %d \r\n", velocity);
        AT(NOW() + 1 * SECONDS);
        velocity += 200;
        pwmMotor.write(velocity);
        PRINTF("Speed is %d \r\n", velocity);
        AT(NOW() + 1 * SECONDS);
        velocity += 200;
        pwmMotor.write(velocity);
        PRINTF("Speed is %d \r\n", velocity);
        AT(NOW() + 1 * SECONDS);
        velocity += 200;
        pwmMotor.write(velocity);
        PRINTF("Speed is %d \r\n", velocity);
        AT(NOW() + 1 * SECONDS);
        velocity += 200;
        pwmMotor.write(velocity);
        PRINTF("Speed is %d \r\n", velocity);
        AT(NOW() + 1 * SECONDS);
        velocity += 200;
        pwmMotor.write(velocity);
        PRINTF("Speed is %d \r\n", velocity);
        AT(NOW() + 1 * SECONDS);
        speed = getMaxMotorSpeed();
        PRINTF("Max Speed: %f \r\n", speed);
    }
};*/

class EncoderTest : StaticThread<>{
public:
    EncoderTest(const char* name):StaticThread(name){}

    void init(){
        EncoderInit();
        initializeMotor();
    }

    void run(){
        //int last_cnt = 0;

        pwmMotor.write(1000);
        /*TIME_LOOP(NOW(), 20 * MILLISECONDS)
        {
          int cnt = get_count();

          if(cnt != last_cnt)
          {
            PRINTF("Counts: %d\r\n", get_count());
            last_cnt = cnt;
          }
        }*/
        MotorSpeedUpdate();
    }
};

EncoderTest encoderTest("EncoderTest");
//MotorTest motorTest("MotorTest");