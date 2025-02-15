#pragma once
#include "main.hpp"

void initializeTorquers();

void driveTorquers1(uint16_t value);

void driveTorquers2(uint16_t value);

float dead_angle(float speed);

class MagTorquer: public Subscriber, public StaticThread<>{
private:
    CommBuffer<satellite_mode> inputMsgBuffer;
public:
    MagTorquer(const char* name, int32_t priority);
    void init();

    void run();

    uint32_t put(const uint32_t topicId, const size_t len, void *data, [[gnu::unused]] const NetMsgInfo &netMsgInfo);
     
};
