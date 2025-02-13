#pragma once
#include "main.hpp"

void initializeTorquers();

void driveTorquers(uint16_t value);

class MagTorquer: public Subscriber, public StaticThread<>{
private:
    CommBuffer<satellite_mode> inputMsgBuffer;
public:
    MagTorquer(const char* name, int32_t priority);
    void init();

    void run();

    uint32_t put(const uint32_t topicId, const size_t len, void *data, [[gnu::unused]] const NetMsgInfo &netMsgInfo);
     
};
