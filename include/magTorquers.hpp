#pragma once
#include "main.hpp"

void initializeTorquers();

void driveTorquers(uint16_t value);

class MagTorquer:StaticThread<>{
public:
    MagTorquer(const char* name, int32_t priority);
    void init();

    void run();
};

struct Receiver : public Subscriber
{
  Receiver();

  uint32_t put(const uint32_t topic_id, const size_t len, void *msg, const NetMsgInfo &) override;
};