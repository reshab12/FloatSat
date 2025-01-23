#pragma once

#include "main.hpp"

class Commander : public StaticThread<>,Subscriber {
    private:
        uint8_t status = -1;
    public:
        void init();

        void run();
    
        Commander(int32_t priority);

        uint32_t put(const uint32_t topic_id, const size_t len, void *msg, const NetMsgInfo &) override;

};