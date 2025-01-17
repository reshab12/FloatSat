#pragma once

#include "main.hpp"

class Commander : public StaticThread<>,Subscriber {
    private:
        // 0=start; 1=waiting; 2=setNextStep;
        uint8_t status = -1;
        uint16_t number_of_pictures = 10;
        float heading = 0.0f;
        
    public:
        void init();

        void run();
    
        Commander();

        uint32_t put(const uint32_t topic_id, const size_t len, void *msg, const NetMsgInfo &) override;

};