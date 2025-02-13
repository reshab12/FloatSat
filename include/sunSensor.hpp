#include "main.hpp"

void readSolarPanel(float voltage);

class SunSensor:public Subscriber, public StaticThread<>{
private:
    CommBuffer<satellite_mode> sunMsgBuffer;
public:
    SunSensor(const char* name, int32_t priority);
    void init();

    void run();

    uint32_t put(const uint32_t topicId, const size_t len, void *data, [[gnu::unused]] const NetMsgInfo &netMsgInfo);
};
