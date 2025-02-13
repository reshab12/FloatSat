#include "main.hpp"

class SunSensor:StaticThread<>{
public:
    SunSensor(const char* name, int32_t priority);
    void init();

    void run();
};
