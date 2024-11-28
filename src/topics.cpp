
#include "rodos.h"
#include "topics.hpp"

Topic<telecommand> topic_telecommand_uplink(topic_id_telecommand, "topic_sensor");
Topic<telemetry> topic_telemetry_downlink(topic_id_telemetry, "topic_time");