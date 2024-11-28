
#include "rodos.h"
#include "topics.hpp"

Topic<telecommand> topic_telecommand_uplink(topic_id_telecommand, "topic_telecommand_uplink");
Topic<telemetry> topic_telemetry_downlink(topic_id_telemetry, "topic_telemetry_downlink");
Topic<imu_data> topic_imu_data(topic_id_imu_data,"topic_imu_data");
Topic<position_data> topic_position_data(topic_id_position_data,"topic_position_data");
Topic<control_value> topic_control_value(topic_id_control_value,"topic_control_value");
Topic<additional_sensor_data> topic_additional_sensor_data(topic_id_additional_sensor_data,"topic_additional_sensor_data");
Topic<satellite_mode> topic_satellite_mode(topic_id_satellite_mode,"topic_satellite_mode");