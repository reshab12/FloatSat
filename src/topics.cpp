
#include "topics.hpp"

Topic<telecommand> topic_telecommand_uplink(topic_id_telecommand, "topic_telecommand_uplink");

Topic<telemetry> topic_telemetry_downlink(topic_id_telemetry, "topic_telemetry_downlink");

Topic<failed_telecommand> topic_failed_telecommand_downlink(topic_id_failed_command, "topic_failed_telecommand_downlink");


Topic<imu_data> topic_imu_data(topic_id_imu_data,"topic_imu_data");

Topic<position_data> topic_position_data(topic_id_position_data,"topic_position_data");

Topic<control_value> topic_control_value(topic_id_control_value,"topic_control_value");

Topic<additional_sensor_data> topic_additional_sensor_data(topic_id_additional_sensor_data,"topic_additional_sensor_data");

Topic<satellite_mode> topic_satellite_mode(topic_id_satellite_mode,"topic_satellite_mode");

Topic<requested_conntrol> topic_requested_conntrol(topic_id_requested_conntrol,"topic_requested_conntrol");

Topic<requested_conntrol> topic_user_requested_conntrol(topic_id_user_requested_conntrol,"topic_user_requested_conntrol");

Topic<motor_data> topic_motor_data(topic_id_motor_data,"topic_motor_data");

Topic<controller_errors_s> topic_vel_errors(topic_id_vel_errors,"topic_vel_errors");

Topic<controller_errors_s> topic_mot_errors(topic_id_mot_errors,"topic_mot_errors");


Topic<raspberry_command> topic_raspberry_command(topic_id_raspberry_command,"topic_raspberry_command");

Topic<raspberry_receive> topic_raspberry_receive(topic_id_raspberry_receive,"topic_raspberry_receive");

Topic<raspberry_settings> topic_raspberry_settings(topic_id_raspberry_settings,"topic_raspberry_settings");

Topic<float> topic_raspberry_control_value(topic_id_raspberry_control_value,"topic_raspberry_control_value");