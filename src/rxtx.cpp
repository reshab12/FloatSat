#include "rxtx.hpp"

static HAL_UART bluetooth(UART_IDX2); // Tx: PD5, Rx: PD6
static LinkinterfaceUART link_name_not_imp(&bluetooth, 115200, 3, 10);
static Gateway gw_name_not_imp(&link_name_not_imp, true);


/* ~~~~~ CommBuffer for Transmitter thread ~~~~~ */
  CommBuffer<satellite_mode> cb_satellite_mode_transmitter_thread;
  // subscriber that fills the CommBuffer cb_satellite_mode_transmitter_thread with values from topic topic_satellite_mode
  Subscriber sub_satellite_mode_transmitter_thread(topic_satellite_mode, cb_satellite_mode_transmitter_thread);

  CommBuffer<imu_data> cb_imu_data_transmitter_thread;
  Subscriber sub_imu_data_transmitter_thread(topic_imu_data, cb_imu_data_transmitter_thread);

  CommBuffer<position_data> cb_position_data_transmitter_thread;
  Subscriber sub_position_data_transmitter_thread(topic_position_data, cb_position_data_transmitter_thread);


/* ~~~~~ Transmitter thread ~~~~~ */

  Transmitter::Transmitter() : StaticThread("STM32 transmitter",210) {}

  void Transmitter::init()
  {
    bluetooth.init(115200);
  }

  void Transmitter::run()
  {
    satellite_mode mode;
    imu_data data;
    position_data pose;
    TIME_LOOP(0, 100 * MILLISECONDS)
    {
      telem.time = (NOW() / MICROSECONDS);

      cb_satellite_mode_transmitter_thread.get(mode);
      telem.satellite_modes = mode;

      cb_imu_data_transmitter_thread.get(data);
      telem.imu = data;

      cb_position_data_transmitter_thread.get(pose);
      telem.position = pose;

      //MW_PRINTF("Rodos sends telemetry: %lld\n",(NOW() / MICROSECONDS));
      //PRINTF("Rodos sends telemetry: %lld\n",(NOW() / MICROSECONDS));
      topic_telemetry_downlink.publish(telem);
    }
  }



/* ~~~~~ CommBuffer for Receiver thread ~~~~~ */

  CommBuffer<satellite_mode> cb_satellite_mode_receiver_thread;
  Subscriber sub_satellite_mode_receiver_thread(topic_satellite_mode, cb_satellite_mode_receiver_thread);

  CommBuffer<variables> cb_variables_receiver_thread;
  Subscriber sub_variables_receiver_thread(topic_variables, cb_variables_receiver_thread);

/* ~~~~~ Receiver thread ~~~~~ */

  Receiver::Receiver() : Subscriber(topic_telecommand_uplink, "STM32 receiver") {}


  uint32_t Receiver::put(const uint32_t topic_id, const size_t len, void *msg, const NetMsgInfo &)
  {
    
    telecommand *telecom = (telecommand *)msg;
    satellite_mode mode;
    variables variables;

       MW_PRINTF("Python sends command-ID: %d, variable: %ld\n",telecom->command_id,telecom->command_variable);

    cb_satellite_mode_receiver_thread.get(mode);
    cb_variables_receiver_thread.get(variables);
    switch (telecom->command_id)
    {
    case command_id_abort_mission:
      if(mode.mission_mode != mission_mode_standby){
        mode.mission_mode = mission_mode_standby;
        mode.control_mode = control_mode_vel;
        variables.requested_rot_speed = 0;
        topic_variables.publish(variables);
        topic_satellite_mode.publish(mode);
      }
      break;

    case command_id_mission_mode:      
      if(mode.mission_mode == mission_mode_standby){
        //whitch mission should be started:
        switch (telecom->command_variable)
        {
        case mission_mode_standby:
          //do nothing mode is in standby
          break;

        case mission_mode_hibernation:
          //change mode to hibernation
          break;

        case mission_mode_star_mapper:
          //start star mapping
          break;

        case mission_mode_object_detection:
          //start object detection
          mode.control_mode = control_mode_vel;
          mode.mission_mode = mission_mode_object_detection;
          variables.requested_rot_speed = 1;

          topic_variables.publish(variables);
          topic_satellite_mode.publish(mode);
          break;

        default:
          break;
        }
      }else{
        MW_PRINTF("mission not in standby");
      }
      break;
    
    case command_id_control_mode:
      mode.control_mode = telecom->command_variable;
      topic_satellite_mode.publish(mode);
      break;

    case command_id_pose_estimation_mode:
      mode.pose_estimation_mode = telecom->command_variable;     
      topic_satellite_mode.publish(mode);
      break;

    case command_id_move_to:
      variables.requested_angle = telecom->command_variable;
      mode.control_mode = control_mode_pos;

      topic_variables.publish(variables);
      topic_satellite_mode.publish(mode);
      break;

    case command_id_accel_to:
      variables.requested_rot_speed = telecom->command_variable;
      mode.control_mode = control_mode_vel;

      topic_variables.publish(variables);
      topic_satellite_mode.publish(mode);
      break;

    default:
      break;
    }

    return true;
  }
 
 



