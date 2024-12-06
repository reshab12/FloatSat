#include "rxtx.hpp"
#include "commands.hpp"

/* ~~~~~ Transmitter thread ~~~~~ */


  Transmitter::Transmitter() : StaticThread("STM32 transmitter") {}

  void Transmitter::init()
  {
    bluetooth.init(115200);
  }

  void Transmitter::run()
  {
    TIME_LOOP(0, 2000 * MILLISECONDS)
    {
      telem.number = NOW() / MICROSECONDS;
      topic_telemetry_downlink.publish(telem);
    }
  }



/* ~~~~~ Receiver thread ~~~~~ */


  Receiver::Receiver() : Subscriber(topic_telecommand_uplink, "STM32 receiver") {}

  uint32_t Receiver::put(const uint32_t topic_id, const size_t len, void *msg, const NetMsgInfo &)
  {
    telecommand *telecom = (telecommand *)msg;

       PRINTF("Python sends command-ID: %d, variable: %lf\n",telecom->command_id,telecom->command_variable);

    switch (telecom->command_id)
    {
    case command_id_mission_mode:
      satellite_mode mode;
      cb_satellite_mode_receiver_thread.get(mode);
      mode.mission_mode = telecom->command_variable;
      topic_satellite_mode.publish(mode);
      break;
    
    case command_id_control_mode:
      satellite_mode mode;
      cb_satellite_mode_receiver_thread.get(mode);
      mode.control_mode = telecom->command_variable;
      topic_satellite_mode.publish(mode);
      break;

    case command_id_pose_estimation_mode:
      satellite_mode mode;
      cb_satellite_mode_receiver_thread.get(mode);
      mode.pose_estimation_mode = telecom->command_variable;     
      topic_satellite_mode.publish(mode);
      break;

    default:
      break;
    }

    return true;
  }
 
 static Transmitter transmitter;
 static Receiver receiver;

