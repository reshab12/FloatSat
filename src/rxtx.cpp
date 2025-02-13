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

  CommBuffer<motor_data> cb_motor_data_Transmitter;
  Subscriber sub_motor_data_Transmitter(topic_motor_data, cb_motor_data_Transmitter);


  CommBuffer<requested_conntrol> cb_user_requested_conntrol_Transmitter;
  Subscriber sub_user_requested_conntrol_Transmitter(topic_user_requested_conntrol, cb_user_requested_conntrol_Transmitter);

  CommBuffer<requested_conntrol> cb_requested_conntrol_Transmitter;
  Subscriber sub_requested_conntrol_Transmitter(topic_requested_conntrol, cb_requested_conntrol_Transmitter);

  
  CommBuffer<control_value> cb_control_value_Transmitter;
  Subscriber sub_control_value_Transmitter(topic_control_value, cb_control_value_Transmitter);

  CommBuffer<controller_errors_s> cb_vel_errors_Transmitter;
  Subscriber sub_vel_errors_Transmitter(topic_vel_errors, cb_vel_errors_Transmitter);

  CommBuffer<controller_errors_s> cb_pos_errors_Transmitter;
  Subscriber sub_pos_errors_Transmitter(topic_pos_errors, cb_pos_errors_Transmitter);

  CommBuffer<controller_errors_s> cb_mot_errors_Transmitter;
  Subscriber sub_mot_errors_Transmitter(topic_mot_errors, cb_mot_errors_Transmitter);

  CommBuffer<additional_sensor_data> cb_additional_sensor_data_Transmitter;
  Subscriber sub_additional_sensor_data_Transmitter(topic_additional_sensor_data, cb_additional_sensor_data_Transmitter);

  CommBuffer<motor_control_value> cb_motor_control_value_Transmitter;
  Subscriber sub_motor_control_value_Transmitter(topic_motor_control_value, cb_motor_control_value_Transmitter);

  CommBuffer<float> cb_raspberry_attitude_Transmitter;
  Subscriber sub_raspberry_attitude_Transmitter(topic_raspberry_attitude,cb_raspberry_attitude_Transmitter);

  CommBuffer<float> cb_raspberry_control_value_Transmitter;
  Subscriber sub_raspberry_control_value_Transmitter(topic_raspberry_control_value,cb_raspberry_control_value_Transmitter);


/* ~~~~~ Transmitter thread ~~~~~ */

  Transmitter::Transmitter(int32_t priority) : StaticThread("STM32 transmitter", priority) {}

  void Transmitter::init()
  {
    bluetooth.init(115200);
  }

  void Transmitter::run()
  {
    satellite_mode mode;
    imu_data data;
    position_data pose;
    motor_data motor_dat;
    requested_conntrol user_req_con;
    requested_conntrol req_con;
    control_value control;
    controller_errors_s vel_errors;
    controller_errors_s pos_errors;    
    controller_errors_s mot_errors;
    additional_sensor_data sensor_data;
    motor_control_value motor_control;
    float raspberry_attitude = 0;
    float torque = 0;
    TIME_LOOP(0, 100 * MILLISECONDS)
    {
      telem.time = (NOW() / MICROSECONDS);

      cb_satellite_mode_transmitter_thread.get(mode);
      telem.satellite_modes = mode;

      cb_imu_data_transmitter_thread.get(data);
      telem.imu = data;

      cb_position_data_transmitter_thread.get(pose);
      telem.position = pose;

      cb_motor_data_Transmitter.get(motor_dat);
      telem.motor_dat = motor_dat;

      cb_requested_conntrol_Transmitter.get(req_con);

      cb_user_requested_conntrol_Transmitter.get(user_req_con);
      telem.req_conntrol = req_con;
      telem.user_req_conntrol = user_req_con;

      cb_control_value_Transmitter.get(control);
      telem.control = control;

      cb_vel_errors_Transmitter.get(vel_errors);
      telem.vel_errors = vel_errors;

      cb_mot_errors_Transmitter.get(mot_errors);
      telem.mot_errors = mot_errors;

      cb_additional_sensor_data_Transmitter.get(sensor_data);
      telem.sensor_data = sensor_data;

      cb_motor_control_value_Transmitter.get(motor_control);
      telem.motor_control = motor_control;

      cb_raspberry_attitude_Transmitter.get(raspberry_attitude);
      telem.raspberry_attitude = raspberry_attitude;

      cb_pos_errors_Transmitter.get(pos_errors);
      telem.pos_errors = pos_errors;

      cb_raspberry_control_value_Transmitter.get(torque);
      telem.torque = torque;

      //MW_PRINTF("Rodos sends telemetry: %lld\n",(NOW() / MICROSECONDS));
      //PRINTF("Rodos sends telemetry: %lld\n",(NOW() / MICROSECONDS));
      topic_telemetry_downlink.publish(telem);
    }
  }



/* ~~~~~ CommBuffer for Receiver thread ~~~~~ */

  CommBuffer<satellite_mode> cb_satellite_mode_receiver_thread;
  Subscriber sub_satellite_mode_receiver_thread(topic_satellite_mode, cb_satellite_mode_receiver_thread);

  CommBuffer<requested_conntrol> cb_requested_conntrol_receiver_thread;
  Subscriber sub_requested_conntrol_receiver_thread(topic_user_requested_conntrol, cb_requested_conntrol_receiver_thread);

  CommBuffer<position_data> cb_position_data_receiver;
  Subscriber sub_position_data_receiver(topic_position_data, cb_position_data_receiver);


/* ~~~~~ Receiver thread ~~~~~ */

  Receiver::Receiver() : Subscriber(topic_telecommand_uplink, "STM32 receiver") {}


  uint32_t Receiver::put(const uint32_t topic_id, const size_t len, void *msg, const NetMsgInfo &)
  {
    
    telecommand *telecom = (telecommand *)msg;
    satellite_mode mode;
    requested_conntrol requested_conntrol;
    position_data pose;
    control_value control;

       PRINTF("Python sends command-ID: %d, variable: %ld\n",telecom->command_id,telecom->command_variable);

    cb_satellite_mode_receiver_thread.get(mode);
    cb_requested_conntrol_receiver_thread.get(requested_conntrol);
    cb_position_data_receiver.get(pose);
    switch (telecom->command_id)
    {
    case command_id_abort_mission:
      if(mode.mission_mode != mission_mode_standby){
        mode.mission_mode = mission_mode_standby;
        mode.control_mode = control_mode_vel;
        requested_conntrol.requested_rot_speed = 0;

        topic_requested_conntrol.publish(requested_conntrol);
        topic_satellite_mode.publish(mode);
      }
      break;

    case command_id_reboot:
      PRINTF("Reboot");
      RODOS::hwResetAndReboot();
      break;

    case command_id_mission_mode:      
        //whitch mission should be started:
        switch (telecom->command_variable)
        {
        case mission_mode_standby:
          mode.mission_mode = mission_mode_standby;
          topic_satellite_mode.publish(mode);
          break;

        case mission_mode_hibernation:
          //change mode to hibernation
          mode.mission_mode = mission_mode_hibernation;
          topic_satellite_mode.publish(mode);
          break;

        case mission_mode_star_mapper:
          //start star mapping
          if(mode.control_mode != control_mode_pos || mode.control_mode != control_mode_ai_pos)
            mode.control_mode = control_mode_pos;
          mode.mission_mode = mission_mode_star_mapper;
          requested_conntrol.requested_angle = pose.heading;

          topic_user_requested_conntrol.publish(requested_conntrol);
          topic_satellite_mode.publish(mode);
          break;

        case mission_mode_object_detection:
          //start object detection
          mode.control_mode = control_mode_vel;
          mode.mission_mode = mission_mode_object_detection;
          requested_conntrol.requested_rot_speed = 1;

          topic_user_requested_conntrol.publish(requested_conntrol);
          topic_satellite_mode.publish(mode);
          break;

        default:
          break;
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
      requested_conntrol.requested_angle = telecom->command_variable;
      mode.control_mode = control_mode_pos;

      topic_user_requested_conntrol.publish(requested_conntrol);
      topic_satellite_mode.publish(mode);
      break;

    case command_id_accel_to:
      requested_conntrol.requested_rot_speed = telecom->command_variable;
      mode.control_mode = control_mode_vel;
      
      //for motor testing
      //control.desiredMotorSpeed = telecom->command_variable;
      //topic_control_value.publish(control);

      //normal operation
      topic_user_requested_conntrol.publish(requested_conntrol);
      topic_satellite_mode.publish(mode);
      break;

    default:
      break;
    }

    return true;
  }
 
 



