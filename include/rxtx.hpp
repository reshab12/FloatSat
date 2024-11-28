#include "rodos.h"
#include "gateway.h"
#include "topics.hpp"

/* ~~~~~ Topic definition ~~~~~ */
/*
const uint32_t topic_id_python_to_rodos = 1002;
const uint32_t topic_id_rodos_to_python = 1003;

// Python Middleware to STM32
struct telecommand 
{
  uint8_t command_id;
  double command_variable;
};

// STM32 to Python Middleware
struct __attribute__((packed)) telemetry
{
  int64_t number;
};

Topic<telecommand> topic_telecommand_uplink(topic_id_python_to_rodos, "topic_sensor");
Topic<telemetry> topic_telemetry_downlink(topic_id_rodos_to_python, "topic_time");*/

/* ~~~~~ Set UART as gateway ~~~~~ */

static HAL_UART bluetooth(UART_IDX2); // Tx: PD5, Rx: PD6
static LinkinterfaceUART link_name_not_imp(&bluetooth, 115200, 3, 10);
static Gateway gw_name_not_imp(&link_name_not_imp, true);

/* ~~~~~ Transmitter thread ~~~~~ */

class Transmitter : public StaticThread<>
{
private:
  telemetry telem;
public:
  Transmitter();

  void init();

  void run();

};

/* ~~~~~ Receiver thread ~~~~~ */

struct Receiver : public Subscriber
{
  Receiver();

  uint32_t put(const uint32_t topic_id, const size_t len, void *msg, const NetMsgInfo &) override;
};