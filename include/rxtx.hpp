#include "rodos.h"
#include "gateway.h"
#include "topics.hpp"

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