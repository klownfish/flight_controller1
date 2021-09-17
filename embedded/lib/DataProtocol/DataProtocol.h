#include <stdint.h>
#include <string.h>
#include "rocket.h"

#define INIT_FRAME_1 0x0A
#define INIT_FRAME_2 0x0D

#define HEADER_SIZE 3
#define PAYLOAD_BUF_LEN 60

__attribute__((weak))
void dataProtocolCallback(uint8_t id, uint8_t* buf, uint8_t len);

enum receiver_state {
    START1,
    START2,
    TYPE,
    PAYLOAD,
};

class DataProtocol {
public:
  DataProtocol(void) {}
  DataProtocol(void (*func)(uint8_t id, uint8_t* buf, uint8_t len)) {
    callback = func;
  }
  void set_callback(void (*func)(uint8_t id, uint8_t* buf, uint8_t len)){
    callback=func;
  }

  void parse_byte(uint8_t byte);
  void parse_bytes(uint8_t* buf, uint8_t len);

  static void build_header(uint8_t id, uint8_t* buf, uint8_t* index);
  static void build_buf(rocket::MessageBase* msg, uint8_t* buf, uint8_t* len) {
    *len = 0;
    buf[(*len)++] = INIT_FRAME_1;
    buf[(*len)++] = INIT_FRAME_2;
    buf[(*len)++] = msg->get_id();
    msg->build_buf(buf, len);
  }

private:
  void (*callback)(uint8_t id, uint8_t* buf, uint8_t len) = &dataProtocolCallback;
  enum receiver_state state = START1;
  uint8_t payload_length;
  uint8_t payload_buf[PAYLOAD_BUF_LEN];
  uint8_t payload_index;
  uint8_t payload_id;
};