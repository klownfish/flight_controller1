#include "DataProtocol.h"
#include "rocket.h"

__attribute__((weak))
void dataProtocolCallback(uint8_t id, uint8_t* buf, uint8_t len){}

//Can only read messages defined by the flight controller protocol
void DataProtocol::parse_byte(uint8_t byte) {
  switch(state) {
    case START1:
      if (byte == INIT_FRAME_1) {
        state = START2;
      }
      break;

    case START2:
      if (byte == INIT_FRAME_2) {
        state = TYPE;
      } else {
        state = START1;
      }
      break;

    case TYPE:
      if (not rocket::is_valid_id(byte)) {
        state = START1;
        break;
      }
      payload_length = rocket::id_to_len(byte);
      payload_id = byte;
      state = PAYLOAD;
      payload_index = 0;
      break;

    case PAYLOAD:
      payload_buf[payload_index] = byte;
      payload_index++;
      break;
  }

  if (payload_index == payload_length && state == PAYLOAD) {
    state = START1;
    payload_index = 0;
    (*callback)(payload_id, payload_buf, payload_length);
  }
}

void DataProtocol::parse_bytes(uint8_t* buf, uint8_t len) {
  for (uint8_t i = 0; i < len; i++) {
    parse_byte(buf[i]);
  }
}

//can read all messages
void DataProtocol::build_header(uint8_t id, uint8_t* buf, uint8_t* index) {
  *index = 0;
  buf[(*index)++] = INIT_FRAME_1;
  buf[(*index)++] = INIT_FRAME_2;
  buf[(*index)++] = id;
}