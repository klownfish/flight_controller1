#include "controller.h"

#include "DataProtocol.h"
#include "protocol.h"
#include "rocket.h"

//#define SERIAL_FLASH
#define SERIAL_RADIO

uint8_t message_count[255] = {0}; //init to 0
uint16_t relay_frequency = ~0;

void handleDataStreams() {
    static DataProtocol radio_protocol;
    static DataProtocol serial_protocol;

    uint8_t buf[RH_RF69_MAX_MESSAGE_LEN]; 
    uint8_t len = RH_RF69_MAX_MESSAGE_LEN;
    if (radio.recv(buf, &len)) {
        radio_protocol.parse_bytes(buf, len);
    }
    while (Serial.available()) {
        serial_protocol.parse_byte(Serial.read());
    }
    while (Serial3.available()) {
        gps.parse_byte(Serial3.read());
    }
}

void sendMsg(rocket::MessageBase* msg) {
    uint8_t id = msg->get_id();
    uint8_t len = msg->get_size() + HEADER_SIZE;
    uint8_t buf[len];
    DataProtocol::build_buf(msg, buf, &len);
    if (flash_enabled && msg->get_receiver() == rocket::nodes::flash) {
        spi_mtx.lock();
        flash.write(flash_addr, buf, len);
        spi_mtx.unlock();
        flash_addr += len;
        #ifdef SERIAL_FLASH
        Serial.write(buf, len);
        #endif
    }

    if (telemetry_enabled && msg->get_receiver() == rocket::nodes::ground) {
        spi_mtx.lock();
        //radio.send(buf, len);
        spi_mtx.unlock();
        #ifdef SERIAL_RADIO
        Serial.write(buf, len);
        #endif
    }

    if (msg->get_receiver() == rocket::nodes::everyone) {
        Serial.write(buf, len);   
    }
}

void dataProtocolCallback(uint8_t id, uint8_t* buf, uint8_t len) {
    rocket::parse_message(id, buf);

    uint8_t header[HEADER_SIZE];
    uint8_t header_len;
    DataProtocol::build_header(id, header, &header_len);

    if (flash_enabled) {
        flash.write(flash_addr, header, header_len);
        flash_addr += header_len;
        flash.write(flash_addr, buf, len);
        flash_addr += len;
    }
}

void rocket::rx(rocket::handshake_from_everyone_to_everyone msg) {
    threads.delay(200);
    sendMsg(&msg);
    threads.delay(500);
}



void dance();
void rocket::rx(rocket::play_music_from_ground_to_rocket msg) {
    if (rocket_state == state::sleeping) {
        dance();
    }
}

void rocket::rx(rocket::wipe_flash_from_ground_to_rocket msg) {
    if (rocket_state != state::sleeping && rocket_state != state::ready) {
        return;
    }
    if (msg.get_this_to_42() != 42) return;
    rgb.setPixel(0, ORANGE);
    rgb.show();
    flash.eraseChip();
    flash_addr = 0;
    rgb.setPixel(0, OK_COLOR);
    rgb.show();
}

void rocket::rx(rocket::set_state_from_ground_to_rocket msg) {
    enterState(msg.get_state());
}

void rocket::rx(rocket::set_logging_from_ground_to_rocket msg) {
    flash_enabled = msg.get_is_enabled();
}

void rocket::rx(rocket::dump_flash_from_ground_to_rocket msg) {
    rgb.setPixel(0, PINK);
    rgb.show();
    uint8_t buf[256];
    delay(1000);
    for (uint32_t i = 0; i < flash_addr; i += 256) {
        flash.read(i, buf, 256);
        Serial.write(buf, 256);
    }
    delay(5000);
    rgb.setPixel(0, OK_COLOR);
    rgb.show();
}