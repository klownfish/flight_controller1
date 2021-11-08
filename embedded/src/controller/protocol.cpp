#include "controller.h"

#include "DataProtocol.h"
#include "protocol.h"
#include "rocket.h"

void smash();

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

void sendMsg(rocket::MessageBase* msg, enum send_when send) {
    uint8_t id = msg->get_id();
    uint8_t len = msg->get_size() + HEADER_SIZE;
    uint8_t buf[len];
    DataProtocol::build_buf(msg, buf, &len);
    if (flash_enabled) {
        flash.writeByteArray(flash_addr, buf, len);
        flash_addr += len;
    }

    switch (send) {
        case REGULAR:
            message_count[id] = (message_count[id] + 1) % relay_frequency;
            if (message_count[id] == 0) {
                if (telemetry_enabled) {
                    radio.send(buf, len);
                }
                #ifdef SERIAL_TELEMETRY
                Serial.write(buf, len);
                #endif
            }
            break;
        case ALWAYS:
            if (telemetry_enabled) {
                radio.send(buf, len);
            }
            #ifdef SERIAL_TELEMETRY
            Serial.write(buf, len);
            #endif
            break;
        case NEVER:
            break;
        case OFFLINE:
            #ifdef SERIAL_TELEMETRY
            Serial.write(buf, len);
            #endif
            break;
    }
}

void dataProtocolCallback(uint8_t id, uint8_t* buf, uint8_t len) {
    rocket::parse_message(id, buf);

    uint8_t header[HEADER_SIZE];
    uint8_t header_len;
    DataProtocol::build_header(id, header, &header_len);

    if (flash_enabled) {
        flash.writeByteArray(flash_addr, header, header_len);
        flash_addr += header_len;
        flash.writeByteArray(flash_addr, buf, len);
        flash_addr += len;
    }
}

void rocket::rx(rocket::handshake_from_everyone_to_everyone msg) {
    delay(200);
    sendMsg(&msg, OFFLINE);
    delay(500);
}

void rocket::rx(rocket::set_state_from_ground_to_rocket msg) {
    enterState(msg.get_state());
    rocket::state_from_rocket_to_ground response;
    response.set_state(rocket_state);
    sendMsg(&response, ALWAYS);
}

void rocket::rx(rocket::mag_calibration_from_ground_to_rocket msg) {
    rgb.setPixel(0, BLUE);
    rgb.show();
    mpu.setMagneticDeclination(msg.get_declination());
    mpu.calibrateMag();
    rgb.setPixel(0, OK_COLOR);
    rgb.show();
}

void dance();
void rocket::rx(rocket::play_music_from_ground_to_rocket msg) {
    if (rocket_state == state::sleeping || rocket_state == state::awake) {
        dance();
    }
}

void rocket::rx(rocket::wipe_flash_from_ground_to_rocket msg) {
    if (msg.get_this_to_42() != 42) return;
    rgb.setPixel(0, ORANGE);
    rgb.show();
    flash.eraseChip();
    flash_addr = 0;
    rgb.setPixel(0, OK_COLOR);
    rgb.show();
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
        flash.readByteArray(i, buf, 256);
        Serial.write(buf, 256);
    }
    delay(5000);
    rgb.setPixel(0, OK_COLOR);
    rgb.show();
}