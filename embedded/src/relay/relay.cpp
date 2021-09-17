#include <Arduino.h>
#include "WS2812Serial.h"

#include "RH_RF69.h"
#include "DataProtocol.h"
#include "definitions.h"
#include "rocket.h"

void sendCallback(uint8_t id, uint8_t* buf, uint8_t len);

DataProtocol usb_protocol {sendCallback};
RH_RF69 radio {PIN_RF_CS, PIN_RF_G0};
byte drawingMemory[3];         //  3 bytes per LED
DMAMEM byte displayMemory[12]; // 12 bytes per LED
WS2812Serial rgb(1, displayMemory, drawingMemory, PIN_RGB_TX, WS2812_RGB);

bool error = false;

void initPins() {
    pinMode(PIN_RF_CS, OUTPUT);
    pinMode(PIN_BAT_READ, INPUT);
    pinMode(PIN_RF_RST, OUTPUT);
    pinMode(PIN_FLASH_CS, OUTPUT);
    pinMode(PIN_BUZZER, OUTPUT);
    pinMode(PIN_PYRO_ARM, OUTPUT);
    pinMode(PIN_PYRO_1, OUTPUT);
    pinMode(PIN_PYRO_2, OUTPUT);
    pinMode(PIN_PYRO_3, OUTPUT);
    pinMode(PIN_PYRO_4, OUTPUT);
    
    SPI.setMISO(PIN_MISO);
    SPI.setMOSI(PIN_MOSI);
    SPI.setSCK(PIN_SCK);
    SPI.begin();

    digitalWrite(PIN_RF_CS, HIGH);
    digitalWrite(PIN_FLASH_CS, HIGH);
    digitalWrite(PIN_PYRO_ARM, LOW);
    delay(100);
}

void initRadio() {
    digitalWrite(PIN_RF_RST, LOW);
    delay(100);
    digitalWrite(PIN_RF_RST, HIGH);
    delay(100);
    digitalWrite(PIN_RF_RST, LOW);
    delay(100);
    if (!radio.init()) {
        Serial.println("could not initialize radio modem");
        error = true;
        return;
    }
    radio.setFrequency(FREQUENCY);
    radio.setTxPower(TX_POWER, true);
    radio.setModemConfig(MODULATION);
    if (IS_HAM) { 
        radio.send(CALLSIGN, sizeof(CALLSIGN));
    }
}

void sendCallback(uint8_t id, uint8_t* buf, uint8_t len) {
    rocket::parse_message(id, buf);
    uint8_t out_buf_len = HEADER_SIZE + len;
    uint8_t out_buf[out_buf_len];
    uint8_t index = 0;
    DataProtocol::build_header(id, out_buf, &index);
    memcpy(out_buf + index, buf, len);
    radio.send(out_buf, out_buf_len);
}

void rocket::rx(rocket::handshake_from_everyone_to_everyone msg) {
    rocket::handshake_from_everyone_to_everyone response;
    uint8_t out_buf[response.get_size() + HEADER_SIZE];
    uint8_t index = 0;
    DataProtocol::build_buf(&response, out_buf, &index);
    delay(200);
    Serial.write(out_buf, index);
    delay(500);
}

void setup() {
    Serial.begin(BAUD);
    initPins();
    rgb.begin();
    initRadio();
    if (!error) {
        rgb.setPixel(0, WHITE);
        rgb.show();
    } else {
        rgb.setPixel(0, ORANGE);
        rgb.show();
    }
}

void loop() {
    uint8_t radio_buf[255];
    uint8_t radio_len = 255;
    while (Serial.available() > 0) {
        usb_protocol.parse_byte(Serial.read());
    }
    if (radio.recv(radio_buf, &radio_len)) {
        Serial.write(radio_buf, radio_len);
    }
}