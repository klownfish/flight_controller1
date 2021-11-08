#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#include "RH_RF69.h"
#include "DataProtocol.h"
#include "definitions.h"
#include "rocket.h"

void DataProtocolCallback(uint8_t id, uint8_t* buf, uint8_t len);

DataProtocol usb_protocol {DataProtocolCallback};
DataProtocol radio_protocol {DataProtocolCallback};
RH_RF69 radio {PIN_RF_CS, PIN_RF_G0};
Adafruit_NeoPixel rgb {1, PIN_RGB_TX};

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

void DataProtocolCallback(uint8_t id, uint8_t* buf, uint8_t len) {
    rocket::parse_message(id, buf);
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

void rocket::rx(rocket::arm_pyro_from_ground_to_launchpad msg) {
    digitalWriteFast(PIN_PYRO_ARM, msg.get_armed());
}

void rocket::rx(rocket::enable_pyro_from_ground_to_launchpad msg) {
    uint8_t channel;
    switch (msg.get_channel()) {
        case 1:
            channel = PIN_PYRO_1;
            break;
        case 2:
            channel = PIN_PYRO_2;
            break;
        case 3:
            channel = PIN_PYRO_3;
            break;
        case 4:
            channel = PIN_PYRO_4;
            break;
    }
    
    tone(PIN_BUZZER, BUZZER_HZ, 300);
    delay(600);
    tone(PIN_BUZZER, BUZZER_HZ, 300);
    delay(600);
    tone(PIN_BUZZER, BUZZER_HZ, 300);
    delay(600);

    tone(PIN_BUZZER, BUZZER_HZ, 1000);
    delay(1500);
    
    digitalWriteFast(PIN_PYRO_1, HIGH);
    delay(500);
    digitalWriteFast(PIN_PYRO_1, LOW);
}

void setup() {
    Serial.begin(BAUD);
    initPins();
    rgb.begin();
    initRadio();
    if (!error) {
        rgb.setPixelColor(0, OK_COLOR);
        rgb.show();
    } else {
        rgb.setPixelColor(0, ERROR_COLOR);
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
        radio_protocol.parse_bytes(radio_buf, radio_len);
    }
}