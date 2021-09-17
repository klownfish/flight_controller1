#include <Arduino.h>
#include "WS2812Serial.h"
#include "elapsedMillis.h"
#include "RH_RF69.h"
#include "DataProtocol.h"
#include "definitions.h"
#include "rocket.h"

void DataProtocolCallback(uint8_t id, uint8_t* buf, uint8_t len);

DataProtocol usb_protocol {DataProtocolCallback};
DataProtocol radio_protocol {DataProtocolCallback};
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
    if (IS_HAM) { 
        radio.send(CALLSIGN, sizeof(CALLSIGN));
    }
}


void setup() {
    Serial.begin(BAUD);
    initPins();
    initRadio();
    if (!error) {
        rgb.setPixel(0, BLUE);
        rgb.show();
    } else {
        rgb.setPixel(0, YELLOW);
        rgb.show();
    }
}

void loop() {
    radio.mod
}