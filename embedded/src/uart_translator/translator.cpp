#include <Arduino.h>
#include "WS2812Serial.h"
#include "definitions.h"

byte drawingMemory[3];         //  3 bytes per LED
DMAMEM byte displayMemory[12]; // 12 bytes per LED
WS2812Serial rgb(1, displayMemory, drawingMemory, PIN_RGB_TX, WS2812_RGB);

void setup() {
    Serial.begin(9600);
    Serial3.begin(9600);

    pinMode(PIN_SCL, OUTPUT); // this is connected to the bootloader
    digitalWrite(PIN_SCL, HIGH);
    pinMode(PIN_SDA, OUTPUT); // this is connected to the bootloader
    digitalWrite(PIN_SDA, HIGH);
    rgb.begin();
    rgb.setPixel(0, OK_COLOR);
    rgb.show();
}

void loop() {
    Serial3.write(0x7f);
    while (Serial3.available() > 0) {
        Serial.write(Serial3.read());
    }
}