#include <Arduino.h>
#include <SPI.h>
#include <Wire.h> // i2c
#include <SerialFlash.h> // for flash chip
#include <SPIMemory.h>
#include <RH_RF69.h>
#include <Adafruit_BMP280.h>
#include <elapsedMillis.h>
#include <WS2812Serial.h>
#include <Adafruit_NeoPixel.h>

#include "MPU9250.h"
#include "definitions.h" // pin and protocol definitions
#include "controller.h"
#include "rocket.h"
#include "protocol.h"
#include "Estimator.h"
#include "Sampler.h"

//haha lolz
void dance();

#define FLASH_TEST_BYTES 10

#define GPS_BAUD 9600

byte drawingMemory[3];         //  3 bytes per LED
DMAMEM byte displayMemory[12]; // 12 bytes per LED
WS2812Serial rgb(1, displayMemory, drawingMemory, PIN_RGB_TX, WS2812_RGB);

RH_RF69 radio {PIN_RF_CS, PIN_RF_G0}; 
SPIFlash flash {PIN_FLASH_CS};
Adafruit_BMP280 bmp;
MPU9250 mpu;
GPS gps;

Estimator estimator;

Sampler sampler;

uint32_t flash_addr = 0;
bool error = false;
bool flash_enabled = false;
rocket::state rocket_state = rocket::state::sleeping;

uint16_t stateToTimeDiv(enum rocket::state state) {
    switch (state) {
        case rocket::state::sleeping:
            return 10000;
            break;
        case rocket::state::awake:
            return 20;
            break;
        
        case rocket::state::ready:
        case rocket::state::debug:
        case rocket::state::falling:
        case rocket::state::powered_flight:
        case rocket::state::passive_flight:
        case rocket::state::landed:
            return 1;
            break;

        default:
            return 10000;
    }
}

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
    Wire.begin();
    Wire.setClock(400000);
    analogReadResolution(12);

    analogWriteFrequency(PIN_BUZZER, BUZZER_HZ);
    digitalWrite(PIN_RF_CS, HIGH);
    digitalWrite(PIN_PYRO_4, LOW);
    digitalWrite(PIN_FLASH_CS, HIGH);
    digitalWrite(PIN_PYRO_ARM, HIGH);
    delay(100);
}

void restoreFlashAddr() {
    uint8_t empty_in_row = 0;
    uint8_t byte = 0;
    while (true) {
        if ((byte = flash.readByte(flash_addr)) == 0xff) {
            empty_in_row += 1;
        } else {
            empty_in_row = 0;
        }
        if (empty_in_row == FLASH_TEST_BYTES) {
            flash_addr -= FLASH_TEST_BYTES - 1;
            break;
        }
        flash_addr++;        
    }
}

void initFlash() {   
    flash.setClock(104000000 / 2);
    if (!flash.begin(MB(16))) {
        Serial.println("Could not init flash chip");
        error = true;
        return;
    }
    for (uint8_t i = 0; i < FLASH_TEST_BYTES; i++) {
        if (flash.readByte(i) != 0xff) {
            restoreFlashAddr();
            Serial.print("restoring to addres: ");
            Serial.println(flash_addr);
            return;
        }
    }
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

void initMpu() {
    MPU9250Setting settings;
    settings.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_125HZ;
    settings.gyro_fs_sel = GYRO_FS_SEL::G250DPS;
    settings.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_10HZ;
    settings.accel_fs_sel = ACCEL_FS_SEL::A4G;
    settings.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_10HZ;
    mpu.selectFilter(QuatFilterSel::NONE);
    mpu.ahrs(false);

    if (!mpu.setup(0x68, settings, Wire)) {
        Serial.println("could not initialize MPU");
        error = true;
        return;
    }
}

void initBmp() {
    if (!bmp.begin(0x76)) {
        Serial.println("could not initialize BMP");
        error = true;
    }
}

void sampleGpsPosition(void*) {
    rocket::gps_pos_from_rocket_to_ground msg;
    if (gps.is_set(FLAG_ALTITUDE) && gps.is_set(FLAG_LONGITUDE) && gps.is_set(FLAG_LATITUDE)) {
        msg.set_altitude(gps.altitude);
        msg.set_longitude(gps.longitude_degrees + gps.longitude_minutes / 60);
        msg.set_latitude(gps.latitude_degrees + gps.latitude_minutes / 60);
        sendMsg(&msg, send_when::REGULAR);
    }
}

void sampleGpsState(void*) {
    rocket::gps_state_from_rocket_to_ground msg;
    if (gps.is_set(FLAG_FIX_STATUS)) {
        switch (gps.fix_status) {
            case 1:
                msg.set_fix_type(rocket::fix_type::none);
                break;
            case 2:
                msg.set_fix_type(rocket::fix_type::fix2D);
                break;
            case 3:
                msg.set_fix_type(rocket::fix_type::fix3D);
                break;
            default:
                msg.set_fix_type(rocket::fix_type::none);
                break;
        }
    } else {
        msg.set_fix_type(rocket::fix_type::none);
    }

    if (gps.is_set(FLAG_PDOP)) {
        msg.set_pdop(gps.pdop);
    } else {
        msg.set_pdop(0);
    }

    if (gps.is_set(FLAG_N_SATELLITES)) {
        msg.set_n_satellites(gps.n_satellites);
    } else {
        msg.set_n_satellites(0);
    }

    sendMsg(&msg, send_when::REGULAR);
}

void sampleMpu(void*) {
    rocket::mpu_from_rocket_to_ground msg;

    static uint32_t last_update = micros();
    if (!mpu.update()) return;
    float ax = mpu.getAccX();
    float ay = mpu.getAccY();
    float az = mpu.getAccZ();
    float gx = mpu.getGyroX();
    float gy = mpu.getGyroY();
    float gz = mpu.getGyroZ();

    estimator.update_imu(ax, ay, az, gx, gy, gz, (micros() - last_update) / 1000000);
    msg.set_acc_x(ax);
    msg.set_acc_y(ay);
    msg.set_acc_z(az);
    msg.set_gyro_x(gx);
    msg.set_gyro_y(gy);
    msg.set_gyro_z(gz);
    last_update = micros();
    sendMsg(&msg, REGULAR);
}

void sampleBmp(void*) {
    static uint32_t last_update = micros();
    rocket::bmp_from_rocket_to_ground bmp_msg;
    bmp_msg.set_pressure(bmp.readPressure());
    bmp_msg.set_temperature(bmp.readTemperature());
    estimator.update_altitude(bmp.readAltitude(), 0);
    sendMsg(&bmp_msg, REGULAR);
}

void sampleBatteryVoltage(void*) {
    rocket::battery_voltage_from_rocket_to_ground battery_msg;
    float volt = (float) analogRead(PIN_BAT_READ);
    volt = volt / ((1 << 12) - 1) * 3.3 * (6.04e3 + 2e3) / (2e3);   
    battery_msg.set_voltage(volt);
    sendMsg(&battery_msg, REGULAR);
}

void sampleFlashMemory(void*) {
    rocket::flash_address_from_rocket_to_ground flash_msg;
    flash_msg.set_address(flash_addr);
    sendMsg(&flash_msg, REGULAR);
}

void sampleState(void*) {
    rocket::state_from_rocket_to_ground state_msg;
    state_msg.set_state(rocket_state);
    sendMsg(&state_msg, REGULAR);
}

void sampleTime(void*) {
    rocket::timestamp_from_rocket_to_ground msg;
    msg.set_ms_since_boot(millis());
    sendMsg(&msg, REGULAR);
}

void sampleEstimate(void*) {
    rocket::estimate_from_rocket_to_ground msg;
    msg.set_altitude(estimator.get_altitude());
    msg.set_ax(estimator.get_ax());
    msg.set_ay(estimator.get_ay());
    msg.set_az(estimator.get_az());
    msg.set_gx(estimator.get_gx());
    msg.set_gy(estimator.get_gy());
    msg.set_gz(estimator.get_gz());
    msg.set_hx(estimator.get_hx());
    msg.set_hy(estimator.get_hy());
    msg.set_hz(estimator.get_hz());
    sendMsg(&msg, REGULAR);
}

void initSampler() {
    sampler.insertFunction(sampleTime, 50);
    sampler.insertFunction(sampleState, 1);
    sampler.insertFunction(sampleMpu, 30);
    sampler.insertFunction(sampleBmp, 50);
    sampler.insertFunction(sampleFlashMemory, 1);
    sampler.insertFunction(sampleBatteryVoltage, 1);
    sampler.insertFunction(sampleEstimate, 20);
    sampler.insertFunction(sampleGpsState, 3);
    sampler.insertFunction(sampleGpsPosition, 3);
}

void setup() {
    Serial.begin(BAUD);
    Serial3.begin(GPS_BAUD);
    initPins();
    rgb.begin();

    initFlash();
    initMpu();
    initBmp();
    initRadio();
    initSampler();
    if (error) {
        // not good
        rgb.setPixel(0, RED);
        rgb.show();
    } else {
        // the all good song
        // dance();
        rgb.setPixel(0, GREEN);
        rgb.show();
    }
}

void loop() {
    handleDataStreams();
    static uint32_t last_update = micros();
    uint32_t dt = micros() - last_update;
    sampler.setClockDivider(stateToTimeDiv(rocket_state));
    sampler.update(dt);
    last_update = micros();
}