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
//#include "Estimator2.h"
#include "Sampler.h"

//haha lolz
void dance();

#define FLASH_TEST_BYTES 10

#define GPS_BAUD 9600

#define SERVO_MIN_PULSE 750
#define SERVO_MAX_PULSE 2250
#define SERVO_ACTUATION (55 * DEG_TO_RAD)

//#define USE_GPS
#define USE_SERVO

#ifdef USE_GPS
#ifdef USE_SERVO
#error "Both servo and gps in use"
#endif
#endif

#define KP 0.5 
#define KI 0.05
#define KD 0.2

byte drawingMemory[3];         //  3 bytes per LED
DMAMEM byte displayMemory[12]; // 12 bytes per LED
WS2812Serial rgb(1, displayMemory, drawingMemory, PIN_RGB_TX, WS2812_RGB);
Bmi088 bmi {Wire, 0x19, 0x69};
RH_RF69 radio {PIN_RF_CS, PIN_RF_G0}; 
SPIFlash flash {PIN_FLASH_CS};
Adafruit_BMP280 bmp;
MPU9250 mpu;
GPS gps;
Servo x_servo;
Servo y_servo;
Pid x_gimbal_pid {KP, KI, KD};
Pid y_gimbal_pid {KP, KI, KD}; //output is rad*s^-2

Estimator estimator;

Sampler sampler;

uint32_t flash_addr = 0;
bool error = false;
bool flash_enabled = false;
bool telemetry_enabled = false;
rocket::state rocket_state = rocket::state::ready;

void enterState(rocket::state state) {
    rocket_state = state;

    switch(rocket_state) {
        case rocket::state::debug:
            sampler.setClockDivider(1);
            relay_frequency = 1;
            break;

        case rocket::state::sleeping:
            sampler.setClockDivider(~0);
            relay_frequency = 1;
            break;

        case rocket::state::awake:
            sampler.setClockDivider(10);
            relay_frequency = 1;
            break;

        case rocket::state::ready:
            sampler.setClockDivider(2);
            estimator.set_stationary(0, 90 * DEG_TO_RAD, 0);
            relay_frequency = 5;
            break;

        case rocket::state::powered_flight:
            sampler.setClockDivider(1);
            relay_frequency = 10;
            break;

        case rocket::state::passive_flight:
            sampler.setClockDivider(2);
            relay_frequency = 5;
            break;

        case rocket::state::falling:
            sampler.setClockDivider(5);
            relay_frequency = 1;
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

void initServos() {
    x_servo.attach(PIN_EXT_RX, 750, 2250);
    y_servo.attach(PIN_EXT_TX, 750, 2250);
}

void initGps() {
    Serial3.begin(GPS_BAUD);
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

void initBmi() {
    if (!bmi.begin()) {
        Serial.println("could not initialize BMI");
        error = true;
        return;
    }
    bmi.setOdr(bmi.ODR_400HZ);
}

void initBmp() {
    if (!bmp.begin(0x76)) {
        Serial.println("could not initialize BMP");
        error = true;
        return;
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
    float ax = mpu.getAccX();
    float ay = mpu.getAccY();
    float az = mpu.getAccZ();
    float gx = mpu.getGyroX();
    float gy = mpu.getGyroY();
    float gz = mpu.getGyroZ();
    //estimator.insert_imu(ax, ay, az, gx, gy, gz, millis()); should be radians not degrees
    msg.set_ax(ax);
    msg.set_ay(ay);
    msg.set_az(az);
    msg.set_gx(gx);
    msg.set_gy(gy);
    msg.set_gz(gz);
    sendMsg(&msg, REGULAR);
}

void sampleBmi(void*) {
    rocket::bmi_from_rocket_to_ground msg;
    bmi.readSensor();
    float ax = bmi.getAccelX_mss();
    float ay = bmi.getAccelY_mss();
    float az = bmi.getAccelZ_mss();
    float gx = bmi.getGyroX_rads();
    float gy = bmi.getGyroY_rads();
    float gz = bmi.getGyroZ_rads();
    estimator.insert_imu(ax, ay, az, gx, gy, gz, millis());
    msg.set_ax(ax);
    msg.set_ay(ay);
    msg.set_az(az);
    msg.set_gx(gx);
    msg.set_gy(gy);
    msg.set_gz(gz);
    sendMsg(&msg, REGULAR);
}

void sampleBmp(void*) {
    static uint32_t last_update = micros();
    rocket::bmp_from_rocket_to_ground bmp_msg;
    bmp_msg.set_pressure(bmp.readPressure());
    bmp_msg.set_temperature(bmp.readTemperature());
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
}

void updateTvc(void*) {
    Serial.println("-----");
    if (millis() > 10000) {
        estimator.set_moving();
    } else {
        estimator.set_stationary(0, 0, 0);
        Serial.println("stationary");
    }
    static uint32_t last_update = micros();
    uint32_t current_time = micros();
    float dt = (current_time - last_update) / 1000000;
    turbomath::Quaternion heading_quat = estimator.get_heading();
    float roll, pitch, yaw;
    heading_quat.get_RPY(&roll, &pitch, & yaw);
    float x_angular_accel = x_gimbal_pid.update(roll, dt);
    float y_angular_accel = y_gimbal_pid.update(pitch, dt); //output is radians*s^-2

    float total_thrust = estimator.get_local_acc().z / ROCKET_MASS;
    float moment_arm = ROCKET_MOUNT - ROCKET_COM;
    float x_thrust =  x_angular_accel * ROCKET_MOI / moment_arm;
    float y_thrust = y_angular_accel * ROCKET_MOI / moment_arm;;
    
    float x_angle = asinf(x_thrust / total_thrust);
    float y_angle = asinf(y_thrust / total_thrust);

    float mount_x_angle = cosf(yaw) * x_angle - sinf(yaw) * y_angle;
    float mount_y_angle = sinf(yaw) * x_angle + cosf(yaw) * y_angle;

    float servo_x_angle = mount_x_angle;
    float servo_y_angle = mount_y_angle;

    servo_x_angle = SERVO_ACTUATION / 2 + constrain(servo_x_angle, -SERVO_ACTUATION / 2, SERVO_ACTUATION / 2);
    servo_y_angle = SERVO_ACTUATION / 2 + constrain(servo_y_angle, -SERVO_ACTUATION / 2, SERVO_ACTUATION / 2);

    x_servo.writeMicroseconds(SERVO_MIN_PULSE + (SERVO_MAX_PULSE - SERVO_MIN_PULSE) * servo_x_angle / SERVO_ACTUATION);
    y_servo.writeMicroseconds(SERVO_MIN_PULSE + (SERVO_MAX_PULSE - SERVO_MIN_PULSE) * servo_y_angle / SERVO_ACTUATION);
}

void initSampler() {
    #ifdef USE_GPS
        sampler.insertFunction(sampleGpsState, 3);
        sampler.insertFunction(sampleGpsPosition, 3);    
    #else
        sampler.insertFunction(updateTvc, 100);
    #endif

    sampler.insertFunction(sampleTime, 20);
    sampler.insertFunction(sampleState, 1);
    sampler.insertFunction(sampleMpu, 50);
    sampler.insertFunction(sampleBmi, 20);
    sampler.insertFunction(sampleBmp, 20);
    sampler.insertFunction(sampleFlashMemory, 1);
    sampler.insertFunction(sampleBatteryVoltage, 1);
    sampler.insertFunction(sampleEstimate, 20);
}

void setup() {
    Serial.begin(BAUD);
    initPins();
    rgb.begin();
    Wire.setClock(400000);
    initFlash();
    initMpu();
    initBmp();
    initBmi();
    initRadio();
    initSampler();
    if (error) {
        // not good
        rgb.setPixel(0, ERROR_COLOR);
        rgb.show();
    } else {
        // the all good song
        // dance();
        rgb.setPixel(0, OK_COLOR);
        rgb.show();
    }

    #ifdef USE_GPS
        initGps();
    #else
        digitalWrite(PIN_PYRO_4, HIGH);
        x_gimbal_pid.set_setpoint(0);
        y_gimbal_pid.set_setpoint(0);
        initServos();
    #endif
}
S
void loop() {
    handleDataStreams();
    static uint32_t last_update = micros();
    uint32_t dt = micros() - last_update;
    sampler.update(dt);
    estimator.update(micros());
    last_update = micros();
}