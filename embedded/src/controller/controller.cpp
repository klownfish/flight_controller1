#include <Arduino.h>
#include <SPI.h>
#include <RH_RF69.h>
#include <Adafruit_BMP280.h>
#include <elapsedMillis.h>
#include <WS2812Serial.h>
#include <Adafruit_NeoPixel.h>

#include "definitions.h" // pin and protocol definitions
#include "controller.h"
#include "rocket.h"
#include "protocol.h"
#include "Sampler.h"

//haha lolz
void dance();

#define SERIAL_DEBUG

#define BMI_GYRO_FREQUENCY 1000
#define BMI_ACCEL_FREQUENCY 250
#define BMP_FREQUENCY 50

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

Threads::Mutex i2c_mtx;
Threads::Mutex spi_mtx;
Threads::Mutex usb_mtx;

byte drawingMemory[3];         //  3 bytes per LED
DMAMEM byte displayMemory[12]; // 12 bytes per LED
WS2812Serial rgb(1, displayMemory, drawingMemory, PIN_RGB_TX, WS2812_RGB);
Bmi088Accel bmiAcc {Master, 0x19};
Bmi088Gyro bmiGyro {Master, 0x69};
RH_RF69 radio {PIN_RF_CS, PIN_RF_G0}; 
AsyncFlash flash {&SPI, PIN_FLASH_CS, 10000000};
MPU9250 mpu;
BMP280 bmp {Master};
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

void threadEnd(float frequency) {
    static uint32_t start_time = micros(); 
    uint32_t end_time = micros();
    uint32_t delay = 1.0 / frequency * 1000000;
    uint32_t time_taken = end_time - start_time;

    #ifdef SERIAL_DEBUG
        if (time_taken > delay) {
            Serial.println("a thread is impossibly slow");
        }
    #endif

    threads.delay_us(delay - time_taken);
    start_time = micros();
    #ifdef SERIAL_DEBUG
        if (start_time - end_time > delay * 2) {
            Serial.println("took too long for a thread to start");
        }
    #endif
}

void enterState(rocket::state state) {
    rocket_state = state;

    switch(rocket_state) {
        case rocket::state::debug:
            break;

        case rocket::state::sleeping:
            break;

        case rocket::state::ready:
            estimator.set_stationary(0, 90 * DEG_TO_RAD, 0);
            break;

        case rocket::state::powered_flight:
            break;

        case rocket::state::passive_flight:
            break;

        case rocket::state::falling:
            break;

        case rocket::state::landed:
            break;
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
        flash.read(flash_addr, &byte, 1);
        if (byte == 0xff) {
            empty_in_row += 1;
        } else {
            empty_in_row = 0;
        }
        if (empty_in_row == FLASH_TEST_BYTES) {
            flash_addr -= FLASH_TEST_BYTES - 2;
            break;
        }
        flash_addr++;
    }
    Serial.print("restoring flash to: ");
    Serial.println(flash_addr);
}

void initFlash() {   
    if (!flash.begin(0xEF6018)) {
        Serial.println("Could not init flash chip");
        error = true;
        return;
    }
    uint8_t byte = 1;
    flash.write(0, &byte, 1);
    restoreFlashAddr();
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
    uint8_t ret = 0;
    ret |= bmiAcc.begin();
    ret |= bmiGyro.begin();
    if (ret != 1) {
        Serial.println("could not ini BMI");
        error = true;
        return;
    }
    bmiAcc.setOdr(Bmi088Accel::Odr::ODR_400HZ_BW_145HZ);
    bmiGyro.setOdr(Bmi088Gyro::Odr::ODR_2000HZ_BW_230HZ);
}

void initBmp() {
    if (!bmp.begin(0x76)) {
        Serial.println("could not initialize BMP");
        error = true;
        return;
    }
}

void sampleGpsPosition() {
    rocket::gps_pos_from_rocket_to_ground msg;
    if (gps.is_set(FLAG_ALTITUDE) && gps.is_set(FLAG_LONGITUDE) && gps.is_set(FLAG_LATITUDE)) {
        msg.set_altitude(gps.altitude);
        msg.set_longitude(gps.longitude_degrees + gps.longitude_minutes / 60);
        msg.set_latitude(gps.latitude_degrees + gps.latitude_minutes / 60);
        sendMsg(&msg, 3);
    }
}

void sampleGpsState() {
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
    sendMsg(&msg, 10);
}

void sampleBmiAccel() {
    for (;;) {
        i2c_mtx.lock();
        bmiAcc.readSensor();
        i2c_mtx.unlock();
        float ax = bmiAcc.getAccelX_mss();
        float ay = bmiAcc.getAccelY_mss();
        float az = bmiAcc.getAccelZ_mss();
        Serial.println("a");
        threadEnd(BMI_ACCEL_FREQUENCY);
    }
}

void sampleBmiGyro() {
    for (;;) {
        i2c_mtx.lock();
        bmiGyro.readSensor();
        i2c_mtx.unlock();
        float gx = bmiGyro.getGyroX_rads();
        float gy = bmiGyro.getGyroY_rads();
        float gz = bmiGyro.getGyroZ_rads();
        Serial.println("g");
        threadEnd(BMI_GYRO_FREQUENCY);
    }
}

void sampleBmp() {
    for (;;) {
        i2c_mtx.lock();
        uint32_t wait = bmp.startMeasurment();
        i2c_mtx.unlock();
        threads.delay(wait);
        i2c_mtx.lock();
        double temp, pressure;
        bmp.getTemperatureAndPressure(temp, pressure);
        i2c_mtx.unlock();
        Serial.println("pressure");
        Serial.println(pressure);
        threadEnd(BMP_FREQUENCY);
    }
}

void sampleBatteryVoltage() {
    rocket::battery_voltage_from_rocket_to_ground battery_msg;
    float volt = (float) analogRead(PIN_BAT_READ);
    volt = volt / ((1 << 12) - 1) * 3.3 * (6.04e3 + 2e3) / (2e3);   
    battery_msg.set_voltage(volt);
    sendMsg(&battery_msg, 1);
}

void sampleFlashMemory() {
    rocket::flash_address_from_rocket_to_ground flash_msg;
    flash_msg.set_address(flash_addr);
    sendMsg(&flash_msg, 1);
}

void sampleState() {
    rocket::state_from_rocket_to_ground state_msg;
    state_msg.set_state(rocket_state);
    sendMsg(&state_msg, 1);
}

void sampleTime() {
    rocket::timestamp_from_rocket_to_ground msg;
    msg.set_ms_since_boot(millis());
    sendMsg(&msg, 1);
}

void sampleEstimate() {
    rocket::estimate_from_rocket_to_ground msg;
}
void updateTvc() {
    for (;;) {
        if (millis() > 10000) {
            estimator.set_moving();
        } else {
            estimator.set_stationary(0, 0, 0);
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
        threads.yield();
    }
}

void initThreads() {
    threads.setMicroTimer();
    threads.addThread(sampleBmp);
}

void setup() {
    //setup doesn't have to be optimized, lock everything
    i2c_mtx.lock();
    spi_mtx.lock();
    usb_mtx.lock();
    Serial.begin(BAUD);
    while(!Serial){}
    initPins();
    enterState(rocket::state::ready);
    sampler.setClockDivider(1);
    rgb.begin();
    rgb.setPixel(0, 0);
    rgb.show();
    Master.begin(400000);
    initFlash();
    initBmp();
    //initRadio();
    initThreads();
    #ifdef USE_GPS
        initGps();
        sampler.insertFunction(sampleGpsPosition, 10);
        sampler.insertFunction(sampleGpsState, 10);
    #else
        initBmi();
        digitalWrite(PIN_PYRO_4, HIGH);
        x_gimbal_pid.set_setpoint(0);
        y_gimbal_pid.set_setpoint(0);
        threads.addThread(updateTvc);
        threads.addThread(sampleBmiAccel);
        threads.addThread(sampleBmiGyro);
        initServos();
    #endif
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
    i2c_mtx.unlock();
    spi_mtx.unlock();
    usb_mtx.unlock();
}

void loop() {
    handleDataStreams();
    estimator.update(micros());
    
    threads.yield();
}