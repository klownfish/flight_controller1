#include "AsyncFlash.h"
#include "RH_RF69.h"
#include "MPU9250.h"
#include "rocket.h"
#include "definitions.h"
#include "WS2812Serial.h"
#include "Gps.h"
#include "Estimator2.h"
#include "async_BMI088.h"
#include "async_BMP280.h"
#include "Pid.h"
#include "Servo.h"
#include "rocket_defines.h"
#include "i2c_driver.h"
#include "TsyDMASPI.h"
#include "TeensyThreads.h"
#include "imx_rt1060/imx_rt1060_i2c_driver.h"

//#define SERIAL_TELEMETRY

#define TVC_MAX_ANGLE_X 8
#define TVC_MAX_ANGLE_Y 8

void enterState(rocket::state state);

extern Threads::Mutex i2c_mtx;
extern Threads::Mutex spi_mtx;
extern Threads::Mutex usb_mtx;

extern Bmi088Accel bmiAcc;
extern Bmi088Gyro bmiGyro;
extern RH_RF69 radio; 
extern AsyncFlash flash;
extern BMP280 bmp;
extern MPU9250 mpu;
extern WS2812Serial rgb;
extern GPS gps;
extern Estimator estimator;
extern Pid x_gimbal_pid;
extern Pid y_gimbal_pid;
extern Servo x_servo;
extern Servo y_servo;
extern float x_gimbal_angle;
extern float y_gimbal_angle;

extern uint32_t flash_addr;
extern bool error;
extern rocket::state rocket_state;
extern bool flash_enabled;
extern bool telemetry_enabled;