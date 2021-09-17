#include "SPIFlash.h"
#include "RH_RF69.h"
#include "Adafruit_BMP280.h"
#include "MPU9250.h"
#include "rocket.h"
#include "definitions.h"
#include "WS2812Serial.h"
#include "Gps.h"
#include "Estimator.h"

#define RED    0x160000
#define GREEN  0x001600
#define BLUE   0x000016
#define YELLOW 0x101400
#define PINK   0x120009
#define ORANGE 0x100400
#define WHITE  0x101010

extern RH_RF69 radio; 
extern SPIFlash flash;
extern Adafruit_BMP280 bmp;
extern MPU9250 mpu;
extern WS2812Serial rgb;
extern GPS gps;
extern Estimator estimator;

extern uint32_t flash_addr;
extern bool error;
extern rocket::state rocket_state;
extern bool flash_enabled;