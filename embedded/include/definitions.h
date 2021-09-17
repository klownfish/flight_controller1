#if __has_include("callsign.h")
#   include "callsign.h"
#   define TX_POWER 20
#   define IS_HAM true
#else
#   define CALLSIGN ""
#   define TX_POWER 13
#   define IS_HAM false
#endif

#define RADIO_BUF_LEN 128

#define FREQUENCY 433.800
#define MODULATION RH_RF69::ModemConfigChoice::GFSK_Rb38_4Fd76_8

#define PIN_CAN_RX 0
#define PIN_CAN_RX 1
#define PIN_RF_G0 3
#define PIN_RF_RST 4
#define PIN_RF_CS 5
#define PIN_PYRO_4 6
#define PIN_PYRO_3 7
#define PIN_PYRO_2 8
#define PIN_PYRO_1 9
#define PIN_FLASH_CS 10
#define PIN_MOSI 11
#define PIN_MISO 12
#define PIN_SCK 13
#define PIN_EXT_RX 14
#define PIN_EXT_TX 15
#define PIN_PYRO_ARM 16
#define PIN_BAT_READ 17
#define PIN_SDA 18
#define PIN_SDL 19
#define PIN_RGB_TX 20
#define PIN_LIFTOFF 22
#define PIN_BUZZER 23

#define RED    0x160000
#define GREEN  0x001600
#define BLUE   0x000016
#define YELLOW 0x101400
#define PINK   0x120009
#define ORANGE 0x100400
#define WHITE  0x101010

#define BAUD 115200

#define BUZZER_HZ 2700