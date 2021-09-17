#include <stdint.h>

#define KNOT_TO_MS 0.51444 

#define FIELD_BUF_LEN 20

#define FLAG_LATITUDE (1 << 0)
#define FLAG_LONGITUDE (1 << 1)
#define FLAG_TIME (1 << 2)
#define FLAG_N_SATELLITES (1 << 3)
#define FLAG_QUALITY (1 << 4)
#define FLAG_HDOP (1 << 5)
#define FLAG_ERROR_MESSAGE (1 << 6)
#define FLAG_POS_MODE (1 << 7)
#define FLAG_ALTITUDE (1 << 8)
#define FLAG_GEOID_SEPARATION (1 << 9)
#define FLAG_DIFF_AGE (1 << 10)
#define FLAG_DIFF_STATION (1 << 11)
#define FLAG_SPEED (1 << 12)
#define FLAG_COURSE_OVER_GROUND (1 << 13)
#define FLAG_DATE (1 << 14)
#define FLAG_MAGNETIC_VARIATION (1 << 15)
#define FLAG_POS_MODE_SINGLE (1 << 16)
#define FLAG_OP_MODE (1 << 17)
#define FLAG_FIX_STATUS (1 << 18)
#define FLAG_PDOP (1 << 19)
#define FLAG_VDOP (1 << 20)

#define NS_COMBINATION 'N'
#define NS_GPS 'P'
#define NS_GALILEO 'A'
#define NS_BEIDOU 'B'
#define NS_GLONASS 'L'
#define NS_QZSS 'Q'

#define POS_MODE_NO_POSITION 'N'
#define POS_MODE_ESTIMATE 'E'
#define POS_MODE_RTK_FLOAT 'F'
#define POS_MODE_RTK_FIXED 'R'
#define POS_MODE_AUTONOMOUS 'A'
#define POS_MODE_DIFFERENTIAL 'D'

class GPS {
public:
  void clear_flags() {flags = 0;}
  bool is_set(uint32_t flag) {return flags & flag;}

  bool parse_message(uint8_t message[], uint8_t buf_len);
  bool parse_byte(uint8_t byte);

  int32_t latitude_degrees; //north positive
  float latitude_minutes;
  int32_t longitude_degrees; //east positive
  float longitude_minutes;
  float altitude;
  float geoid_separation;

  float speed_knot;
  float speed_ms;
  float course_over_ground;

  uint32_t raw_time;
  uint8_t hours;
  uint8_t minutes;
  uint8_t seconds;
  uint16_t milliseconds;
  uint8_t ms_precision;
  uint8_t day;
  uint8_t month;
  uint8_t year;

  uint8_t fix_status; // multi gnss

  uint8_t op_mode; // multi gnss

  uint8_t gps_pos_mode;
  uint8_t glonass_pos_mode;
  uint8_t galileo_pos_mode;
  uint8_t beidou_pos_mode;
  uint8_t pos_mode; //pos_mode from the last message. can be gps, galileo, beidou anything!

  float hdop;
  float pdop;
  float vdop;
  uint8_t n_satellites;
  uint8_t diff_age;
  uint8_t diff_station;

  float magnetic_variation;

private:
  uint32_t flags = 0;
  void set(uint32_t flag) {flags |= flag;}
  void clear(uint32_t flag) {flags &= !flag;}

  template <class T>
  bool parse_len_to_uint(T* whole, uint8_t message[], uint8_t* index, uint8_t len);
  template <class T>
  bool parse_field_to_uint(T* whole, uint8_t message[], uint8_t* index);
  bool parse_field_to_float(float* num, uint8_t message[], uint8_t* index);

  void parse_gns(uint8_t message[]);
  void parse_rmc(uint8_t message[]);
  void parse_gsa(uint8_t message[]);

  void to_hex(uint8_t num, uint8_t buf[]);
  void generate_checksum(uint8_t message[], uint8_t buf[]);
  bool verify_message(uint8_t message[], uint8_t buf_len);
  void send_to_GPS(uint8_t message[]);

  uint8_t message_buf[100];
  uint8_t message_buf_index = 0;

  uint8_t constellation;
};