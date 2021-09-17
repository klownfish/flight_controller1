#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <Arduino.h>

#include "Gps.h"


// parses a field to an unsigned integer
// the index will be pointing on the character after ','
template <class T>
bool GPS::parse_field_to_uint(T* result, uint8_t message[], uint8_t* msg_index) {
  uint8_t buf[FIELD_BUF_LEN];
  uint8_t buf_index = 0;
  while (buf_index < FIELD_BUF_LEN) {
    if (message[*msg_index] < '0' || message[*msg_index] > '9') {
      break;
    } 
    buf[buf_index++] = message[(*msg_index)++];
  }
  (*msg_index)++; //skip the ','

  //empty field
  if (buf_index == 0) {
    *result = 0;
    return false;
  }
  buf[++buf_index] = '\0'; //terminate string
  *result = atoi(buf);
  return true;
}

// parses X amount of bytes to an unsigned integer
// the index will be pointing after the last parsed byte
template <class T>
bool GPS::parse_len_to_uint(T* result, uint8_t message[], uint8_t* index, uint8_t len) {
  *result = 0;
  
  for (uint8_t i = 0; i < len; i++) {
    *result *= 10;
    *result += message[(*index)++] - '0';
  }
  return true;
}

// parses a field to a float
// the index will be pointing on the character after ','
bool GPS::parse_field_to_float(float* result, uint8_t message[], uint8_t* msg_index) {
  uint8_t buf[FIELD_BUF_LEN];
  uint8_t buf_index = 0;
  
  while (buf_index < FIELD_BUF_LEN) {
    if ((message[*msg_index] < '0' || message[*msg_index] > '9') && 
        message[*msg_index] != '.') {
      break;
    }
    buf[buf_index++] = message[(*msg_index)++];
  }
  (*msg_index)++; //skip ,

  //empty field
  if (buf_index == 0) {
    return false;
  }
  buf[buf_index] = '\0'; //terminate string
  *result = atof((char*) buf);
  return true; 
}

//generates a checksum for a message to a 2 long buffer
void GPS::generate_checksum(uint8_t message[], uint8_t buf[]) {
  uint8_t index = 1; //1 to skip $
  uint8_t checksum = 0;
  while (true) {
    checksum ^= message[index];
    index++;
    if (message[index] == '*') {
      break;
    }
  }
  to_hex(checksum, buf);
}


// verify if a gps message is valid,
bool GPS::verify_message(uint8_t message[], uint8_t msg_len) {
  uint8_t checksum[2];
  uint8_t index = 0;
  while (true) {
    index++;
    if (message[index] == '*') {
      break; 
    }
    //if there is no checksum it't can be wrong!
    if (message[index] == '\r') {
      return true;
    }
    if (index == msg_len) {
      return false;
    }
  }
  generate_checksum(message, checksum);
  return memcmp(checksum, &message[++index], 2) == 0;
}

// generate the hex value of a num in ascii
void GPS::to_hex(uint8_t num, uint8_t buf[]) {
  const char* chars = "0123456789ABCDEF";
  buf[1] = chars[num & 0x0F];
  buf[0] = chars[(num & 0xF0) >> 4];
}


// parse generic message
bool GPS::parse_message(uint8_t message[], uint8_t buf_len) {
  if (!verify_message(message, buf_len)) {
    set(FLAG_ERROR_MESSAGE);
    return false;
  }

  //switch on talker id
  constellation = message[2];

  //only care about the multi constellation messages for now
  if (constellation != NS_COMBINATION) {
    return true;
  }
  
  //switch on message type
  uint8_t* type_pos = &message[3];
  
  //GNS is basically GGA 2.0
  if (memcmp(type_pos, "GNS", 3) == 0) {
    parse_gns(type_pos + 4);
  }
  if (memcmp(type_pos, "GSA", 3) == 0) {
    parse_gsa(type_pos + 4);
  } else 
  if (memcmp(type_pos, "RMC", 3) == 0) {
    parse_rmc(type_pos + 4);
  }
  return true;
}

bool GPS::parse_byte(uint8_t byte) {
  message_buf[message_buf_index] = byte;
  message_buf_index++;
  if (byte == '\n') {
    bool result =  parse_message(message_buf, message_buf_index);
    message_buf_index = 0;
    return result;
  }
  return false;
}

// parse the GNS message
void GPS::parse_gns(uint8_t message[]) {
  uint8_t index = 0;
  //decode time
  if (message[index] != ',') {
    set(FLAG_TIME);
    parse_len_to_uint(&hours, message, &index, 2);
    parse_len_to_uint(&minutes, message, &index, 2);
    parse_len_to_uint(&seconds, message, &index, 2);
    index++; //skip '.'
    uint8_t start = index;
    parse_field_to_uint(&milliseconds, message, &index);
    milliseconds *= pow(10,  - index + start);
    ms_precision = start - index;
    raw_time = milliseconds + seconds * 1000 + minutes * 1000 * 100 + hours * 1000 * 100 * 100;
  } else {
    index++;
  } //end time

  //decode latitude
  if (message[index] != ',') {
    set(FLAG_LATITUDE);
    parse_len_to_uint(&latitude_degrees, message, &index, 2);
    parse_field_to_float(&latitude_minutes, message, &index);
  } else {
    index++;
  }

  //NS
  if (message[index] != ',') {
    if (message[index] == 'S') {
      latitude_degrees *= -1;
    }
    index++;
  }
  index++;

  //longitude
  if (message[index] != ',') {
    set(FLAG_LONGITUDE);
    parse_len_to_uint(&longitude_degrees, message, &index, 3);
    parse_field_to_float(&longitude_minutes, message, &index);
  } else {
    index++;
  }

  //EW
  if (message[index] != ',') {
    if (message[index] == 'W') {
      latitude_degrees *= -1;
    }
    index++;
  }
  index++;

  //constellation information
  if (message[index] != ',') {
    set(FLAG_POS_MODE);
    gps_pos_mode = message[index++];
    glonass_pos_mode = message[index++];
    galileo_pos_mode = message[index++];
    beidou_pos_mode = message[index++];
  }
  index++;

  //n_satellites
  if (parse_field_to_uint(&n_satellites, message, &index)) {
    set(FLAG_N_SATELLITES);
  }
  //hdop
  if (parse_field_to_float(&hdop, message, &index)) {
    set(FLAG_HDOP);
  }
  //altitude
  if (parse_field_to_float(&altitude, message, &index)) {
    set(FLAG_ALTITUDE);
  }
  //geoid_separation
  if (parse_field_to_float(&geoid_separation, message, &index)) {
    set(FLAG_GEOID_SEPARATION);
  }
  //diff_age
  if (parse_field_to_uint(&diff_age, message, &index)) {
    set(FLAG_DIFF_AGE);
  } 
  //diff_station
  if (parse_field_to_uint(&diff_station, message, &index)) {
    set(FLAG_DIFF_STATION);
  }
}

// parse the RMC message
void GPS::parse_rmc(uint8_t* message) {
  uint8_t index = 0;

  //decode time
  if (message[index] != ',') {
    set(FLAG_TIME);
    parse_len_to_uint(&hours, message, &index, 2);
    parse_len_to_uint(&minutes, message, &index, 2);
    parse_len_to_uint(&seconds, message, &index, 2);
    index++; //skip '.'
    uint8_t start = index;
    parse_field_to_uint(&milliseconds, message, &index);
    milliseconds *= pow(10, 4 - index + start);
  } else {
    index++;
  } //end time

  //status is just posmode light
  //this is pretty useless
  if (message[index] != ',') {
    index++;
  }
  index++;

  //decode latitude
  if (message[index] != ',') {
    set(FLAG_LATITUDE);
    parse_len_to_uint(&latitude_degrees, message, &index, 2);
    parse_field_to_float(&latitude_minutes, message, &index);
  } else {
    index++;
  }

  //NS
  if (message[index] != ',') {
    if (message[index] == 'S') {
      latitude_degrees *= -1;
    }
    index++;
  }
  index++;

  //longitude
  if (message[index] != ',') {
    set(FLAG_LONGITUDE);
    parse_len_to_uint(&longitude_degrees, message, &index, 3);
    parse_field_to_float(&longitude_minutes, message, &index);
  } else {
    index++;
  }

  //EW
  if (message[index] != ',') {
    if (message[index] == 'W') {
      longitude_degrees *= -1;
    }
    index++;
  }
  index++;

  //speed
  if (parse_field_to_float(&speed_knot, message, &index)) {
    set(FLAG_SPEED);
    speed_ms = speed_knot * KNOT_TO_MS;
  }

  //course over ground
  if (parse_field_to_float(&course_over_ground, message, &index)) {
    set(FLAG_COURSE_OVER_GROUND);
  }

  //date
  if (message[index] != ',') {
    set(FLAG_DATE);
    parse_len_to_uint(&day, message, &index, 2);
    parse_len_to_uint(&month, message, &index, 2);
    parse_len_to_uint(&year, message, &index, 2);
  }
  index++;

  //magnetic variation
  if (parse_field_to_float(&magnetic_variation, message, &index)) {
    set(FLAG_MAGNETIC_VARIATION);
  } 
  if (message[index] != ',') {
    if (message[index] == 'W') {
      magnetic_variation *= -1;
    }
    index++;
  }
  index++;
  
  //posmode, the one from gns is better 
  if (message[index] != ',') {
    set(FLAG_POS_MODE_SINGLE);
    pos_mode = message[index];
  }
}

//Parses relevant parts of the GSA message
void GPS::parse_gsa(uint8_t* message) {
  uint8_t index = 0;
  float waste;

  if(message[index] != ',') {
    set(FLAG_OP_MODE);
    op_mode = message[index];
    index++;
  }
  index++;

  if(parse_field_to_uint(&fix_status, message, &index)) {
    set(FLAG_FIX_STATUS);
  }

  //12 fields of satellite IDs, this is pretty useless informaiton so i throw it away
  for (uint8_t i = 0; i < 12; i++) {
    parse_field_to_float(&waste, message, &index);
  }

  if(parse_field_to_float(&pdop, message, &index)) {
    set(FLAG_PDOP);
  }
  if(parse_field_to_float(&hdop, message, &index)) {
    set(FLAG_HDOP);
  }
  if(parse_field_to_float(&vdop, message, &index)) {
    set(FLAG_VDOP);
  }
}