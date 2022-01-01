/*****************************
GENERATED FILE DO NOT EDIT
******************************/

#ifndef _rocket_H
#define _rocket_H
// if you want to use floats or doubles
#define rocket_FLOAT_DEF float

#include <math.h>
#include <stdint.h>
#include <string.h>

namespace rocket {
enum struct state : uint8_t {
  debug = 0,
  sleeping = 1,
  ready = 2,
  powered_flight = 3,
  passive_flight = 4,
  falling = 5,
  landed = 6,
};
enum struct fix_type : uint8_t {
  none = 0,
  fix2D = 1,
  fix3D = 2,
};
enum struct nodes : uint8_t {
  everyone = 0,
  rocket = 1,
  ground = 2,
  flash = 3,
  launchpad = 4,
};
enum struct fields : uint8_t {
  ms_since_boot = 0,
  flash_address = 1,
  ax = 2,
  ay = 3,
  az = 4,
  roll = 5,
  pitch = 6,
  yaw = 7,
  altitude = 8,
  satellites = 9,
  voltage = 10,
  state = 11,
  rssi = 12,
  longitude = 13,
  latitude = 14,
  pressure = 15,
  temperature = 16,
  gx = 17,
  gy = 18,
  gz = 19,
  mx = 20,
  my = 21,
  mz = 22,
  angle_x = 23,
  angle_y = 24,
  pdop = 25,
  n_satellites = 26,
  fix_type = 27,
  armed = 28,
  channel = 29,
  this_to_42 = 30,
  is_enabled = 31,
};
enum struct messages : uint8_t {
  handshake = 0,
  telemetry = 1,
  position = 2,
  bmp = 3,
  mpu = 4,
  bmi_accel = 5,
  bmi_gyro = 6,
  tvc_angle = 7,
  battery_voltage = 8,
  state = 9,
  rssi = 10,
  gps = 11,
  ms_since_boot = 12,
  arm_pyro = 13,
  enable_pyro = 14,
  wipe_flash = 15,
  play_music = 16,
  set_logging = 17,
  dump_flash = 18,
  set_state = 19,
};
enum struct categories : uint8_t {
  none = 0,
};
template <typename T>
void scaledFloat_to_uint(rocket_FLOAT_DEF value, rocket_FLOAT_DEF scale,
                         T *out) {
  *out = value * scale;
}

template <typename T>
void uint_to_scaledFloat(T value, rocket_FLOAT_DEF scale,
                         rocket_FLOAT_DEF *out) {
  *out = value / scale;
}

template <typename T>
void packedFloat_to_uint(rocket_FLOAT_DEF value, rocket_FLOAT_DEF minValue,
                         rocket_FLOAT_DEF maxValue, T *out) {
  T intMax = ~0;
  if (value < minValue) {
    *out = 0;
    return;
  }
  if (value > maxValue) {
    *out = intMax;
    return;
  }
  rocket_FLOAT_DEF ratio = (value - minValue) / (maxValue - minValue);
  *out = 1 + ((intMax - 2)) * ratio;
}

template <typename T>
void uint_to_packedFloat(T value, rocket_FLOAT_DEF minValue,
                         rocket_FLOAT_DEF maxValue, rocket_FLOAT_DEF *out) {
  T intMax = ~0;
  if (value <= 0) {
    *out = minValue - 1.0;
    return;
  }
  if (value >= intMax) {
    *out = maxValue + 1.0;
    return;
  }
  rocket_FLOAT_DEF ratio = (value - 1) / (intMax - 2);
  *out = ratio * (maxValue - minValue) + minValue;
}

class MessageBase {
public:
  virtual void build_buf(uint8_t *buf, uint8_t *index) {}
  virtual void parse_buf(uint8_t *buf) {}
  virtual uint8_t get_id() {}
  virtual enum categories get_category() {}
  virtual enum nodes get_receiver() {}
  virtual enum nodes get_sender() {}
  virtual uint8_t get_size() {}
};
class handshake_from_everyone_to_everyone : public MessageBase {
public:
  uint8_t size = 0;
  enum rocket::messages message = rocket::messages::handshake;
  enum rocket::nodes sender = rocket::nodes::everyone;
  enum rocket::nodes receiver = rocket::nodes::everyone;
  enum rocket::categories category = rocket::categories::none;
  uint8_t id = 0;
  enum categories get_category() override { return category; }
  uint8_t get_size() override { return size; }
  enum nodes get_sender() override { return sender; }
  enum nodes get_receiver() override { return receiver; }
  uint8_t get_id() override { return id; }
  void build_buf(uint8_t *buf, uint8_t *index) override {}
  void parse_buf(uint8_t *buf) override {}
};

class telemetry_from_rocket_to_ground : public MessageBase {
public:
  uint32_t ms_since_boot;
  static_assert((sizeof(ms_since_boot) == 4), "invalid size");
  uint32_t flash_address;
  static_assert((sizeof(flash_address) == 4), "invalid size");
  float_t ax;
  static_assert((sizeof(ax) == 4), "invalid size");
  float_t ay;
  static_assert((sizeof(ay) == 4), "invalid size");
  float_t az;
  static_assert((sizeof(az) == 4), "invalid size");
  float_t roll;
  static_assert((sizeof(roll) == 4), "invalid size");
  float_t pitch;
  static_assert((sizeof(pitch) == 4), "invalid size");
  float_t yaw;
  static_assert((sizeof(yaw) == 4), "invalid size");
  float_t altitude;
  static_assert((sizeof(altitude) == 4), "invalid size");
  uint8_t satellites;
  static_assert((sizeof(satellites) == 1), "invalid size");
  float_t voltage;
  static_assert((sizeof(voltage) == 4), "invalid size");
  enum state state;
  static_assert((sizeof(state) == 1), "invalid size");
  float_t rssi;
  static_assert((sizeof(rssi) == 4), "invalid size");
  uint8_t size = 46;
  enum rocket::messages message = rocket::messages::telemetry;
  enum rocket::nodes sender = rocket::nodes::rocket;
  enum rocket::nodes receiver = rocket::nodes::ground;
  enum rocket::categories category = rocket::categories::none;
  uint8_t id = 1;
  enum categories get_category() override { return category; }
  uint8_t get_size() override { return size; }
  enum nodes get_sender() override { return sender; }
  enum nodes get_receiver() override { return receiver; }
  uint8_t get_id() override { return id; }
  void set_ms_since_boot(uint32_t value) { ms_since_boot = value; }
  void set_flash_address(uint32_t value) { flash_address = value; }
  void set_ax(float_t value) { ax = value; }
  void set_ay(float_t value) { ay = value; }
  void set_az(float_t value) { az = value; }
  void set_roll(float_t value) { roll = value; }
  void set_pitch(float_t value) { pitch = value; }
  void set_yaw(float_t value) { yaw = value; }
  void set_altitude(float_t value) { altitude = value; }
  void set_satellites(uint8_t value) { satellites = value; }
  void set_voltage(float_t value) { voltage = value; }
  void set_state(enum state value) { state = value; }
  void set_rssi(float_t value) { rssi = value; }
  uint32_t get_ms_since_boot() { return ms_since_boot; }
  uint32_t get_flash_address() { return flash_address; }
  float_t get_ax() { return ax; }
  float_t get_ay() { return ay; }
  float_t get_az() { return az; }
  float_t get_roll() { return roll; }
  float_t get_pitch() { return pitch; }
  float_t get_yaw() { return yaw; }
  float_t get_altitude() { return altitude; }
  uint8_t get_satellites() { return satellites; }
  float_t get_voltage() { return voltage; }
  enum state get_state() { return state; }
  float_t get_rssi() { return rssi; }
  void build_buf(uint8_t *buf, uint8_t *index) override {
    memcpy(buf + *index, &ms_since_boot, sizeof(ms_since_boot));
    *index += sizeof(ms_since_boot);
    memcpy(buf + *index, &flash_address, sizeof(flash_address));
    *index += sizeof(flash_address);
    memcpy(buf + *index, &ax, sizeof(ax));
    *index += sizeof(ax);
    memcpy(buf + *index, &ay, sizeof(ay));
    *index += sizeof(ay);
    memcpy(buf + *index, &az, sizeof(az));
    *index += sizeof(az);
    memcpy(buf + *index, &roll, sizeof(roll));
    *index += sizeof(roll);
    memcpy(buf + *index, &pitch, sizeof(pitch));
    *index += sizeof(pitch);
    memcpy(buf + *index, &yaw, sizeof(yaw));
    *index += sizeof(yaw);
    memcpy(buf + *index, &altitude, sizeof(altitude));
    *index += sizeof(altitude);
    memcpy(buf + *index, &satellites, sizeof(satellites));
    *index += sizeof(satellites);
    memcpy(buf + *index, &voltage, sizeof(voltage));
    *index += sizeof(voltage);
    memcpy(buf + *index, &state, sizeof(state));
    *index += sizeof(state);
    memcpy(buf + *index, &rssi, sizeof(rssi));
    *index += sizeof(rssi);
  }
  void parse_buf(uint8_t *buf) override {
    uint8_t index = 0;
    memcpy(&ms_since_boot, buf + index, sizeof(ms_since_boot));
    index += sizeof(ms_since_boot);
    memcpy(&flash_address, buf + index, sizeof(flash_address));
    index += sizeof(flash_address);
    memcpy(&ax, buf + index, sizeof(ax));
    index += sizeof(ax);
    memcpy(&ay, buf + index, sizeof(ay));
    index += sizeof(ay);
    memcpy(&az, buf + index, sizeof(az));
    index += sizeof(az);
    memcpy(&roll, buf + index, sizeof(roll));
    index += sizeof(roll);
    memcpy(&pitch, buf + index, sizeof(pitch));
    index += sizeof(pitch);
    memcpy(&yaw, buf + index, sizeof(yaw));
    index += sizeof(yaw);
    memcpy(&altitude, buf + index, sizeof(altitude));
    index += sizeof(altitude);
    memcpy(&satellites, buf + index, sizeof(satellites));
    index += sizeof(satellites);
    memcpy(&voltage, buf + index, sizeof(voltage));
    index += sizeof(voltage);
    memcpy(&state, buf + index, sizeof(state));
    index += sizeof(state);
    memcpy(&rssi, buf + index, sizeof(rssi));
    index += sizeof(rssi);
  }
};

class position_from_rocket_to_ground : public MessageBase {
public:
  float_t longitude;
  static_assert((sizeof(longitude) == 4), "invalid size");
  float_t latitude;
  static_assert((sizeof(latitude) == 4), "invalid size");
  uint8_t size = 8;
  enum rocket::messages message = rocket::messages::position;
  enum rocket::nodes sender = rocket::nodes::rocket;
  enum rocket::nodes receiver = rocket::nodes::ground;
  enum rocket::categories category = rocket::categories::none;
  uint8_t id = 2;
  enum categories get_category() override { return category; }
  uint8_t get_size() override { return size; }
  enum nodes get_sender() override { return sender; }
  enum nodes get_receiver() override { return receiver; }
  uint8_t get_id() override { return id; }
  void set_longitude(float_t value) { longitude = value; }
  void set_latitude(float_t value) { latitude = value; }
  float_t get_longitude() { return longitude; }
  float_t get_latitude() { return latitude; }
  void build_buf(uint8_t *buf, uint8_t *index) override {
    memcpy(buf + *index, &longitude, sizeof(longitude));
    *index += sizeof(longitude);
    memcpy(buf + *index, &latitude, sizeof(latitude));
    *index += sizeof(latitude);
  }
  void parse_buf(uint8_t *buf) override {
    uint8_t index = 0;
    memcpy(&longitude, buf + index, sizeof(longitude));
    index += sizeof(longitude);
    memcpy(&latitude, buf + index, sizeof(latitude));
    index += sizeof(latitude);
  }
};

class bmp_from_rocket_to_flash : public MessageBase {
public:
  float_t pressure;
  static_assert((sizeof(pressure) == 4), "invalid size");
  float_t temperature;
  static_assert((sizeof(temperature) == 4), "invalid size");
  uint8_t size = 8;
  enum rocket::messages message = rocket::messages::bmp;
  enum rocket::nodes sender = rocket::nodes::rocket;
  enum rocket::nodes receiver = rocket::nodes::flash;
  enum rocket::categories category = rocket::categories::none;
  uint8_t id = 3;
  enum categories get_category() override { return category; }
  uint8_t get_size() override { return size; }
  enum nodes get_sender() override { return sender; }
  enum nodes get_receiver() override { return receiver; }
  uint8_t get_id() override { return id; }
  void set_pressure(float_t value) { pressure = value; }
  void set_temperature(float_t value) { temperature = value; }
  float_t get_pressure() { return pressure; }
  float_t get_temperature() { return temperature; }
  void build_buf(uint8_t *buf, uint8_t *index) override {
    memcpy(buf + *index, &pressure, sizeof(pressure));
    *index += sizeof(pressure);
    memcpy(buf + *index, &temperature, sizeof(temperature));
    *index += sizeof(temperature);
  }
  void parse_buf(uint8_t *buf) override {
    uint8_t index = 0;
    memcpy(&pressure, buf + index, sizeof(pressure));
    index += sizeof(pressure);
    memcpy(&temperature, buf + index, sizeof(temperature));
    index += sizeof(temperature);
  }
};

class mpu_from_rocket_to_flash : public MessageBase {
public:
  float_t ax;
  static_assert((sizeof(ax) == 4), "invalid size");
  float_t ay;
  static_assert((sizeof(ay) == 4), "invalid size");
  float_t az;
  static_assert((sizeof(az) == 4), "invalid size");
  float_t gx;
  static_assert((sizeof(gx) == 4), "invalid size");
  float_t gy;
  static_assert((sizeof(gy) == 4), "invalid size");
  float_t gz;
  static_assert((sizeof(gz) == 4), "invalid size");
  float_t mx;
  static_assert((sizeof(mx) == 4), "invalid size");
  float_t my;
  static_assert((sizeof(my) == 4), "invalid size");
  float_t mz;
  static_assert((sizeof(mz) == 4), "invalid size");
  uint8_t size = 36;
  enum rocket::messages message = rocket::messages::mpu;
  enum rocket::nodes sender = rocket::nodes::rocket;
  enum rocket::nodes receiver = rocket::nodes::flash;
  enum rocket::categories category = rocket::categories::none;
  uint8_t id = 4;
  enum categories get_category() override { return category; }
  uint8_t get_size() override { return size; }
  enum nodes get_sender() override { return sender; }
  enum nodes get_receiver() override { return receiver; }
  uint8_t get_id() override { return id; }
  void set_ax(float_t value) { ax = value; }
  void set_ay(float_t value) { ay = value; }
  void set_az(float_t value) { az = value; }
  void set_gx(float_t value) { gx = value; }
  void set_gy(float_t value) { gy = value; }
  void set_gz(float_t value) { gz = value; }
  void set_mx(float_t value) { mx = value; }
  void set_my(float_t value) { my = value; }
  void set_mz(float_t value) { mz = value; }
  float_t get_ax() { return ax; }
  float_t get_ay() { return ay; }
  float_t get_az() { return az; }
  float_t get_gx() { return gx; }
  float_t get_gy() { return gy; }
  float_t get_gz() { return gz; }
  float_t get_mx() { return mx; }
  float_t get_my() { return my; }
  float_t get_mz() { return mz; }
  void build_buf(uint8_t *buf, uint8_t *index) override {
    memcpy(buf + *index, &ax, sizeof(ax));
    *index += sizeof(ax);
    memcpy(buf + *index, &ay, sizeof(ay));
    *index += sizeof(ay);
    memcpy(buf + *index, &az, sizeof(az));
    *index += sizeof(az);
    memcpy(buf + *index, &gx, sizeof(gx));
    *index += sizeof(gx);
    memcpy(buf + *index, &gy, sizeof(gy));
    *index += sizeof(gy);
    memcpy(buf + *index, &gz, sizeof(gz));
    *index += sizeof(gz);
    memcpy(buf + *index, &mx, sizeof(mx));
    *index += sizeof(mx);
    memcpy(buf + *index, &my, sizeof(my));
    *index += sizeof(my);
    memcpy(buf + *index, &mz, sizeof(mz));
    *index += sizeof(mz);
  }
  void parse_buf(uint8_t *buf) override {
    uint8_t index = 0;
    memcpy(&ax, buf + index, sizeof(ax));
    index += sizeof(ax);
    memcpy(&ay, buf + index, sizeof(ay));
    index += sizeof(ay);
    memcpy(&az, buf + index, sizeof(az));
    index += sizeof(az);
    memcpy(&gx, buf + index, sizeof(gx));
    index += sizeof(gx);
    memcpy(&gy, buf + index, sizeof(gy));
    index += sizeof(gy);
    memcpy(&gz, buf + index, sizeof(gz));
    index += sizeof(gz);
    memcpy(&mx, buf + index, sizeof(mx));
    index += sizeof(mx);
    memcpy(&my, buf + index, sizeof(my));
    index += sizeof(my);
    memcpy(&mz, buf + index, sizeof(mz));
    index += sizeof(mz);
  }
};

class bmi_accel_from_rocket_to_flash : public MessageBase {
public:
  float_t ax;
  static_assert((sizeof(ax) == 4), "invalid size");
  float_t ay;
  static_assert((sizeof(ay) == 4), "invalid size");
  float_t az;
  static_assert((sizeof(az) == 4), "invalid size");
  uint8_t size = 12;
  enum rocket::messages message = rocket::messages::bmi_accel;
  enum rocket::nodes sender = rocket::nodes::rocket;
  enum rocket::nodes receiver = rocket::nodes::flash;
  enum rocket::categories category = rocket::categories::none;
  uint8_t id = 5;
  enum categories get_category() override { return category; }
  uint8_t get_size() override { return size; }
  enum nodes get_sender() override { return sender; }
  enum nodes get_receiver() override { return receiver; }
  uint8_t get_id() override { return id; }
  void set_ax(float_t value) { ax = value; }
  void set_ay(float_t value) { ay = value; }
  void set_az(float_t value) { az = value; }
  float_t get_ax() { return ax; }
  float_t get_ay() { return ay; }
  float_t get_az() { return az; }
  void build_buf(uint8_t *buf, uint8_t *index) override {
    memcpy(buf + *index, &ax, sizeof(ax));
    *index += sizeof(ax);
    memcpy(buf + *index, &ay, sizeof(ay));
    *index += sizeof(ay);
    memcpy(buf + *index, &az, sizeof(az));
    *index += sizeof(az);
  }
  void parse_buf(uint8_t *buf) override {
    uint8_t index = 0;
    memcpy(&ax, buf + index, sizeof(ax));
    index += sizeof(ax);
    memcpy(&ay, buf + index, sizeof(ay));
    index += sizeof(ay);
    memcpy(&az, buf + index, sizeof(az));
    index += sizeof(az);
  }
};

class bmi_gyro_from_rocket_to_flash : public MessageBase {
public:
  float_t gx;
  static_assert((sizeof(gx) == 4), "invalid size");
  float_t gy;
  static_assert((sizeof(gy) == 4), "invalid size");
  float_t gz;
  static_assert((sizeof(gz) == 4), "invalid size");
  uint8_t size = 12;
  enum rocket::messages message = rocket::messages::bmi_gyro;
  enum rocket::nodes sender = rocket::nodes::rocket;
  enum rocket::nodes receiver = rocket::nodes::flash;
  enum rocket::categories category = rocket::categories::none;
  uint8_t id = 6;
  enum categories get_category() override { return category; }
  uint8_t get_size() override { return size; }
  enum nodes get_sender() override { return sender; }
  enum nodes get_receiver() override { return receiver; }
  uint8_t get_id() override { return id; }
  void set_gx(float_t value) { gx = value; }
  void set_gy(float_t value) { gy = value; }
  void set_gz(float_t value) { gz = value; }
  float_t get_gx() { return gx; }
  float_t get_gy() { return gy; }
  float_t get_gz() { return gz; }
  void build_buf(uint8_t *buf, uint8_t *index) override {
    memcpy(buf + *index, &gx, sizeof(gx));
    *index += sizeof(gx);
    memcpy(buf + *index, &gy, sizeof(gy));
    *index += sizeof(gy);
    memcpy(buf + *index, &gz, sizeof(gz));
    *index += sizeof(gz);
  }
  void parse_buf(uint8_t *buf) override {
    uint8_t index = 0;
    memcpy(&gx, buf + index, sizeof(gx));
    index += sizeof(gx);
    memcpy(&gy, buf + index, sizeof(gy));
    index += sizeof(gy);
    memcpy(&gz, buf + index, sizeof(gz));
    index += sizeof(gz);
  }
};

class tvc_angle_from_rocket_to_flash : public MessageBase {
public:
  float_t angle_x;
  static_assert((sizeof(angle_x) == 4), "invalid size");
  float_t angle_y;
  static_assert((sizeof(angle_y) == 4), "invalid size");
  uint8_t size = 8;
  enum rocket::messages message = rocket::messages::tvc_angle;
  enum rocket::nodes sender = rocket::nodes::rocket;
  enum rocket::nodes receiver = rocket::nodes::flash;
  enum rocket::categories category = rocket::categories::none;
  uint8_t id = 7;
  enum categories get_category() override { return category; }
  uint8_t get_size() override { return size; }
  enum nodes get_sender() override { return sender; }
  enum nodes get_receiver() override { return receiver; }
  uint8_t get_id() override { return id; }
  void set_angle_x(float_t value) { angle_x = value; }
  void set_angle_y(float_t value) { angle_y = value; }
  float_t get_angle_x() { return angle_x; }
  float_t get_angle_y() { return angle_y; }
  void build_buf(uint8_t *buf, uint8_t *index) override {
    memcpy(buf + *index, &angle_x, sizeof(angle_x));
    *index += sizeof(angle_x);
    memcpy(buf + *index, &angle_y, sizeof(angle_y));
    *index += sizeof(angle_y);
  }
  void parse_buf(uint8_t *buf) override {
    uint8_t index = 0;
    memcpy(&angle_x, buf + index, sizeof(angle_x));
    index += sizeof(angle_x);
    memcpy(&angle_y, buf + index, sizeof(angle_y));
    index += sizeof(angle_y);
  }
};

class battery_voltage_from_rocket_to_flash : public MessageBase {
public:
  float_t voltage;
  static_assert((sizeof(voltage) == 4), "invalid size");
  uint8_t size = 4;
  enum rocket::messages message = rocket::messages::battery_voltage;
  enum rocket::nodes sender = rocket::nodes::rocket;
  enum rocket::nodes receiver = rocket::nodes::flash;
  enum rocket::categories category = rocket::categories::none;
  uint8_t id = 8;
  enum categories get_category() override { return category; }
  uint8_t get_size() override { return size; }
  enum nodes get_sender() override { return sender; }
  enum nodes get_receiver() override { return receiver; }
  uint8_t get_id() override { return id; }
  void set_voltage(float_t value) { voltage = value; }
  float_t get_voltage() { return voltage; }
  void build_buf(uint8_t *buf, uint8_t *index) override {
    memcpy(buf + *index, &voltage, sizeof(voltage));
    *index += sizeof(voltage);
  }
  void parse_buf(uint8_t *buf) override {
    uint8_t index = 0;
    memcpy(&voltage, buf + index, sizeof(voltage));
    index += sizeof(voltage);
  }
};

class state_from_rocket_to_flash : public MessageBase {
public:
  enum state state;
  static_assert((sizeof(state) == 1), "invalid size");
  uint8_t size = 1;
  enum rocket::messages message = rocket::messages::state;
  enum rocket::nodes sender = rocket::nodes::rocket;
  enum rocket::nodes receiver = rocket::nodes::flash;
  enum rocket::categories category = rocket::categories::none;
  uint8_t id = 9;
  enum categories get_category() override { return category; }
  uint8_t get_size() override { return size; }
  enum nodes get_sender() override { return sender; }
  enum nodes get_receiver() override { return receiver; }
  uint8_t get_id() override { return id; }
  void set_state(enum state value) { state = value; }
  enum state get_state() { return state; }
  void build_buf(uint8_t *buf, uint8_t *index) override {
    memcpy(buf + *index, &state, sizeof(state));
    *index += sizeof(state);
  }
  void parse_buf(uint8_t *buf) override {
    uint8_t index = 0;
    memcpy(&state, buf + index, sizeof(state));
    index += sizeof(state);
  }
};

class rssi_from_rocket_to_flash : public MessageBase {
public:
  int16_t rssi;
  static_assert((sizeof(rssi) == 2), "invalid size");
  uint8_t size = 2;
  enum rocket::messages message = rocket::messages::rssi;
  enum rocket::nodes sender = rocket::nodes::rocket;
  enum rocket::nodes receiver = rocket::nodes::flash;
  enum rocket::categories category = rocket::categories::none;
  uint8_t id = 10;
  enum categories get_category() override { return category; }
  uint8_t get_size() override { return size; }
  enum nodes get_sender() override { return sender; }
  enum nodes get_receiver() override { return receiver; }
  uint8_t get_id() override { return id; }
  void set_rssi(rocket_FLOAT_DEF value) {
    scaledFloat_to_uint(value, 100, &rssi);
  }
  rocket_FLOAT_DEF get_rssi() {
    rocket_FLOAT_DEF out;
    uint_to_scaledFloat(rssi, 100, &out);
    return out;
  }
  void build_buf(uint8_t *buf, uint8_t *index) override {
    memcpy(buf + *index, &rssi, sizeof(rssi));
    *index += sizeof(rssi);
  }
  void parse_buf(uint8_t *buf) override {
    uint8_t index = 0;
    memcpy(&rssi, buf + index, sizeof(rssi));
    index += sizeof(rssi);
  }
};

class gps_from_rocket_to_flash : public MessageBase {
public:
  uint16_t pdop;
  static_assert((sizeof(pdop) == 2), "invalid size");
  uint8_t n_satellites;
  static_assert((sizeof(n_satellites) == 1), "invalid size");
  enum fix_type fix_type;
  static_assert((sizeof(fix_type) == 1), "invalid size");
  float_t altitude;
  static_assert((sizeof(altitude) == 4), "invalid size");
  float_t latitude;
  static_assert((sizeof(latitude) == 4), "invalid size");
  float_t longitude;
  static_assert((sizeof(longitude) == 4), "invalid size");
  uint8_t size = 16;
  enum rocket::messages message = rocket::messages::gps;
  enum rocket::nodes sender = rocket::nodes::rocket;
  enum rocket::nodes receiver = rocket::nodes::flash;
  enum rocket::categories category = rocket::categories::none;
  uint8_t id = 11;
  enum categories get_category() override { return category; }
  uint8_t get_size() override { return size; }
  enum nodes get_sender() override { return sender; }
  enum nodes get_receiver() override { return receiver; }
  uint8_t get_id() override { return id; }
  void set_pdop(rocket_FLOAT_DEF value) {
    scaledFloat_to_uint(value, 100, &pdop);
  }
  void set_n_satellites(uint8_t value) { n_satellites = value; }
  void set_fix_type(enum fix_type value) { fix_type = value; }
  void set_altitude(float_t value) { altitude = value; }
  void set_latitude(float_t value) { latitude = value; }
  void set_longitude(float_t value) { longitude = value; }
  rocket_FLOAT_DEF get_pdop() {
    rocket_FLOAT_DEF out;
    uint_to_scaledFloat(pdop, 100, &out);
    return out;
  }
  uint8_t get_n_satellites() { return n_satellites; }
  enum fix_type get_fix_type() { return fix_type; }
  float_t get_altitude() { return altitude; }
  float_t get_latitude() { return latitude; }
  float_t get_longitude() { return longitude; }
  void build_buf(uint8_t *buf, uint8_t *index) override {
    memcpy(buf + *index, &pdop, sizeof(pdop));
    *index += sizeof(pdop);
    memcpy(buf + *index, &n_satellites, sizeof(n_satellites));
    *index += sizeof(n_satellites);
    memcpy(buf + *index, &fix_type, sizeof(fix_type));
    *index += sizeof(fix_type);
    memcpy(buf + *index, &altitude, sizeof(altitude));
    *index += sizeof(altitude);
    memcpy(buf + *index, &latitude, sizeof(latitude));
    *index += sizeof(latitude);
    memcpy(buf + *index, &longitude, sizeof(longitude));
    *index += sizeof(longitude);
  }
  void parse_buf(uint8_t *buf) override {
    uint8_t index = 0;
    memcpy(&pdop, buf + index, sizeof(pdop));
    index += sizeof(pdop);
    memcpy(&n_satellites, buf + index, sizeof(n_satellites));
    index += sizeof(n_satellites);
    memcpy(&fix_type, buf + index, sizeof(fix_type));
    index += sizeof(fix_type);
    memcpy(&altitude, buf + index, sizeof(altitude));
    index += sizeof(altitude);
    memcpy(&latitude, buf + index, sizeof(latitude));
    index += sizeof(latitude);
    memcpy(&longitude, buf + index, sizeof(longitude));
    index += sizeof(longitude);
  }
};

class ms_since_boot_from_rocket_to_flash : public MessageBase {
public:
  uint32_t ms_since_boot;
  static_assert((sizeof(ms_since_boot) == 4), "invalid size");
  uint8_t size = 4;
  enum rocket::messages message = rocket::messages::ms_since_boot;
  enum rocket::nodes sender = rocket::nodes::rocket;
  enum rocket::nodes receiver = rocket::nodes::flash;
  enum rocket::categories category = rocket::categories::none;
  uint8_t id = 12;
  enum categories get_category() override { return category; }
  uint8_t get_size() override { return size; }
  enum nodes get_sender() override { return sender; }
  enum nodes get_receiver() override { return receiver; }
  uint8_t get_id() override { return id; }
  void set_ms_since_boot(uint32_t value) { ms_since_boot = value; }
  uint32_t get_ms_since_boot() { return ms_since_boot; }
  void build_buf(uint8_t *buf, uint8_t *index) override {
    memcpy(buf + *index, &ms_since_boot, sizeof(ms_since_boot));
    *index += sizeof(ms_since_boot);
  }
  void parse_buf(uint8_t *buf) override {
    uint8_t index = 0;
    memcpy(&ms_since_boot, buf + index, sizeof(ms_since_boot));
    index += sizeof(ms_since_boot);
  }
};

class arm_pyro_from_ground_to_launchpad : public MessageBase {
public:
  uint8_t bit_field = 0;
  static_assert((sizeof(bit_field) == 1), "invalid size");
  void set_armed(bool value) {
    bit_field =
        value * (bit_field | (1 << 0)) + !value * (bit_field & ~(1 << 0));
  }
  bool get_armed() { return bit_field & (1 << 0); }
  uint8_t size = 1;
  enum rocket::messages message = rocket::messages::arm_pyro;
  enum rocket::nodes sender = rocket::nodes::ground;
  enum rocket::nodes receiver = rocket::nodes::launchpad;
  enum rocket::categories category = rocket::categories::none;
  uint8_t id = 13;
  enum categories get_category() override { return category; }
  uint8_t get_size() override { return size; }
  enum nodes get_sender() override { return sender; }
  enum nodes get_receiver() override { return receiver; }
  uint8_t get_id() override { return id; }
  void build_buf(uint8_t *buf, uint8_t *index) override {
    memcpy(buf + *index, &bit_field, sizeof(bit_field));
    *index += sizeof(bit_field);
  }
  void parse_buf(uint8_t *buf) override {
    uint8_t index = 0;
    memcpy(&bit_field, buf + index, sizeof(bit_field));
    index += sizeof(bit_field);
  }
};

class enable_pyro_from_ground_to_launchpad : public MessageBase {
public:
  uint8_t channel;
  static_assert((sizeof(channel) == 1), "invalid size");
  uint8_t size = 1;
  enum rocket::messages message = rocket::messages::enable_pyro;
  enum rocket::nodes sender = rocket::nodes::ground;
  enum rocket::nodes receiver = rocket::nodes::launchpad;
  enum rocket::categories category = rocket::categories::none;
  uint8_t id = 14;
  enum categories get_category() override { return category; }
  uint8_t get_size() override { return size; }
  enum nodes get_sender() override { return sender; }
  enum nodes get_receiver() override { return receiver; }
  uint8_t get_id() override { return id; }
  void set_channel(uint8_t value) { channel = value; }
  uint8_t get_channel() { return channel; }
  void build_buf(uint8_t *buf, uint8_t *index) override {
    memcpy(buf + *index, &channel, sizeof(channel));
    *index += sizeof(channel);
  }
  void parse_buf(uint8_t *buf) override {
    uint8_t index = 0;
    memcpy(&channel, buf + index, sizeof(channel));
    index += sizeof(channel);
  }
};

class wipe_flash_from_ground_to_rocket : public MessageBase {
public:
  uint8_t this_to_42;
  static_assert((sizeof(this_to_42) == 1), "invalid size");
  uint8_t size = 1;
  enum rocket::messages message = rocket::messages::wipe_flash;
  enum rocket::nodes sender = rocket::nodes::ground;
  enum rocket::nodes receiver = rocket::nodes::rocket;
  enum rocket::categories category = rocket::categories::none;
  uint8_t id = 15;
  enum categories get_category() override { return category; }
  uint8_t get_size() override { return size; }
  enum nodes get_sender() override { return sender; }
  enum nodes get_receiver() override { return receiver; }
  uint8_t get_id() override { return id; }
  void set_this_to_42(uint8_t value) { this_to_42 = value; }
  uint8_t get_this_to_42() { return this_to_42; }
  void build_buf(uint8_t *buf, uint8_t *index) override {
    memcpy(buf + *index, &this_to_42, sizeof(this_to_42));
    *index += sizeof(this_to_42);
  }
  void parse_buf(uint8_t *buf) override {
    uint8_t index = 0;
    memcpy(&this_to_42, buf + index, sizeof(this_to_42));
    index += sizeof(this_to_42);
  }
};

class play_music_from_ground_to_rocket : public MessageBase {
public:
  uint8_t size = 0;
  enum rocket::messages message = rocket::messages::play_music;
  enum rocket::nodes sender = rocket::nodes::ground;
  enum rocket::nodes receiver = rocket::nodes::rocket;
  enum rocket::categories category = rocket::categories::none;
  uint8_t id = 16;
  enum categories get_category() override { return category; }
  uint8_t get_size() override { return size; }
  enum nodes get_sender() override { return sender; }
  enum nodes get_receiver() override { return receiver; }
  uint8_t get_id() override { return id; }
  void build_buf(uint8_t *buf, uint8_t *index) override {}
  void parse_buf(uint8_t *buf) override {}
};

class set_logging_from_ground_to_rocket : public MessageBase {
public:
  uint8_t bit_field = 0;
  static_assert((sizeof(bit_field) == 1), "invalid size");
  void set_is_enabled(bool value) {
    bit_field =
        value * (bit_field | (1 << 0)) + !value * (bit_field & ~(1 << 0));
  }
  bool get_is_enabled() { return bit_field & (1 << 0); }
  uint8_t size = 1;
  enum rocket::messages message = rocket::messages::set_logging;
  enum rocket::nodes sender = rocket::nodes::ground;
  enum rocket::nodes receiver = rocket::nodes::rocket;
  enum rocket::categories category = rocket::categories::none;
  uint8_t id = 17;
  enum categories get_category() override { return category; }
  uint8_t get_size() override { return size; }
  enum nodes get_sender() override { return sender; }
  enum nodes get_receiver() override { return receiver; }
  uint8_t get_id() override { return id; }
  void build_buf(uint8_t *buf, uint8_t *index) override {
    memcpy(buf + *index, &bit_field, sizeof(bit_field));
    *index += sizeof(bit_field);
  }
  void parse_buf(uint8_t *buf) override {
    uint8_t index = 0;
    memcpy(&bit_field, buf + index, sizeof(bit_field));
    index += sizeof(bit_field);
  }
};

class dump_flash_from_ground_to_rocket : public MessageBase {
public:
  uint8_t size = 0;
  enum rocket::messages message = rocket::messages::dump_flash;
  enum rocket::nodes sender = rocket::nodes::ground;
  enum rocket::nodes receiver = rocket::nodes::rocket;
  enum rocket::categories category = rocket::categories::none;
  uint8_t id = 18;
  enum categories get_category() override { return category; }
  uint8_t get_size() override { return size; }
  enum nodes get_sender() override { return sender; }
  enum nodes get_receiver() override { return receiver; }
  uint8_t get_id() override { return id; }
  void build_buf(uint8_t *buf, uint8_t *index) override {}
  void parse_buf(uint8_t *buf) override {}
};

class set_state_from_ground_to_rocket : public MessageBase {
public:
  enum state state;
  static_assert((sizeof(state) == 1), "invalid size");
  uint8_t size = 1;
  enum rocket::messages message = rocket::messages::set_state;
  enum rocket::nodes sender = rocket::nodes::ground;
  enum rocket::nodes receiver = rocket::nodes::rocket;
  enum rocket::categories category = rocket::categories::none;
  uint8_t id = 19;
  enum categories get_category() override { return category; }
  uint8_t get_size() override { return size; }
  enum nodes get_sender() override { return sender; }
  enum nodes get_receiver() override { return receiver; }
  uint8_t get_id() override { return id; }
  void set_state(enum state value) { state = value; }
  enum state get_state() { return state; }
  void build_buf(uint8_t *buf, uint8_t *index) override {
    memcpy(buf + *index, &state, sizeof(state));
    *index += sizeof(state);
  }
  void parse_buf(uint8_t *buf) override {
    uint8_t index = 0;
    memcpy(&state, buf + index, sizeof(state));
    index += sizeof(state);
  }
};

void rx(handshake_from_everyone_to_everyone msg);
void rx(handshake_from_everyone_to_everyone msg, void *misc);
void rx(telemetry_from_rocket_to_ground msg);
void rx(telemetry_from_rocket_to_ground msg, void *misc);
void rx(position_from_rocket_to_ground msg);
void rx(position_from_rocket_to_ground msg, void *misc);
void rx(bmp_from_rocket_to_flash msg);
void rx(bmp_from_rocket_to_flash msg, void *misc);
void rx(mpu_from_rocket_to_flash msg);
void rx(mpu_from_rocket_to_flash msg, void *misc);
void rx(bmi_accel_from_rocket_to_flash msg);
void rx(bmi_accel_from_rocket_to_flash msg, void *misc);
void rx(bmi_gyro_from_rocket_to_flash msg);
void rx(bmi_gyro_from_rocket_to_flash msg, void *misc);
void rx(tvc_angle_from_rocket_to_flash msg);
void rx(tvc_angle_from_rocket_to_flash msg, void *misc);
void rx(battery_voltage_from_rocket_to_flash msg);
void rx(battery_voltage_from_rocket_to_flash msg, void *misc);
void rx(state_from_rocket_to_flash msg);
void rx(state_from_rocket_to_flash msg, void *misc);
void rx(rssi_from_rocket_to_flash msg);
void rx(rssi_from_rocket_to_flash msg, void *misc);
void rx(gps_from_rocket_to_flash msg);
void rx(gps_from_rocket_to_flash msg, void *misc);
void rx(ms_since_boot_from_rocket_to_flash msg);
void rx(ms_since_boot_from_rocket_to_flash msg, void *misc);
void rx(arm_pyro_from_ground_to_launchpad msg);
void rx(arm_pyro_from_ground_to_launchpad msg, void *misc);
void rx(enable_pyro_from_ground_to_launchpad msg);
void rx(enable_pyro_from_ground_to_launchpad msg, void *misc);
void rx(wipe_flash_from_ground_to_rocket msg);
void rx(wipe_flash_from_ground_to_rocket msg, void *misc);
void rx(play_music_from_ground_to_rocket msg);
void rx(play_music_from_ground_to_rocket msg, void *misc);
void rx(set_logging_from_ground_to_rocket msg);
void rx(set_logging_from_ground_to_rocket msg, void *misc);
void rx(dump_flash_from_ground_to_rocket msg);
void rx(dump_flash_from_ground_to_rocket msg, void *misc);
void rx(set_state_from_ground_to_rocket msg);
void rx(set_state_from_ground_to_rocket msg, void *misc);
void parse_message(uint8_t id, uint8_t *buf);
void parse_message(uint8_t id, uint8_t *buf, void *misc);
bool is_valid_id(uint8_t id);
uint8_t id_to_len(uint8_t id);
enum nodes id_to_sender(uint8_t id);
enum nodes id_to_receiver(uint8_t id);
enum categories id_to_category(uint8_t id);
} // namespace rocket
#endif
