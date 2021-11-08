/*****************************
GENERATED FILE DO NOT EDIT
******************************/

#include "rocket.h"
#include <stdint.h>

namespace rocket {
__attribute__((weak)) void rx(local_timestamp_from_local_to_local msg) {}
__attribute__((weak)) void rx(local_timestamp_from_local_to_local msg,
                              void *misc) {}
__attribute__((weak)) void rx(timestamp_from_rocket_to_ground msg) {}
__attribute__((weak)) void rx(timestamp_from_rocket_to_ground msg, void *misc) {
}
__attribute__((weak)) void rx(handshake_from_everyone_to_everyone msg) {}
__attribute__((weak)) void rx(handshake_from_everyone_to_everyone msg,
                              void *misc) {}
__attribute__((weak)) void rx(mag_calibration_from_ground_to_rocket msg) {}
__attribute__((weak)) void rx(mag_calibration_from_ground_to_rocket msg,
                              void *misc) {}
__attribute__((weak)) void rx(wipe_flash_from_ground_to_rocket msg) {}
__attribute__((weak)) void rx(wipe_flash_from_ground_to_rocket msg,
                              void *misc) {}
__attribute__((weak)) void rx(play_music_from_ground_to_rocket msg) {}
__attribute__((weak)) void rx(play_music_from_ground_to_rocket msg,
                              void *misc) {}
__attribute__((weak)) void rx(set_logging_from_ground_to_rocket msg) {}
__attribute__((weak)) void rx(set_logging_from_ground_to_rocket msg,
                              void *misc) {}
__attribute__((weak)) void rx(dump_flash_from_ground_to_rocket msg) {}
__attribute__((weak)) void rx(dump_flash_from_ground_to_rocket msg,
                              void *misc) {}
__attribute__((weak)) void rx(flash_address_from_rocket_to_ground msg) {}
__attribute__((weak)) void rx(flash_address_from_rocket_to_ground msg,
                              void *misc) {}
__attribute__((weak)) void rx(bmp_from_rocket_to_ground msg) {}
__attribute__((weak)) void rx(bmp_from_rocket_to_ground msg, void *misc) {}
__attribute__((weak)) void rx(mpu_from_rocket_to_ground msg) {}
__attribute__((weak)) void rx(mpu_from_rocket_to_ground msg, void *misc) {}
__attribute__((weak)) void rx(bmi_from_rocket_to_ground msg) {}
__attribute__((weak)) void rx(bmi_from_rocket_to_ground msg, void *misc) {}
__attribute__((weak)) void rx(battery_voltage_from_rocket_to_ground msg) {}
__attribute__((weak)) void rx(battery_voltage_from_rocket_to_ground msg,
                              void *misc) {}
__attribute__((weak)) void rx(set_state_from_ground_to_rocket msg) {}
__attribute__((weak)) void rx(set_state_from_ground_to_rocket msg, void *misc) {
}
__attribute__((weak)) void rx(state_from_rocket_to_ground msg) {}
__attribute__((weak)) void rx(state_from_rocket_to_ground msg, void *misc) {}
__attribute__((weak)) void rx(rssi_from_rocket_to_ground msg) {}
__attribute__((weak)) void rx(rssi_from_rocket_to_ground msg, void *misc) {}
__attribute__((weak)) void rx(rssi_from_relay_to_ground msg) {}
__attribute__((weak)) void rx(rssi_from_relay_to_ground msg, void *misc) {}
__attribute__((weak)) void rx(gps_state_from_rocket_to_ground msg) {}
__attribute__((weak)) void rx(gps_state_from_rocket_to_ground msg, void *misc) {
}
__attribute__((weak)) void rx(gps_pos_from_rocket_to_ground msg) {}
__attribute__((weak)) void rx(gps_pos_from_rocket_to_ground msg, void *misc) {}
__attribute__((weak)) void rx(ms_since_boot_from_rocket_to_ground msg) {}
__attribute__((weak)) void rx(ms_since_boot_from_rocket_to_ground msg,
                              void *misc) {}
__attribute__((weak)) void rx(arm_pyro_from_ground_to_launchpad msg) {}
__attribute__((weak)) void rx(arm_pyro_from_ground_to_launchpad msg,
                              void *misc) {}
__attribute__((weak)) void rx(enable_pyro_from_ground_to_launchpad msg) {}
__attribute__((weak)) void rx(enable_pyro_from_ground_to_launchpad msg,
                              void *misc) {}
__attribute__((weak)) void rx(estimate_from_rocket_to_ground msg) {}
__attribute__((weak)) void rx(estimate_from_rocket_to_ground msg, void *misc) {}
void parse_message(uint8_t id, uint8_t *buf) {
  switch (id) {
  case 0: {
    local_timestamp_from_local_to_local __message;
    __message.parse_buf(buf);
    rx(__message);
    break;
  }
  case 1: {
    timestamp_from_rocket_to_ground __message;
    __message.parse_buf(buf);
    rx(__message);
    break;
  }
  case 2: {
    handshake_from_everyone_to_everyone __message;
    __message.parse_buf(buf);
    rx(__message);
    break;
  }
  case 3: {
    mag_calibration_from_ground_to_rocket __message;
    __message.parse_buf(buf);
    rx(__message);
    break;
  }
  case 4: {
    wipe_flash_from_ground_to_rocket __message;
    __message.parse_buf(buf);
    rx(__message);
    break;
  }
  case 5: {
    play_music_from_ground_to_rocket __message;
    __message.parse_buf(buf);
    rx(__message);
    break;
  }
  case 6: {
    set_logging_from_ground_to_rocket __message;
    __message.parse_buf(buf);
    rx(__message);
    break;
  }
  case 7: {
    dump_flash_from_ground_to_rocket __message;
    __message.parse_buf(buf);
    rx(__message);
    break;
  }
  case 8: {
    flash_address_from_rocket_to_ground __message;
    __message.parse_buf(buf);
    rx(__message);
    break;
  }
  case 9: {
    bmp_from_rocket_to_ground __message;
    __message.parse_buf(buf);
    rx(__message);
    break;
  }
  case 10: {
    mpu_from_rocket_to_ground __message;
    __message.parse_buf(buf);
    rx(__message);
    break;
  }
  case 11: {
    bmi_from_rocket_to_ground __message;
    __message.parse_buf(buf);
    rx(__message);
    break;
  }
  case 12: {
    battery_voltage_from_rocket_to_ground __message;
    __message.parse_buf(buf);
    rx(__message);
    break;
  }
  case 13: {
    set_state_from_ground_to_rocket __message;
    __message.parse_buf(buf);
    rx(__message);
    break;
  }
  case 14: {
    state_from_rocket_to_ground __message;
    __message.parse_buf(buf);
    rx(__message);
    break;
  }
  case 15: {
    rssi_from_rocket_to_ground __message;
    __message.parse_buf(buf);
    rx(__message);
    break;
  }
  case 16: {
    rssi_from_relay_to_ground __message;
    __message.parse_buf(buf);
    rx(__message);
    break;
  }
  case 17: {
    gps_state_from_rocket_to_ground __message;
    __message.parse_buf(buf);
    rx(__message);
    break;
  }
  case 18: {
    gps_pos_from_rocket_to_ground __message;
    __message.parse_buf(buf);
    rx(__message);
    break;
  }
  case 19: {
    ms_since_boot_from_rocket_to_ground __message;
    __message.parse_buf(buf);
    rx(__message);
    break;
  }
  case 20: {
    arm_pyro_from_ground_to_launchpad __message;
    __message.parse_buf(buf);
    rx(__message);
    break;
  }
  case 21: {
    enable_pyro_from_ground_to_launchpad __message;
    __message.parse_buf(buf);
    rx(__message);
    break;
  }
  case 22: {
    estimate_from_rocket_to_ground __message;
    __message.parse_buf(buf);
    rx(__message);
    break;
  }
  }
}

void parse_message(uint8_t id, uint8_t *buf, void *misc) {
  switch (id) {
  case 0: {
    local_timestamp_from_local_to_local __message;
    __message.parse_buf(buf);
    rx(__message, misc);
    break;
  }
  case 1: {
    timestamp_from_rocket_to_ground __message;
    __message.parse_buf(buf);
    rx(__message, misc);
    break;
  }
  case 2: {
    handshake_from_everyone_to_everyone __message;
    __message.parse_buf(buf);
    rx(__message, misc);
    break;
  }
  case 3: {
    mag_calibration_from_ground_to_rocket __message;
    __message.parse_buf(buf);
    rx(__message, misc);
    break;
  }
  case 4: {
    wipe_flash_from_ground_to_rocket __message;
    __message.parse_buf(buf);
    rx(__message, misc);
    break;
  }
  case 5: {
    play_music_from_ground_to_rocket __message;
    __message.parse_buf(buf);
    rx(__message, misc);
    break;
  }
  case 6: {
    set_logging_from_ground_to_rocket __message;
    __message.parse_buf(buf);
    rx(__message, misc);
    break;
  }
  case 7: {
    dump_flash_from_ground_to_rocket __message;
    __message.parse_buf(buf);
    rx(__message, misc);
    break;
  }
  case 8: {
    flash_address_from_rocket_to_ground __message;
    __message.parse_buf(buf);
    rx(__message, misc);
    break;
  }
  case 9: {
    bmp_from_rocket_to_ground __message;
    __message.parse_buf(buf);
    rx(__message, misc);
    break;
  }
  case 10: {
    mpu_from_rocket_to_ground __message;
    __message.parse_buf(buf);
    rx(__message, misc);
    break;
  }
  case 11: {
    bmi_from_rocket_to_ground __message;
    __message.parse_buf(buf);
    rx(__message, misc);
    break;
  }
  case 12: {
    battery_voltage_from_rocket_to_ground __message;
    __message.parse_buf(buf);
    rx(__message, misc);
    break;
  }
  case 13: {
    set_state_from_ground_to_rocket __message;
    __message.parse_buf(buf);
    rx(__message, misc);
    break;
  }
  case 14: {
    state_from_rocket_to_ground __message;
    __message.parse_buf(buf);
    rx(__message, misc);
    break;
  }
  case 15: {
    rssi_from_rocket_to_ground __message;
    __message.parse_buf(buf);
    rx(__message, misc);
    break;
  }
  case 16: {
    rssi_from_relay_to_ground __message;
    __message.parse_buf(buf);
    rx(__message, misc);
    break;
  }
  case 17: {
    gps_state_from_rocket_to_ground __message;
    __message.parse_buf(buf);
    rx(__message, misc);
    break;
  }
  case 18: {
    gps_pos_from_rocket_to_ground __message;
    __message.parse_buf(buf);
    rx(__message, misc);
    break;
  }
  case 19: {
    ms_since_boot_from_rocket_to_ground __message;
    __message.parse_buf(buf);
    rx(__message, misc);
    break;
  }
  case 20: {
    arm_pyro_from_ground_to_launchpad __message;
    __message.parse_buf(buf);
    rx(__message, misc);
    break;
  }
  case 21: {
    enable_pyro_from_ground_to_launchpad __message;
    __message.parse_buf(buf);
    rx(__message, misc);
    break;
  }
  case 22: {
    estimate_from_rocket_to_ground __message;
    __message.parse_buf(buf);
    rx(__message, misc);
    break;
  }
  }
}

bool is_valid_id(uint8_t id) {
  switch (id) {
  case 0:
    return true;
    break;
  case 1:
    return true;
    break;
  case 2:
    return true;
    break;
  case 3:
    return true;
    break;
  case 4:
    return true;
    break;
  case 5:
    return true;
    break;
  case 6:
    return true;
    break;
  case 7:
    return true;
    break;
  case 8:
    return true;
    break;
  case 9:
    return true;
    break;
  case 10:
    return true;
    break;
  case 11:
    return true;
    break;
  case 12:
    return true;
    break;
  case 13:
    return true;
    break;
  case 14:
    return true;
    break;
  case 15:
    return true;
    break;
  case 16:
    return true;
    break;
  case 17:
    return true;
    break;
  case 18:
    return true;
    break;
  case 19:
    return true;
    break;
  case 20:
    return true;
    break;
  case 21:
    return true;
    break;
  case 22:
    return true;
    break;
  default:
    return false;
  }
}

uint8_t id_to_len(uint8_t id) {
  switch (id) {
  case 0:
    return 4;
    break;
  case 1:
    return 4;
    break;
  case 2:
    return 0;
    break;
  case 3:
    return 4;
    break;
  case 4:
    return 1;
    break;
  case 5:
    return 0;
    break;
  case 6:
    return 1;
    break;
  case 7:
    return 0;
    break;
  case 8:
    return 4;
    break;
  case 9:
    return 8;
    break;
  case 10:
    return 36;
    break;
  case 11:
    return 24;
    break;
  case 12:
    return 4;
    break;
  case 13:
    return 1;
    break;
  case 14:
    return 1;
    break;
  case 15:
    return 2;
    break;
  case 16:
    return 2;
    break;
  case 17:
    return 4;
    break;
  case 18:
    return 12;
    break;
  case 19:
    return 4;
    break;
  case 20:
    return 1;
    break;
  case 21:
    return 1;
    break;
  case 22:
    return 60;
    break;
  default:
    return 0;
  }
}

enum nodes id_to_sender(uint8_t id) {
  switch (id) {
  case 0:
    return nodes::local;
    break;
  case 1:
    return nodes::rocket;
    break;
  case 2:
    return nodes::everyone;
    break;
  case 3:
    return nodes::ground;
    break;
  case 4:
    return nodes::ground;
    break;
  case 5:
    return nodes::ground;
    break;
  case 6:
    return nodes::ground;
    break;
  case 7:
    return nodes::ground;
    break;
  case 8:
    return nodes::rocket;
    break;
  case 9:
    return nodes::rocket;
    break;
  case 10:
    return nodes::rocket;
    break;
  case 11:
    return nodes::rocket;
    break;
  case 12:
    return nodes::rocket;
    break;
  case 13:
    return nodes::ground;
    break;
  case 14:
    return nodes::rocket;
    break;
  case 15:
    return nodes::rocket;
    break;
  case 16:
    return nodes::relay;
    break;
  case 17:
    return nodes::rocket;
    break;
  case 18:
    return nodes::rocket;
    break;
  case 19:
    return nodes::rocket;
    break;
  case 20:
    return nodes::ground;
    break;
  case 21:
    return nodes::ground;
    break;
  case 22:
    return nodes::rocket;
    break;
  }
}

enum nodes id_to_receiver(uint8_t id) {
  switch (id) {
  case 0:
    return nodes::local;
    break;
  case 1:
    return nodes::ground;
    break;
  case 2:
    return nodes::everyone;
    break;
  case 3:
    return nodes::rocket;
    break;
  case 4:
    return nodes::rocket;
    break;
  case 5:
    return nodes::rocket;
    break;
  case 6:
    return nodes::rocket;
    break;
  case 7:
    return nodes::rocket;
    break;
  case 8:
    return nodes::ground;
    break;
  case 9:
    return nodes::ground;
    break;
  case 10:
    return nodes::ground;
    break;
  case 11:
    return nodes::ground;
    break;
  case 12:
    return nodes::ground;
    break;
  case 13:
    return nodes::rocket;
    break;
  case 14:
    return nodes::ground;
    break;
  case 15:
    return nodes::ground;
    break;
  case 16:
    return nodes::ground;
    break;
  case 17:
    return nodes::ground;
    break;
  case 18:
    return nodes::ground;
    break;
  case 19:
    return nodes::ground;
    break;
  case 20:
    return nodes::launchpad;
    break;
  case 21:
    return nodes::launchpad;
    break;
  case 22:
    return nodes::ground;
    break;
  }
}

enum categories id_to_category(uint8_t id) {
  switch (id) {
  case 0:
    return categories::none;
    break;
  case 1:
    return categories::none;
    break;
  case 2:
    return categories::none;
    break;
  case 3:
    return categories::none;
    break;
  case 4:
    return categories::none;
    break;
  case 5:
    return categories::none;
    break;
  case 6:
    return categories::none;
    break;
  case 7:
    return categories::none;
    break;
  case 8:
    return categories::none;
    break;
  case 9:
    return categories::none;
    break;
  case 10:
    return categories::none;
    break;
  case 11:
    return categories::none;
    break;
  case 12:
    return categories::none;
    break;
  case 13:
    return categories::none;
    break;
  case 14:
    return categories::none;
    break;
  case 15:
    return categories::none;
    break;
  case 16:
    return categories::none;
    break;
  case 17:
    return categories::none;
    break;
  case 18:
    return categories::none;
    break;
  case 19:
    return categories::none;
    break;
  case 20:
    return categories::none;
    break;
  case 21:
    return categories::none;
    break;
  case 22:
    return categories::none;
    break;
  }
}

} // namespace rocket