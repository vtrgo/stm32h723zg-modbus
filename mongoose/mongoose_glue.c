// SPDX-FileCopyrightText: 2024 Cesanta Software Limited
// SPDX-License-Identifier: GPL-2.0-only or commercial
// Generated by Mongoose Wizard, https://mongoose.ws/wizard/

// Default mock implementation of the API callbacks

#include "stm32h7xx_hal.h"

#include "mongoose_glue.h"

// Mock a device that has 125 read/write registers at address 1000
static uint16_t s_modbus_regs[225] = {0}; // Array of 125 registers, initialized to zero
static uint16_t s_modbus_base = 1000;  // Base address of our registers

// Read/write registers via Modbus API
// Return true if the register is read/written successfully, false otherwise
// The address is a 16-bit unsigned integer, the value is also a 16-bit unsigned integer
// The address is in the range [s_modbus_base, s_modbus_base + count), where count is the number of registers
bool glue_modbus_read_reg(uint16_t address, uint16_t *value) {
  bool success = false;
  size_t count = sizeof(s_modbus_regs) / sizeof(s_modbus_regs[0]);
  if (address >= s_modbus_base && address < s_modbus_base + count) {
    *value = s_modbus_regs[address - s_modbus_base];
    success = true;
  }
  MG_INFO(("%s: %hu = %hu", success ? "Read OK" : "Read FAIL", address, *value));
  return success;
}

void ws_voltage(struct mg_connection *c) {
  mg_ws_printf(c, WEBSOCKET_OP_TEXT, "{%m: %u}", MG_ESC("voltage"), glue_get_local_reg(1010));
}

bool glue_modbus_write_reg(uint16_t address, uint16_t value) {
  bool success = false;
  size_t count = sizeof(s_modbus_regs) / sizeof(s_modbus_regs[0]);
  if (address >= s_modbus_base && address < s_modbus_base + count) {
    s_modbus_regs[address - s_modbus_base] = value;
    success = true;
  }
  MG_INFO(("%s: %hu = %hu", success ? "Write OK" : "Write FAIL", address, value));
  return success;

}

uint16_t glue_get_local_reg(uint16_t address) {
  size_t count = sizeof(s_modbus_regs) / sizeof(s_modbus_regs[0]);
  if (address >= s_modbus_base && address < s_modbus_base + count) {
    return s_modbus_regs[address - s_modbus_base];
  }
  return 0;
}

bool glue_get_local_block(uint16_t start, uint16_t *dest, size_t len) {
  size_t count = sizeof(s_modbus_regs) / sizeof(s_modbus_regs[0]);
  if (start < s_modbus_base || (start + len) > (s_modbus_base + count)) return false;
  memcpy(dest, &s_modbus_regs[start - s_modbus_base], len * sizeof(uint16_t));
  return true;
}
// Authenticate user/password. Return access level for the authenticated user:
//   0 - authentication error
//   1,2,3... - authentication success. Higher levels are more privileged than lower
int glue_authenticate(const char *user, const char *pass) {
  int level = 0; // Authentication failure
  if (strcmp(user, "admin") == 0 && strcmp(pass, "admin") == 0) {
    level = 7;  // Administrator
  } else if (strcmp(user, "user") == 0 && strcmp(pass, "user") == 0) {
    level = 3;  // Ordinary dude
  }
  return level;
}

static uint64_t s_action_timeout_reboot;  // Time when reboot ends
bool glue_check_reboot(void) {
  return s_action_timeout_reboot > mg_now(); // Return true if reboot is in progress
}
void glue_start_reboot(struct mg_str params) {
  MG_DEBUG(("Passed parameters: [%.*s]", params.len, params.buf));
  s_action_timeout_reboot = mg_now() + 1000; // Start reboot, finish after 1 second
}

static uint64_t s_action_timeout_reformat;  // Time when reformat ends
bool glue_check_reformat(void) {
  return s_action_timeout_reformat > mg_now(); // Return true if reformat is in progress
}
void glue_start_reformat(struct mg_str params) {
  MG_DEBUG(("Passed parameters: [%.*s]", params.len, params.buf));
  s_action_timeout_reformat = mg_now() + 1000; // Start reformat, finish after 1 second
}

void *glue_ota_begin_firmware_update(char *file_name, size_t total_size) {
  bool ok = mg_ota_begin(total_size);
  MG_DEBUG(("%s size %lu, ok: %d", file_name, total_size, ok));
  return ok ? (void *) 1 : NULL;
}
bool glue_ota_end_firmware_update(void *context) {
  mg_timer_add(&g_mgr, 500, 0, (void (*)(void *)) (void *) mg_ota_end, context);
  return true;
}
bool glue_ota_write_firmware_update(void *context, void *buf, size_t len) {
  MG_DEBUG(("ctx: %p %p/%lu", context, buf, len));
  return mg_ota_write(buf, len);
}

void *glue_upload_open_file_upload(char *file_name, size_t total_size) {
  char path[128], *p = NULL;
  FILE *fp = NULL;
  if ((p = strrchr(file_name, '/')) == NULL) p = file_name;
  mg_snprintf(path, sizeof(path), "/tmp/%s", p);
#if MG_ENABLE_POSIX_FS
  fp = fopen(path, "w+b");
#endif
  MG_DEBUG(("opening [%s] size %lu, fp %p", path, total_size, fp));
  return fp;
}
bool glue_upload_close_file_upload(void *fp) {
  MG_DEBUG(("closing %p", fp));
#if MG_ENABLE_POSIX_FS
  return fclose((FILE *) fp) == 0;
#else
  return false;
#endif
}
bool glue_upload_write_file_upload(void *fp, void *buf, size_t len) {
  MG_DEBUG(("writing fp %p %p %lu bytes", fp, buf, len));
#if MG_ENABLE_POSIX_FS
  return fwrite(buf, 1, len, (FILE *) fp) == len;
#else
  return false;
#endif
}

void glue_reply_graph_data(struct mg_connection *c, struct mg_http_message *hm) {
  const char *headers = "Cache-Control: no-cache\r\n" "Content-Type: application/json\r\n";
  const char *value = "[[1724576787,20.3],[1724576847,27.2],[1724576907,29.7],[1724576967,27.9],[1724577027,25.1],[1724577087,23.8],[1724577147,22.5],[1724577207,22.2],[1724577267,23.3],[1724577327,23.9]]";
  (void) hm;
  mg_http_reply(c, 200, headers, "%s\n", value);
}
static struct state s_state = {42, 27, 67, 10, "1.0.0", true, false, 83};
void glue_get_state(struct state *data) {
  *data = s_state;  // Sync with your device
  data->speed = glue_get_local_reg(1005);
  data->temperature = glue_get_local_reg(1006);
  data->humidity = glue_get_local_reg(1007);
  data->level= glue_get_local_reg(1008);
}

static struct leds s_leds = {false, true, false};
void glue_get_leds(struct leds *data) {
  *data = s_leds;  // Sync with your device
  data->led1 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
  data->led2 = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_1);
  data->led3 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14);
}
void glue_set_leds(struct leds *data) {
  s_leds = *data; // Sync with your device
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, data->led1 ? GPIO_PIN_SET : GPIO_PIN_RESET);  // LD1
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, data->led2 ? GPIO_PIN_SET : GPIO_PIN_RESET);  // LD2
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, data->led3 ? GPIO_PIN_SET : GPIO_PIN_RESET); // LD3
}

static struct network_settings s_network_settings = {"192.168.0.42", "192.168.0.1", "255.255.255.0", true};
void glue_get_network_settings(struct network_settings *data) {
  *data = s_network_settings;  // Sync with your device
}
void glue_set_network_settings(struct network_settings *data) {
  s_network_settings = *data; // Sync with your device
}

static struct settings s_settings = {"edit & save me", "info", 123.12345, 17, true};
void glue_get_settings(struct settings *data) {
  *data = s_settings;  // Sync with your device
}
void glue_set_settings(struct settings *data) {
  s_settings = *data; // Sync with your device
}

static struct security s_security = {"admin", "user"};
void glue_get_security(struct security *data) {
  *data = s_security;  // Sync with your device
}
void glue_set_security(struct security *data) {
  s_security = *data; // Sync with your device
}

void glue_reply_loglevels(struct mg_connection *c, struct mg_http_message *hm) {
  const char *headers = "Cache-Control: no-cache\r\n" "Content-Type: application/json\r\n";
  const char *value = "[\"disabled\",\"error\",\"info\",\"debug\",\"verbose\"]";
  (void) hm;
  mg_http_reply(c, 200, headers, "%s\n", value);
}
void glue_reply_events(struct mg_connection *c, struct mg_http_message *hm) {
  const char *headers = "Cache-Control: no-cache\r\n" "Content-Type: application/json\r\n";
  const char *value = "[{\"priority\":1,\"timestamp\":1738653279,\"active\":false,\"message\":\"event 1\"},{\"priority\":2,\"timestamp\":1738653390,\"active\":true,\"message\":\"event 2\"}]";
  (void) hm;
  mg_http_reply(c, 200, headers, "%s\n", value);
}
