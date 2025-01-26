#include "freertos/FreeRTOS.h"
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <app_wifi.h>

#include <hap.h>
#include <hap_apple_servs.h>
#include <hap_apple_chars.h>
#include <hap_fw_upgrade.h>
#include <app_hap_setup_payload.h>
#include <hap_platform_keystore.h>

#include "esp_task_wdt.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_intr_alloc.h"
#include "driver/gpio.h"

#include "config.h"

// just
void hk_updateHapChar(char *uuid, hap_val_t *val);

static QueueHandle_t queue_garageSensor = NULL;

hap_serv_t *hap_gService = NULL;
/// @brief 0 is OPEN, 1 is CLOSED
uint8_t hk_gState = 0;
/// @brief 0 is OPEN, 1 is CLOSED
uint8_t hk_gRequestedState = 0;
/// @brief 0 is FALSE, 1 is TRUE
uint8_t hk_gIsObstructed = 0;
uint8_t hk_isDisabled = 0;

void init_wifi() {
  esp_err_t nvsErrCheck = nvs_flash_init();
  if (nvsErrCheck == ESP_ERR_NVS_NO_FREE_PAGES ||
      nvsErrCheck == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    nvsErrCheck = nvs_flash_init();
  }
  ESP_ERROR_CHECK(nvsErrCheck);

  app_wifi_init();

  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
}

void startWifi() {
  app_wifi_start(portMAX_DELAY);

  esp_wifi_set_ps(WIFI_PS_NONE);

  ESP_LOGI("[WIFI]", "WIFI CONNECTED");
}

void gTriggerRelay() {
  ESP_LOGI("[DEBUG - gTriggerRelay]", "Triggering Relay");

  gpio_set_level(GRELAY_PIN, 1);
  vTaskDelay(pdMS_TO_TICKS(RELAY_TRIGGER_TIME));
  gpio_set_level(GRELAY_PIN, 0);
}

/// @brief 0 is OPEN, 1 is CLOSED
/// @return 1 or 0
uint8_t gGetState() {
  // gpio_get_level(GSENSOR_PIN); will return 1 if its SEES SOMETHING, therefore
  // door is OPEN
  int level = gpio_get_level(GSENSOR_PIN);

  if (level == 1) {
    hk_gState = 0;
  } else {
    hk_gState = 1;
  }

  return hk_gState;
}

/// @brief 0 is OPEN, 1 is CLOSED
/// @return 1 or 0
uint8_t gRequestState(uint8_t state) {
  hk_gRequestedState = state;

  ESP_LOGI("[DEBUG - gRequestState]",
           "Setting State: %d (%s)",
           (int)hk_gRequestedState,
           hk_gRequestedState ? "CLOSED" : "OPEN");

  if (hk_gRequestedState == hk_gState) {
    return hk_gRequestedState;
  }

  gTriggerRelay();

  return hk_gRequestedState;
}

void gUpdateState(uint8_t state) {
  hap_val_t new_val;
  new_val.u = state;

  hk_updateHapChar(HAP_CHAR_UUID_CURRENT_DOOR_STATE, &new_val);
  hk_updateHapChar(HAP_CHAR_UUID_TARGET_DOOR_STATE, &new_val);
}

static void IRAM_ATTR isr_gSensorStateChange(void *arg) {
  uint8_t garageState = gGetState();
  xQueueSendFromISR(queue_garageSensor, &garageState, NULL);
}

void task_queueHandler(void *pvParameter) {
  uint8_t buf;

  for (;;) {

    if (xQueueReceive(queue_garageSensor, &buf, portMAX_DELAY)) {
      ESP_LOGI("[DEBUG - tQueueHandler]",
               "Garage State Change: %d (%s)",
               (int)buf,
               buf ? "OPEN" : "CLOSED");

      hk_gState = buf;
      gUpdateState(hk_gState);
    }
  }
}

void init_gpio() {
  gpio_config_t cfg_gSensor = { .pin_bit_mask = (1ULL << GSENSOR_PIN),
                                .mode = GPIO_MODE_INPUT,
                                .pull_down_en = GPIO_PULLDOWN_ENABLE,
                                .pull_up_en = GPIO_PULLUP_DISABLE,
                                .intr_type = GPIO_INTR_ANYEDGE };
  esp_err_t ret = gpio_config(&cfg_gSensor);
  if (ret != ESP_OK) {
    ESP_LOGE("[INIT - gpio]", "Error configuring sensor GPIO");
    ESP_ERROR_CHECK(ret);
  }

  queue_garageSensor = xQueueCreate(8, sizeof(uint8_t));
  xTaskCreate(task_queueHandler, "task_queueHandler", 2048, NULL, 3, NULL);

  ESP_ERROR_CHECK(gpio_install_isr_service(0));
  ESP_ERROR_CHECK(
    gpio_isr_handler_add(GSENSOR_PIN, isr_gSensorStateChange, NULL));

  gpio_config_t cfg_gRelay = { .pin_bit_mask = (1ULL << GRELAY_PIN),
                               .mode = GPIO_MODE_OUTPUT,
                               .pull_down_en = GPIO_PULLDOWN_DISABLE,
                               .pull_up_en = GPIO_PULLUP_DISABLE,
                               .intr_type = GPIO_INTR_DISABLE };
  ret = gpio_config(&cfg_gRelay);
  if (ret != ESP_OK) {
    ESP_LOGE("[INIT - gpio]", "Error configuring relay GPIO");
    ESP_ERROR_CHECK(ret);
  }

  gpio_set_level(GRELAY_PIN, 0);

  ESP_LOGI("[INIT - gpio]", "GPIO Initialized");
}

void hk_updateHapChar(char *uuid, hap_val_t *val) {
  hap_char_t *hap_char = hap_serv_get_char_by_uuid(hap_gService, uuid);

  hap_char_update_val(hap_char, val);
}

static int hk_identify() {
  ESP_LOGI("[DEBUG - identify]", "Requested Identify");

  return HAP_SUCCESS;
}

int hk_gWrite(hap_write_data_t write_data[],
              int count,
              void *serv_priv,
              void *write_priv) {
  ESP_LOGI("[HK]", "Requested Write");

  int ret = HAP_FAIL;

  hap_write_data_t *write;
  for (int i = 0; i < count; i++) {
    write = &write_data[i];

    const char *writeUUID = hap_char_get_type_uuid(write->hc);

    /* Setting a default error value */
    if (!strcmp(writeUUID, HAP_CHAR_UUID_TARGET_DOOR_STATE)) {
      ESP_LOGI("[Garage Write]", "Received TARGET_DOOR_STATE");

      gRequestState(write->val.u);

      ESP_LOGI("[DEBUG - hk_gWrite]", "Reported Val: %d", (int)write->val.u);

      hap_char_update_val(write->hc, &(write->val));
      *(write->status) = HAP_STATUS_SUCCESS;
      ret = HAP_SUCCESS;
    } else if (!strcmp(writeUUID, HAP_CHAR_UUID_DISABLED)) {
      ESP_LOGI("[Garage Write]", "Received DISABLED");

      hk_isDisabled = write->val.b;

      ESP_LOGI("[DEBUG - hk_gWrite]", "Reported Val: %d", write->val.b);

      hap_char_update_val(write->hc, &(write->val));
      *(write->status) = HAP_STATUS_SUCCESS;
      ret = HAP_SUCCESS;
    } else {
      ESP_LOGW("[DEBUG - hk_gWrite]", "UNHANDLED UUID: %s", writeUUID);

      *(write->status) = HAP_STATUS_RES_ABSENT;
    }
  }

  return ret;
}

int hk_gRead(hap_char_t *hc,
             hap_status_t *status_code,
             void *serv_priv,
             void *read_priv) {
  ESP_LOGI("[HK]", "Requested Read");

  if (hap_req_get_ctrl_id(read_priv)) {
    ESP_LOGI("[HK]", "Received read from %s", hap_req_get_ctrl_id(read_priv));
  }

  const char *readUUID = hap_char_get_type_uuid(hc);

  /* Setting a default error value */
  hap_val_t readVal;
  if (!strcmp(readUUID, HAP_CHAR_UUID_TARGET_DOOR_STATE)) {
    ESP_LOGI("[Garage Read]", "Received TARGET_DOOR_STATE");

    // do something
    readVal.u = hk_gRequestedState;

    ESP_LOGI("[DEBUG - hk_gRead]", "Reported Val: %d", (int)readVal.u);

    hap_char_update_val(hc, &readVal);
    *status_code = HAP_STATUS_SUCCESS;
  } else if (!strcmp(readUUID, HAP_CHAR_UUID_CURRENT_DOOR_STATE)) {
    ESP_LOGI("[Garage Read]", "Received CURRENT_DOOR_STATE");

    readVal.u = hk_gState;

    ESP_LOGI("[DEBUG - hk_gRead]", "Reported Val: %d", (int)readVal.u);

    hap_char_update_val(hc, &readVal);
    *status_code = HAP_STATUS_SUCCESS;
  } else if (!strcmp(readUUID, HAP_CHAR_UUID_NAME)) {
    ESP_LOGI("[Garage Read]", "Received NAME");

    readVal.s = "Garage Door";

    ESP_LOGI("[DEBUG - hk_gRead]", "Reported Val: %s", readVal.s);

    hap_char_update_val(hc, &readVal);
    *status_code = HAP_STATUS_SUCCESS;

  } else if (!strcmp(readUUID, HAP_CHAR_UUID_OBSTRUCTION_DETECTED)) {
    ESP_LOGI("[Garage Read]", "Received OBSTRUCTION_DETECTED");

    readVal.b = hk_gIsObstructed;

    ESP_LOGI("[DEBUG - hk_gRead]", "Reported Val: %d", readVal.b);

    hap_char_update_val(hc, &readVal);
    *status_code = HAP_STATUS_SUCCESS;
  } else if (!strcmp(readUUID, HAP_CHAR_UUID_DISABLED)) {
    ESP_LOGI("[Garage Read]", "Received DISABLED");

    readVal.b = hk_isDisabled;

    ESP_LOGI("[DEBUG - hk_gRead]", "Reported Val: %d", readVal.b);

    hap_char_update_val(hc, &readVal);
    *status_code = HAP_STATUS_SUCCESS;
  } else {
    ESP_LOGW("[DEBUG - hk_gRead]", "UNHANDLED UUID: %s", readUUID);

    *status_code = HAP_STATUS_RES_ABSENT;
  }

  if (*status_code != HAP_STATUS_SUCCESS) {
    ESP_LOGE("[DEBUG - hk_gRead]", "Error in read");

    return HAP_FAIL;
  }

  return HAP_SUCCESS;
}

void task_main(void *pvParameters) {
  hap_acc_t *accessory;
  hap_serv_t *service;

  hap_init(HAP_TRANSPORT_WIFI);

  hap_acc_cfg_t cfg_hap = {
    .name = "SKW-Garage",
    .manufacturer = "SKW",
    .model = "ESP32C6",
    .serial_num = "0000001",
    .fw_rev = "0.0.1",
    .hw_rev = "1.0",
    .pv = "1.1",
    .identify_routine = hk_identify,
    .cid = HAP_CID_GARAGE_DOOR_OPENER,
  };

  accessory = hap_acc_create(&cfg_hap);
  if (!accessory) {
    ESP_LOGE("[tHAP]", "Failed to create accessory");
    goto ERROR_INIT_GARAGEHAP;
  }

  uint8_t product_data[] = { 'S', 'K', 'W', 'G', 'A', 'R', 'A', 'G' };
  hap_acc_add_product_data(accessory, product_data, sizeof(product_data));

  hap_acc_add_wifi_transport_service(accessory, 0);

  service = hap_serv_garage_door_opener_create(hk_gState,
                                               hk_gRequestedState,
                                               hk_gIsObstructed);
  if (!service) {
    ESP_LOGE("[tHAP]", "Failed to create Garage Door Service");
    goto ERROR_INIT_GARAGEHAP;
  }

  int ret = hap_serv_add_char(service, hap_char_name_create("Garage Door"));
  if (ret != HAP_SUCCESS) {
    ESP_LOGE("[tHAP]", "Failed to add name to Garage Door Service");
    goto ERROR_INIT_GARAGEHAP;
  }

  hap_gService = service;

  // hap_char_t *char_disabled =
  //   hap_char_bool_create(HAP_CHAR_UUID_DISABLED,
  //                        HAP_CHAR_PERM_PR | HAP_CHAR_PERM_PW |
  //                        HAP_CHAR_PERM_EV, hk_isDisabled);
  // hap_serv_add_char(service, char_disabled);

  hap_serv_set_write_cb(service, hk_gWrite);
  hap_serv_set_read_cb(service, hk_gRead);
  hap_acc_add_serv(accessory, service);

  hap_add_accessory(accessory);

  init_gpio();

  hap_set_setup_code("111-22-333\0");
  /* Unique four character Setup Id. Default: ES32 */
  hap_set_setup_id("SKWG\0");
  app_hap_setup_payload("111-22-333\0", "SKWG\0", false, cfg_hap.cid);

  init_wifi();
  hap_start();
  startWifi();

  vTaskDelete(NULL);

ERROR_INIT_GARAGEHAP:
  ESP_LOGE("[tHAP]", "Error in garageHAP");

  hap_acc_delete(accessory);
  vTaskDelete(NULL);
}

void app_main(void) {
  xTaskCreate(task_main, "task_main", 4096, NULL, 10, NULL);
}