/*Our header file was used to declare our
functions...time to actually implement them!
*/
#include "ESP32-mac.h"

#include <stdio.h>
#include <string.h>

#include "esp_event.h"
#include "esp_mac.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

esp_err_t esp32_mac_ip_init(void) {
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  return ret;
}

esp_err_t esp32_mac_ip_get_mac(esp_mac_type_t mac_type, mac_info_t *mac_info) {
  // invalid argument??? so i guess if the input is incorrect.....
  if (mac_info == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  // this actually gets the mac address of the esp 32 in question
  // not exactly sure what or who is inputting the parameter but I know that
  // the first parameter is used to input a MAC number into the array within the
  // structure we made in the header file
  esp_err_t ret = esp_read_mac(mac_info->mac, mac_type);

  // if it doesn't check out then we'll have to just return the error code
  if (ret != ESP_OK) {
    return ret;
  }
  // else we actually put the contents inside the strucutre for the mac
  // information
  //  Format MAC address as string
  snprintf(mac_info->mac_string,
           sizeof(mac_info->mac_string),  // not exactly sure but it kinda looks
                                          // we're pointing
           // to the array and the string inside the structure, making use of
           // the string and its size....
           "%02X:%02X:%02X:%02X:%02X:%02X", mac_info->mac[0], mac_info->mac[1],
           mac_info->mac[2], mac_info->mac[3], mac_info->mac[4],
           mac_info->mac[5]);

  return ESP_OK;
}
// @param struture type mac_info_t, with any member
void esp32_ip_print_mac(const mac_info_t *mac_info) {
  if (mac_info == NULL) {
    return;
  }
  printf("\n===Device MAC Address===\n");
  // pointed to the contents within the string inside the struture mac_info_t
  printf("ESP32 MAC address: %s\n", mac_info->mac_string);
  printf("=============================\n");
}

esp_err_t esp32_mac_ip_deinit(void) { return ESP_OK; }