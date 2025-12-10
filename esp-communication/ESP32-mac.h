/*
Header file and c++ from pgiacalo
up in github.....
need to just get mac address of esp32
https://github.com/pgiacalo/ESP32_MAC_and_IP_Address/blob/main/main/esp32_mac_ip.c#L394
*/
#ifndef ESP32_MAC_H
#define ESP32_MAC_H
#include "esp_err.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include <stdbool.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C"
#endif


//A structure to contain info on mac_address
typedef struct {
    uint8_t mac[6];
    char mac_string[18]; //it'll look like "XX:XX:XX:XX:XX:XX" FORMAT!
} mac_info_t;

/*
initialization for MAC library
Something to do with NVS flash storage.... 
Must be called before any other library functions
In a sense we made a function of integer type esp_err_t
just to check if the given microcontroller has any error codes
whether in the system or in the python script....seems like this thing
is handled by some api.....

now there is a preprocessor constant called ESP_OK and ESP_FAIL
if you get ESP_OK this means no errors otherwise..... 
so it appears that esp_err_t is returning nothing more than a number
*/
esp_err_t esp32_mac_ip_init(void);

/*
so here we make a function that REUTRNS that type esp_err_t which in this case
is a number.....
BUT we want to have parameters mac_type and a pointer *mac_info which is the 
structure we made above
*/
esp_err_t esp32_mac_ip_get_mac(esp_mac_type_t mac_type, mac_info_t *mac_info);


/*
then if we so desire we will print the contents within the mac_info struct....
which has our desired mac_address
*/
void esp32_ip_print_mac(const mac_info_t *mac_info);

/*
Once we are finished we will simply deintialize the MAC library
*/
esp_err_t esp32_mac_ip_deinit(void);
// #ifdef __cplusplus
// }
// #endif
#endif