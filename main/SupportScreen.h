#ifndef SUPPORT_SCREEN_H
#define SUPPORT_SCREEN_H

#include "u8g2.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/temperature_sensor.h"
#include "driver/i2c_master.h"
#include "stdint.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#include <vector>
#include "lora.h"
#include "loraEncryption.h"


#define UART_NUM UART_NUM_1
#define UART_TX_PIN GPIO_NUM_10
#define UART_RX_PIN GPIO_NUM_8

#define LED_PIN           GPIO_NUM_2
#define MAC_SIZE 6
#define LORA_BUF_SIZE 1024

esp_err_t i2c_local_master_init(void);
void u8g2_display_init(u8g2_t *u8g2);

void displayBigText(u8g2_t *u8g2, const char* text);

// Wifi related functions
void init_nvs(void);
void wifi_init(void); 
void config_gpio(gpio_num_t gpio_num, gpio_mode_t mode);
void uart_init(void);
void extract_mac_from_lora_message(const char* lora_message, unsigned char* mac);
int16_t init_lora_module(uint32_t frequency, uint8_t coding_rate, uint8_t bandwidth, uint8_t spreading_factor, const uint8_t* lora_key); // 433e6, 1, 7, 7
int16_t init_lora_unified(void);

#endif // SUPPORT_SCREEN_H