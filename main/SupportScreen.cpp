#include <stdio.h>
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include <string.h>
#include "SupportScreen.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "lora.h"
#include "LoraEncryption.h"

#define I2C_MASTER_SCL_IO           GPIO_NUM_0      // GPIO number for I2C master clock
#define I2C_MASTER_SDA_IO           GPIO_NUM_2      // GPIO number for I2C master data
#define I2C_MASTER_NUM              I2C_NUM_0 // I2C port number for master dev
#define I2C_MASTER_FREQ_HZ          400000   // I2C master clock frequency
#define I2C_MASTER_TX_BUF_DISABLE   0        // I2C master doesn't need buffer
#define I2C_MASTER_RX_BUF_DISABLE   0        // I2C master doesn't need buffer

i2c_master_bus_handle_t i2c_bus = NULL;
i2c_master_dev_handle_t i2c_dev = NULL;

static const char *TAG = "U8G2";

esp_err_t i2c_local_master_init(void)
{
    i2c_master_bus_config_t i2c_bus_config = {
        .i2c_port = -1,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 1,
        .flags = {
            .enable_internal_pullup = true
        }
    };
    esp_err_t err = i2c_new_master_bus(&i2c_bus_config, &i2c_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus initialization failed");
    }

    i2c_device_config_t i2c_dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x3C,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
        .scl_wait_us = 1000,
        .flags = {
            .disable_ack_check = false
        }
    };

    i2c_master_bus_add_device(i2c_bus, &i2c_dev_config, &i2c_dev);
    return err;
}

// GPIO and delay function for u8g2
uint8_t u8x8_gpio_and_delay_esp32(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
    switch (msg) {
        case U8X8_MSG_GPIO_AND_DELAY_INIT:
            break;
        case U8X8_MSG_DELAY_MILLI:
            vTaskDelay(pdMS_TO_TICKS(arg_int));
            break;
        case U8X8_MSG_DELAY_10MICRO:
            vTaskDelay(pdMS_TO_TICKS(0.01 * arg_int));
            break;
        case U8X8_MSG_DELAY_100NANO:
            vTaskDelay(pdMS_TO_TICKS(0.0001));
            break;
        default:
            return 0;
    }
    return 1;
}


uint8_t u8x8_byte_esp32_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
       static uint8_t buffer[32];  // static buffer
       static uint8_t buf_idx;
       uint8_t *data;

       switch (msg) {
           case U8X8_MSG_BYTE_SEND:
               data = (uint8_t *)arg_ptr;
               while (arg_int > 0) {
                   buffer[buf_idx++] = *data;
                   data++;
                   arg_int--;
               }
               break;
           case U8X8_MSG_BYTE_INIT:
               // Already initialized in i2c_master_init()
               break;
           case U8X8_MSG_BYTE_SET_DC:
               // DC (Data/Command) bit is set as part of the I2C data
               break;
           case U8X8_MSG_BYTE_START_TRANSFER:
               buf_idx = 0;
               break;
           case U8X8_MSG_BYTE_END_TRANSFER:
               i2c_master_transmit(i2c_dev, buffer, buf_idx, 1000 / portTICK_PERIOD_MS);
               break;
           default:
               return 0;
               break;
       }
       return 1;
   }
   

void u8g2_display_init(u8g2_t *u8g2) {
    u8g2_Setup_ssd1306_i2c_128x32_univision_f(u8g2, U8G2_R0, u8x8_byte_esp32_i2c, u8x8_gpio_and_delay_esp32);
    u8g2_InitDisplay(u8g2);
    vTaskDelay(pdMS_TO_TICKS(100));  // Add a 100ms delay
    u8g2_SetPowerSave(u8g2, 0);  // Wake up display
    u8g2_ClearBuffer(u8g2);      // Clear the internal buffer
    u8g2_SetFont(u8g2, u8g2_font_8x13B_tr);  // Set font
    u8g2_DrawStr(u8g2, 0, 15, "Hello World!"); // Draw text
    u8g2_SendBuffer(u8g2);       // Transfer buffer to display
}


void displayBigText(u8g2_t *u8g2, const char* text){
    u8g2_ClearBuffer(u8g2);      // Clear the internal buffer
    u8g2_SetFont(u8g2, u8g2_font_8x13B_tr);  // Set font
    u8g2_DrawStr(u8g2, 40, 25, text); // Draw text
    u8g2_SendBuffer(u8g2);       // Transfer buffer to display
}

// Wifi related functions
void init_nvs() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

}

void wifi_init() {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
}

// Function configure the GPIO as output or input
void config_gpio(gpio_num_t gpio_num, gpio_mode_t mode) {
    // Configure the GPIO as output
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;           // Disable interrupt
    io_conf.mode = mode;                             // Set mode
    io_conf.pin_bit_mask = (1ULL << gpio_num);       // Bit mask of the pin
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;    // Disable pull-down mode
    if(gpio_num != LED_PIN){
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;        // Disable pull-up mode
    }
    else{
        io_conf.pull_up_en = GPIO_PULLUP_ENABLE;        // Enable pull-up mode
    }   
    gpio_config(&io_conf);                           // Configure GPIO with the given settings
}

void uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, LORA_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

esp_err_t i2c_master_init(i2c_master_bus_handle_t *i2c_bus)
{
    i2c_master_bus_config_t i2c_bus_config = {
        .i2c_port = -1,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 1,
        .flags = {
            .enable_internal_pullup = true
        }
    };
    esp_err_t err = i2c_new_master_bus(&i2c_bus_config, i2c_bus);
    return err;
}

void extract_mac_from_lora_message(const char* lora_message, unsigned char* mac) {
    //lora message starts with the mac address
    for (int i = 0; i < MAC_SIZE; i++) {
        char byte_str[3];
        byte_str[0] = lora_message[i*2];
        byte_str[1] = lora_message[i*2 + 1];
        byte_str[2] = '\0';
        mac[i] = strtol(byte_str, NULL, 16);
    }
}

int16_t init_lora_module(uint32_t frequency, uint8_t coding_rate, uint8_t bandwidth, uint8_t spreading_factor, const uint8_t* lora_key) // 433e6, 1, 7, 7
{
    // Initialize LoRa
    if (lora_init() == 0) {
        ESP_LOGE(pcTaskGetName(NULL), "Does not recognize the module");
        return 1;
    }

    ESP_LOGI(pcTaskGetName(NULL), "Frequency is 433MHz");
    lora_set_frequency(frequency); // 433MHz

    lora_enable_crc();

    lora_set_coding_rate(coding_rate);
    ESP_LOGI(pcTaskGetName(NULL), "coding_rate=%d", coding_rate);

    lora_set_bandwidth(bandwidth);
    ESP_LOGI(pcTaskGetName(NULL), "bandwidth=%d", bandwidth);

    lora_set_spreading_factor(spreading_factor);
    ESP_LOGI(pcTaskGetName(NULL), "spreading_factor=%d", spreading_factor);
    
    //Lora encryption side
    lora_encryption.init(lora_key);
    
    return 0;
}

int16_t init_lora_unified(void)
{
    return init_lora_module(433e6, 1, 7, 9, LORA_KEY);
}