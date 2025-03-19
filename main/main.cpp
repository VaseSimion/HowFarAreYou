#include <string.h>
#include <string>
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_now.h"
#include "esp_littlefs.h"
#include "esp_sleep.h"
#include "GenericFunctions.h"
#include "Config.h"
#include "RelaySpecific.h"
#include "lora.h"
#include "LoraEncryption.h"
#include "esp_mac.h"
#include "u8g2.h" 
#include "SupportScreen.h"
#include "esp_mac.h"
#include <math.h>

//Logging tags
static const char* TAG = "General";
static const char* TAG_FAIL = "FAIL_SEND";

// Earth radius in kilometers
#define EARTH_RADIUS_KM 6371.0

//Temperature sensor variables
temperature_sensor_handle_t temp_handle = NULL;
float temperature = 255.0;

// Task handles
TaskHandle_t LoraTaskHandle = NULL;
TaskHandle_t DisplayTaskHandle = NULL;
TaskHandle_t EspNowTaskHandle = NULL;
TaskHandle_t GpsTaskHandle = NULL;

//ESP-NOW variables
unsigned char mac_base[MAC_SIZE] = {0};
uint8_t broadcastAddress[MAC_SIZE];

esp_now_peer_info_t peerInfo;

//Various variables
u8g2_t u8g2; // a structure which will contain all the data for one display
int i=0;
char number_str[10];

//Variables for positioning;
long own_latitude_decimal = 54493400; 
long own_longitude_decimal = 8465178;
long received_latitude_decimal = 0; 
long received_longitude_decimal = 0;

// Function to calculate distance between two points using Haversine formula
double calculate_distance(double lat1, double lon1, double lat2, double lon2) {
    // Convert degrees to radians
    ESP_LOGI(TAG, "lat1: %f, lon1: %f, lat2: %f, lon2: %f", lat1, lon1, lat2, lon2);
    lat1 = lat1 * M_PI / 180.0;
    lon1 = lon1 * M_PI / 180.0;
    lat2 = lat2 * M_PI / 180.0;
    lon2 = lon2 * M_PI / 180.0;
    
    // Haversine formula
    double dlon = lon2 - lon1;
    double dlat = lat2 - lat1;
    
    double a = sin(dlat/2) * sin(dlat/2) + cos(lat1) * cos(lat2) * sin(dlon/2) * sin(dlon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    double distance = EARTH_RADIUS_KM * c;
    
    return distance; // Returns distance in kilometers
}

// If you want the distance in meters
double calculate_distance_meters(double lat1, double lon1, double lat2, double lon2) {
    return calculate_distance(lat1, lon1, lat2, lon2) * 1000.0; // Convert km to meters
}

// Convert NMEA coordinate format (DDmm.mmmm) to decimal degrees
float nmea_to_decimal_degrees(const char *nmea_coordinate, char direction) {
    float coordinate = atof(nmea_coordinate);
    
    // Extract the degrees part (DD)
    int degrees = (int)(coordinate / 100);
    
    // Extract the minutes part (mm.mmmm)
    float minutes = coordinate - (degrees * 100);
    
    // Convert to decimal degrees format
    float decimal_degrees = degrees + (minutes / 60.0);
    
    // Apply negative sign for South or West coordinates
    if (direction == 'S' || direction == 'W') {
        decimal_degrees = -decimal_degrees;
    }
    
    return decimal_degrees;
}

// Function to parse GGA sentence which contains essential fix data
void parse_gga(char *gga_str) {
    // Example GGA: $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
    char *token;
    char *saveptr;
    
    // Get the first token (should be $GPGGA)
    token = strtok_r(gga_str, ",", &saveptr);
    if (token == NULL || strcmp(token, "$GPGGA") != 0) 
    {
        return;
    }
    
    // UTC Time
    token = strtok_r(NULL, ",", &saveptr);
    if (token != NULL) 
    {
        //ESP_LOGI(TAG, "UTC Time: %s", token);
    }
    
    // Latitude
    token = strtok_r(NULL, ",", &saveptr);
    char latitude[20] = {0};
    if (token != NULL) 
    {
        strcpy(latitude, token);
    }
    
    // N/S indicator
    token = strtok_r(NULL, ",", &saveptr);
    char ns_indicator = token != NULL ? token[0] : ' ';
    
    // Longitude
    token = strtok_r(NULL, ",", &saveptr);
    char longitude[20] = {0};
    if (token != NULL) 
    {
        strcpy(longitude, token);
    }
    
    // E/W indicator
    token = strtok_r(NULL, ",", &saveptr);
    char ew_indicator = token != NULL ? token[0] : ' ';
    
    // Position fix indicator
    token = strtok_r(NULL, ",", &saveptr);
    int fix_quality = token != NULL ? atoi(token) : 0;
    
    // Output the parsed values
    if (fix_quality > 0) 
    {
        own_latitude_decimal = 1000000 * nmea_to_decimal_degrees(latitude, ns_indicator);
        own_longitude_decimal = 1000000 * nmea_to_decimal_degrees(longitude, ew_indicator);
        
        //ESP_LOGI(TAG, "Raw coordinates: %s%c, %s%c", latitude, ns_indicator, longitude, ew_indicator);
        //ESP_LOGI(TAG, "Decimal degrees: %ld, %ld (Fix quality: %d)", own_latitude_decimal, own_longitude_decimal, fix_quality);
    } 
    else 
    {
        //ESP_LOGI(TAG, "No valid position fix obtained");
    }

    // Satellites in use
    token = strtok_r(NULL, ",", &saveptr);
    if (token != NULL) 
    {
        //ESP_LOGI(TAG, "Satellites in use: %s", token);
    }
    
    // HDOP
    token = strtok_r(NULL, ",", &saveptr);
    
    // Altitude
    token = strtok_r(NULL, ",", &saveptr);
    if (token != NULL) 
    {
        //ESP_LOGI(TAG, "Altitude: %s meters", token);
    }
}

void process_gps_data(char *buffer, int len) {
    // Null-terminate the buffer to treat it as a string
    buffer[len] = 0;
    
    // Split the buffer into lines
    char *line;
    char *saveptr;
    line = strtok_r(buffer, "\r\n", &saveptr);
    
    while (line != NULL) {
        //ESP_LOGI(TAG, "Line: %s", line);
        // Check for GGA sentence
        if (strstr(line, "$GPGGA") == line) {
            parse_gga(line);
        }
        line = strtok_r(NULL, "\r\n", &saveptr);
    }
}// Function to parse GGA sentence which contains essential fix data

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    ESP_LOGI(TAG, "Sent to MAC Address: %02X:%02X:%02X:%02X:%02X:%02X", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    ESP_LOGI(TAG, "Last Packet Send Status: %s", status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Updated callback function with robust MAC address printing
void OnDataRecv(const esp_now_recv_info_t *esp_now_info, const uint8_t *incomingData, int len) {
    ESP_LOGI(TAG, "Received from: %02X:%02X:%02X:%02X:%02X:%02X", esp_now_info->src_addr[0], esp_now_info->src_addr[1], esp_now_info->src_addr[2], esp_now_info->src_addr[3], esp_now_info->src_addr[4], esp_now_info->src_addr[5]);
    ESP_LOGI(TAG, "Received message: %s", incomingData);
    char *token = strtok((char*)incomingData, ";");
    if (token != NULL) {
        received_latitude_decimal = atol(token);
        token = strtok(NULL, ";");
        if (token != NULL) {
            received_longitude_decimal = atol(token);
        }
    }
    double distance = calculate_distance_meters(own_latitude_decimal/1000000.0, own_longitude_decimal/1000000.0, received_latitude_decimal/1000000.0, received_longitude_decimal/1000000.0);
    ESP_LOGI(pcTaskGetName(NULL), "Distance from received coordinates: %ld meters", lround(distance));
    u8g2_ClearBuffer(&u8g2);      // Clear the internal buffer
    u8g2_DrawStr(&u8g2, 0, 15, "EspNow");
    u8g2_SetFont(&u8g2, u8g2_font_8x13B_tr);
    char print_data[10];
    snprintf(print_data, sizeof(print_data), "%ld", lround(distance));
    u8g2_DrawStr(&u8g2, 50, 22, print_data);
    u8g2_SendBuffer(&u8g2);    
}

void init_esp_now() {
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(OnDataSent));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(OnDataRecv));
}

static void espnow_task(void *pvParameters)
{
    while (1) {
        char data[20];
        snprintf(data,sizeof(data), "%ld;%ld", own_latitude_decimal, own_longitude_decimal);
        ESP_LOGI(TAG, "Sending data: %s", data);
        esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)data, 20);
        if (result == ESP_OK) {
            ESP_LOGI(TAG, "Sent data: %s", data);
        } else {
            ESP_LOGE(TAG_FAIL, "Failed to send data: %s", data);
        }
        vTaskDelay(2000 / portTICK_PERIOD_MS); // Small delay to prevent tight looping
    }
}

static void lora_task(void *pvParameters)
{
    static int i=0;
    while (1) {
        char data[20];
        snprintf(data, sizeof(data), "%ld;%ld", own_latitude_decimal, own_longitude_decimal);
        data[19] = '\0';
        ESP_LOGI(TAG, "Sending data: %s", data);
        //IF Lora is used
        uint8_t encrypted_data[LORA_BUF_SIZE] = {0};
        size_t encrypted_len;
        if (!lora_received())
        {
            lora_encryption.encrypt((uint8_t*)data, sizeof(data), associated_data, associated_data_len, encrypted_data, LORA_BUF_SIZE, &encrypted_len);
            if (encrypted_len > 0) {
                // Log the encrypted data as hex bytes instead
                // ESP_LOG_BUFFER_HEXDUMP(TAG, encrypted_data, encrypted_len, ESP_LOG_INFO);
                
                lora_send_packet(encrypted_data, encrypted_len);
                ESP_LOGI(pcTaskGetName(NULL), "%d byte packet sent...", encrypted_len);
                int lost = lora_packet_lost();
                if (lost != 0) {
                ESP_LOGW(pcTaskGetName(NULL), "%d packets lost", lost);
                }  
            }
            lora_receive(); // put into receive mode, needs to be done after each receive
            vTaskDelay(2000 / portTICK_PERIOD_MS); // Small delay to prevent tight looping    
        }
        else
        {
            vTaskDelay(100 / portTICK_PERIOD_MS); // Small delay to prevent tight looping    
        }
    }
}

static void gps_task(void *pvParameters)
{
    while (1) {

        char ldata[LORA_BUF_SIZE];
        int len = uart_read_bytes(UART_NUM, ldata, (LORA_BUF_SIZE - 1), 20 / portTICK_PERIOD_MS);
        
        if (len > 0) {
            process_gps_data((char *)ldata, len);
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS); // Small delay to prevent tight looping
    }
}

static void display_lora_task(void *pvParameters)
{
    static int i=0;
    while (1) {
        if (lora_received()) {

            uint8_t encrypted_data[LORA_BUF_SIZE];
            uint8_t decrypted_data[LORA_BUF_SIZE];
            int rxLen = lora_receive_packet(encrypted_data, sizeof(encrypted_data));
            ESP_LOGI(pcTaskGetName(NULL), "Received packet length: %d", rxLen);

            size_t decrypted_len;
            esp_err_t decryption_error = lora_encryption.decrypt(encrypted_data, rxLen, associated_data, associated_data_len, 
                                                                 decrypted_data, LORA_BUF_SIZE, &decrypted_len);
            if (decryption_error != ESP_OK) {
                ESP_LOGE(pcTaskGetName(NULL), "Decryption error: %d", decryption_error);
            }
            else if(decrypted_len > 0) {
                ESP_LOGI(pcTaskGetName(NULL), "%d byte packet received:[%s]", decrypted_len, decrypted_data);
                char *token = strtok((char*)decrypted_data, ";");
                if (token != NULL) {
                    received_latitude_decimal = atol(token);
                    token = strtok(NULL, ";");
                    if (token != NULL) {
                        received_longitude_decimal = atol(token);
                    }
                }
                double distance = calculate_distance_meters(own_latitude_decimal/1000000.0, own_longitude_decimal/1000000.0, received_latitude_decimal/1000000.0, received_longitude_decimal/1000000.0);
                ESP_LOGI(pcTaskGetName(NULL), "Distance from received coordinates: %ld meters", lround(distance));
                u8g2_ClearBuffer(&u8g2);      // Clear the internal buffer
                u8g2_DrawStr(&u8g2, 0, 15, "Lora");
                u8g2_SetFont(&u8g2, u8g2_font_8x13B_tr);
                char print_data[10];
                snprintf(print_data, sizeof(print_data), "%ld", lround(distance));
                u8g2_DrawStr(&u8g2, 50, 22, print_data);
                u8g2_SendBuffer(&u8g2);                
            }
        }
        lora_receive(); // put into receive mode, needs to be done after each receive
        vTaskDelay(500 / portTICK_PERIOD_MS); // Small delay to prevent tight looping    
    }
}

extern "C" void app_main(void)
{
    init_nvs();
    init_littlefs();
    uart_init();
    ESP_ERROR_CHECK(i2c_local_master_init());
    u8g2_display_init(&u8g2);

    esp_efuse_mac_get_default(mac_base);
    ESP_LOGI(TAG, "MAC address: %02X:%02X:%02X:%02X:%02X:%02X", mac_base[0], mac_base[1], mac_base[2], mac_base[3], mac_base[4], mac_base[5]);

    if(mac_base[5]==0x54)
    {
        uint8_t target_address[MAC_SIZE] = {0x9C, 0x9E, 0x6E, 0x10, 0x9c, 0x4C};
        memccpy(&broadcastAddress, target_address, 6, 6);
    }
    else if(mac_base[5]==0x4C)
    {
        uint8_t target_address[MAC_SIZE] = {0x9C, 0x9E, 0x6E, 0x10, 0x9c, 0x54};
        memccpy(&broadcastAddress, target_address, 6, 6);
    }
    else
    {
        uint8_t target_address[MAC_SIZE] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
        memccpy(&broadcastAddress, target_address, 6, 6);
    }
    
    if(init_lora_unified() == 0)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Small delay to prevent tight looping
        lora_receive(); // put into receive mode, needs to be done after each receive

        init_temp_sens(&temp_handle);
        temperature = get_temperature(&temp_handle);
        ESP_LOGI(TAG, "Internal temperature: %.02f", temperature);


        xTaskCreate(lora_task, "lora_task", 4096, NULL, 3, &LoraTaskHandle);
        xTaskCreate(display_lora_task, "display_task", 4096, NULL, 4, &DisplayTaskHandle);
        xTaskCreate(gps_task, "dgps_task", 4096, NULL, 5, &GpsTaskHandle);
    }
    else
    {
        wifi_init();
        init_esp_now();
        memcpy(peerInfo.peer_addr, broadcastAddress, 6);
        peerInfo.channel = 0;
        peerInfo.encrypt = false;
        if (esp_now_add_peer(&peerInfo) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to add peer");
            return;
        }

        ESP_LOGE(TAG, "Lora module not recognized");
        xTaskCreate(espnow_task, "ESPNOW_task", 4096, NULL, 5, &EspNowTaskHandle);
        xTaskCreate(gps_task, "dgps_task", 4096, NULL, 6, &GpsTaskHandle);
    }
    //u8g2_ClearBuffer(&u8g2);
    //u8g2_SetFont(&u8g2, u8g2_font_8x13B_tr);
    //u8g2_DrawStr(&u8g2, 0, 15, "Hello World!");

    //u8g2_SendBuffer(&u8g2);

}