idf_component_register(SRCS "main.cpp" "SupportScreen.cpp" 
                       INCLUDE_DIRS "."
                       REQUIRES esp_wifi u8g2 esp_adc esp_littlefs nvs_flash esp_driver_tsens 
                                esp_driver_gpio CommonLibrary-C3)