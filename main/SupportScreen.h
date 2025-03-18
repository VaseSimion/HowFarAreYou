#ifndef SUPPORT_SCREEN_H
#define SUPPORT_SCREEN_H

#include "u8g2.h"
#include "esp_log.h"
#include "GenericFunctions.h"
#include "SoilMoistureSensorSpecific.h"

esp_err_t i2c_local_master_init(void);
void u8g2_display_init(u8g2_t *u8g2);

void displayBigText(u8g2_t *u8g2, const char* text);

#endif // SUPPORT_SCREEN_H