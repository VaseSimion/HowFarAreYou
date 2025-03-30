#include "LoraEncryption.h"

LoRaEncryption lora_encryption;

const uint8_t associated_data[10] = "MogosBest"; // Example AAD
size_t associated_data_len = strlen((char*)associated_data);