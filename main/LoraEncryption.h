#ifndef LORAENCRYPTION_H
#define LORAENCRYPTION_H

#include <stdint.h>
#include <stdlib.h>
#include "mbedtls/aes.h"
#include "mbedtls/gcm.h" // Include GCM header
#include "esp_random.h"
#include <cstring>
#include <esp_err.h>
#include <esp_log.h>

#define AES_KEY_SIZE 16
#define GCM_IV_SIZE 12 // Recommended IV size for GCM
#define GCM_TAG_SIZE 16 // Authentication Tag size

static const char *LORA_TAG = "LoRaEncryption";
const uint8_t LORA_KEY[16] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
    0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10};
    
class LoRaEncryption {
private:
    mbedtls_gcm_context gcm_ctx; // Use GCM context
    uint8_t key[AES_KEY_SIZE];

public:
    LoRaEncryption() {
        mbedtls_gcm_init(&gcm_ctx); // Initialize GCM context
    }

    ~LoRaEncryption() {
        mbedtls_gcm_free(&gcm_ctx); // Free GCM context
    }

    // Initialize with a pre-shared key
    esp_err_t init(const uint8_t *preset_key) {
        memcpy(key, preset_key, AES_KEY_SIZE);
        int ret = mbedtls_gcm_setkey(&gcm_ctx, MBEDTLS_CIPHER_ID_AES, key, AES_KEY_SIZE * 8);
        if (ret != 0) {
            ESP_LOGE(LORA_TAG, "mbedtls_gcm_setkey failed: %d", ret);
            return ESP_FAIL;
        }
        return ESP_OK;
    }

    // Encrypt data using AES-GCM
    esp_err_t encrypt(const uint8_t *input, size_t input_len,
                      const uint8_t *aad, size_t aad_len, // Associated Data
                      uint8_t *output, size_t max_output_size,
                      size_t *output_len) {

        if (max_output_size < GCM_IV_SIZE + input_len + GCM_TAG_SIZE) {
            ESP_LOGE(LORA_TAG, "Output buffer too small!");
            return ESP_ERR_INVALID_SIZE;
        }

        uint8_t iv[GCM_IV_SIZE];
        esp_fill_random(iv, GCM_IV_SIZE);
        memcpy(output, iv, GCM_IV_SIZE); // Copy IV to the beginning of the output

        int ret = mbedtls_gcm_crypt_and_tag(&gcm_ctx, MBEDTLS_AES_ENCRYPT, input_len,
                                             iv, GCM_IV_SIZE, // Initialization Vector
                                             aad, aad_len,       // Additional Authenticated Data
                                             input,              // Input data
                                             output + GCM_IV_SIZE, // Output data (after the IV)
                                             GCM_TAG_SIZE,       // Tag size
                                             output + GCM_IV_SIZE + input_len); // Tag output location

        if (ret != 0) {
            ESP_LOGE(LORA_TAG, "mbedtls_gcm_crypt_and_tag failed: %d", ret);
            return ESP_FAIL;
        }

        *output_len = GCM_IV_SIZE + input_len + GCM_TAG_SIZE; // Correct way to set output_len
        return ESP_OK;
    }

    // Decrypt data using AES-GCM
    esp_err_t decrypt(const uint8_t *input, size_t input_len,
                      const uint8_t *aad, size_t aad_len, // Associated Data
                      uint8_t *output, size_t max_output_size,
                      size_t *output_len) {

        if (input_len < GCM_IV_SIZE + GCM_TAG_SIZE) {
            ESP_LOGE(LORA_TAG, "Input too short to contain IV and tag!");
            return ESP_ERR_INVALID_SIZE;
        }

        uint8_t iv[GCM_IV_SIZE];
        memcpy(iv, input, GCM_IV_SIZE); // Extract IV from the beginning of the input

        size_t ciphertext_len = input_len - GCM_IV_SIZE - GCM_TAG_SIZE;

        if (max_output_size < ciphertext_len) {
            ESP_LOGE(LORA_TAG, "Output buffer too small!");
            return ESP_ERR_INVALID_SIZE;
        }

        int ret = mbedtls_gcm_auth_decrypt(&gcm_ctx, ciphertext_len,
            iv, GCM_IV_SIZE,
            aad, aad_len,
            input + GCM_IV_SIZE + ciphertext_len, //Tag Pointer
            GCM_TAG_SIZE,
            input + GCM_IV_SIZE,   // Ciphertext input (Correct!)
            output);
        
        if (ret != 0) {
            ESP_LOGE(LORA_TAG, "mbedtls_gcm_auth_decrypt failed: %d", ret);
            return ESP_FAIL; // Authentication failed or decryption error
        }

        *output_len = ciphertext_len; // Correct way to set output_len
        return ESP_OK;
    }
};

extern const uint8_t LORA_KEY[16]; //TODO:SIS - Change this key and move to NVS
extern LoRaEncryption lora_encryption;
extern const uint8_t associated_data[10];
extern size_t associated_data_len;
#endif // LORAENCRYPTION_H