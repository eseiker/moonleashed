#include "aes_common.h"

#include <mbedtls/aes.h>
#include <string.h>

/*
 * AES-128 for the protocols that need it, over the mbedtls implementation the
 * firmware already links for NFC and BLE. This file used to carry its own
 * AES-128, which cost about 900 bytes of flash for a second copy of the same
 * algorithm, including two 256-byte substitution tables (KNOW-710).
 *
 * The interface is unchanged: aes_key_expansion fills a caller-owned buffer that
 * the encrypt and decrypt calls take back. mbedtls expands the key inside its
 * own context, so the buffer now just carries the 16-byte key. Callers pass a
 * 176-byte buffer and never look inside it.
 */

#define AES128_KEY_SIZE 16

void aes_key_expansion(const uint8_t* key, uint8_t* round_keys) {
    memcpy(round_keys, key, AES128_KEY_SIZE);
}

void aes128_encrypt(const uint8_t* expanded_key, uint8_t* data) {
    mbedtls_aes_context ctx;
    mbedtls_aes_init(&ctx);
    if(mbedtls_aes_setkey_enc(&ctx, expanded_key, AES128_KEY_SIZE * 8) == 0) {
        mbedtls_aes_crypt_ecb(&ctx, MBEDTLS_AES_ENCRYPT, data, data);
    }
    mbedtls_aes_free(&ctx);
}

void aes128_decrypt(const uint8_t* expanded_key, uint8_t* data) {
    mbedtls_aes_context ctx;
    mbedtls_aes_init(&ctx);
    if(mbedtls_aes_setkey_dec(&ctx, expanded_key, AES128_KEY_SIZE * 8) == 0) {
        mbedtls_aes_crypt_ecb(&ctx, MBEDTLS_AES_DECRYPT, data, data);
    }
    mbedtls_aes_free(&ctx);
}

void reverse_bits_in_bytes(uint8_t* data, uint8_t len) {
    for(uint8_t i = 0; i < len; i++) {
        uint8_t byte = data[i];
        uint8_t step1 = ((byte & 0x55) << 1) | ((byte >> 1) & 0x55);
        uint8_t step2 = ((step1 & 0x33) << 2) | ((step1 >> 2) & 0x33);
        data[i] = ((step2 & 0x0F) << 4) | (step2 >> 4);
    }
}
