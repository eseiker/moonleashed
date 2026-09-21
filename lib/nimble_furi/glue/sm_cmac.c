/* AES-128-CMAC (RFC 4493) for the Security Manager's f4, f5, f6 and g2.
 * ble_sm_alg.c calls mbedtls_cipher_cmac, which is provided here over
 * mbedtls_aes_crypt_ecb instead of building mbedtls' generic cipher layer. */

#include <furi.h>
#include <string.h>

#include <mbedtls/aes.h>
#include <mbedtls/cipher.h>
#include <mbedtls/cmac.h>

#define TAG "NimbleCmac"

#define CMAC_BLOCK 16

/* ble_sm_alg.c only passes this on to mbedtls_cipher_cmac: non-NULL is enough. */
static const int cmac_aes128_ecb_info = 0;

const mbedtls_cipher_info_t*
    mbedtls_cipher_info_from_type(const mbedtls_cipher_type_t cipher_type) {
    if(cipher_type != MBEDTLS_CIPHER_AES_128_ECB) return NULL;
    return (const mbedtls_cipher_info_t*)&cmac_aes128_ecb_info;
}

/* Doubling in GF(2^128), for the subkeys */
static void cmac_double(uint8_t* b) {
    uint8_t carry = b[0] >> 7;
    for(size_t i = 0; i < CMAC_BLOCK - 1; i++) {
        b[i] = (uint8_t)((b[i] << 1) | (b[i + 1] >> 7));
    }
    b[CMAC_BLOCK - 1] <<= 1;
    if(carry) b[CMAC_BLOCK - 1] ^= 0x87;
}

static void cmac_xor(uint8_t* dst, const uint8_t* src) {
    for(size_t i = 0; i < CMAC_BLOCK; i++) {
        dst[i] ^= src[i];
    }
}

int mbedtls_cipher_cmac(
    const mbedtls_cipher_info_t* cipher_info,
    const unsigned char* key,
    size_t keylen,
    const unsigned char* input,
    size_t ilen,
    unsigned char* output) {
    if(!cipher_info || !key || !output) return -1;
    if(keylen != 128) return -1;
    if(ilen && !input) return -1;

    mbedtls_aes_context aes;
    mbedtls_aes_init(&aes);
    int rc = mbedtls_aes_setkey_enc(&aes, key, 128);

    uint8_t k1[CMAC_BLOCK];
    uint8_t block[CMAC_BLOCK];
    uint8_t state[CMAC_BLOCK];

    if(rc == 0) {
        /* L = AES(key, 0), K1 = L << 1, K2 = K1 << 1 */
        memset(block, 0, sizeof(block));
        rc = mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_ENCRYPT, block, k1);
    }

    if(rc == 0) {
        cmac_double(k1); /* k1 now holds K1 */

        size_t full = ilen ? (ilen - 1) / CMAC_BLOCK : 0;
        memset(state, 0, sizeof(state));
        for(size_t i = 0; i < full && rc == 0; i++) {
            memcpy(block, input + i * CMAC_BLOCK, CMAC_BLOCK);
            cmac_xor(block, state);
            rc = mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_ENCRYPT, block, state);
        }

        if(rc == 0) {
            /* A full last block uses K1; a padded one uses K2. */
            size_t rest = ilen - full * CMAC_BLOCK;
            if(ilen && rest == CMAC_BLOCK) {
                memcpy(block, input + full * CMAC_BLOCK, CMAC_BLOCK);
                cmac_xor(block, k1);
            } else {
                memset(block, 0, sizeof(block));
                if(rest) memcpy(block, input + full * CMAC_BLOCK, rest);
                block[rest] = 0x80;
                cmac_double(k1); /* k1 now holds K2 */
                cmac_xor(block, k1);
            }
            cmac_xor(block, state);
            rc = mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_ENCRYPT, block, output);
        }
    }

    mbedtls_aes_free(&aes);
    memset(k1, 0, sizeof(k1));
    memset(block, 0, sizeof(block));
    memset(state, 0, sizeof(state));

    if(rc != 0) FURI_LOG_E(TAG, "AES-CMAC failed: %d", rc);
    return rc;
}

bool sm_cmac_selftest(void) {
    /* RFC 4493 vectors */
    static const uint8_t key[16] = {
        0x2b,
        0x7e,
        0x15,
        0x16,
        0x28,
        0xae,
        0xd2,
        0xa6,
        0xab,
        0xf7,
        0x15,
        0x88,
        0x09,
        0xcf,
        0x4f,
        0x3c};
    static const uint8_t msg[64] = {0x6b, 0xc1, 0xbe, 0xe2, 0x2e, 0x40, 0x9f, 0x96, 0xe9, 0x3d,
                                    0x7e, 0x11, 0x73, 0x93, 0x17, 0x2a, 0xae, 0x2d, 0x8a, 0x57,
                                    0x1e, 0x03, 0xac, 0x9c, 0x9e, 0xb7, 0x6f, 0xac, 0x45, 0xaf,
                                    0x8e, 0x51, 0x30, 0xc8, 0x1c, 0x46, 0xa3, 0x5c, 0xe4, 0x11,
                                    0xe5, 0xfb, 0xc1, 0x19, 0x1a, 0x0a, 0x52, 0xef, 0xf6, 0x9f,
                                    0x24, 0x45, 0xdf, 0x4f, 0x9b, 0x17, 0xad, 0x2b, 0x41, 0x7b,
                                    0xe6, 0x6c, 0x37, 0x10};
    static const struct {
        size_t len;
        uint8_t mac[16];
    } cases[] = {
        {0,
         {0xbb,
          0x1d,
          0x69,
          0x29,
          0xe9,
          0x59,
          0x37,
          0x28,
          0x7f,
          0xa3,
          0x7d,
          0x12,
          0x9b,
          0x75,
          0x67,
          0x46}},
        {16,
         {0x07,
          0x0a,
          0x16,
          0xb4,
          0x6b,
          0x4d,
          0x41,
          0x44,
          0xf7,
          0x9b,
          0xdd,
          0x9d,
          0xd0,
          0x4a,
          0x28,
          0x7c}},
        {40,
         {0xdf,
          0xa6,
          0x67,
          0x47,
          0xde,
          0x9a,
          0xe6,
          0x30,
          0x30,
          0xca,
          0x32,
          0x61,
          0x14,
          0x97,
          0xc8,
          0x27}},
        {64,
         {0x51,
          0xf0,
          0xbe,
          0xbf,
          0x7e,
          0x3b,
          0x9d,
          0x92,
          0xfc,
          0x49,
          0x74,
          0x17,
          0x79,
          0x36,
          0x3c,
          0xfe}},
    };

    const mbedtls_cipher_info_t* info = mbedtls_cipher_info_from_type(MBEDTLS_CIPHER_AES_128_ECB);
    for(size_t i = 0; i < COUNT_OF(cases); i++) {
        uint8_t out[16];
        if(mbedtls_cipher_cmac(info, key, 128, msg, cases[i].len, out) != 0) {
            FURI_LOG_E(TAG, "selftest: case %u failed to run", i);
            return false;
        }
        if(memcmp(out, cases[i].mac, sizeof(out)) != 0) {
            FURI_LOG_E(TAG, "selftest: case %u wrong MAC", i);
            return false;
        }
    }
    FURI_LOG_I(TAG, "AES-CMAC selftest passed");
    return true;
}
