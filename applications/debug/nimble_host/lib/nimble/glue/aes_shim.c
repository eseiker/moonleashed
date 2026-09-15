/*
 * Minimal AES-128 ECB backing for NimBLE SMP legacy pairing (TASK-589 B2).
 *
 * ble_sm_alg.c (restored from upstream) implements the legacy pairing crypto
 * (c1, s1) with mbedTLS AES: mbedtls_aes_init/setkey_enc/crypt_ecb/free on a
 * single 128-bit ECB block. The firmware ships mbedTLS but does NOT export
 * mbedtls_aes_* to FAPs (api_symbols.csv marks them '-'), so a FAP cannot link
 * them. Rather than reflash the firmware to export them, this shim provides the
 * four functions locally inside the FAP, backed by a small self-contained
 * AES-128 encrypt. Legacy pairing only ever needs single-block ECB encryption,
 * so decrypt and other key sizes are intentionally unsupported.
 *
 * The real mbedtls_aes_context (from the exported mbedtls/aes.h) is used only as
 * opaque storage: its buf is far larger than the 176-byte AES-128 key schedule,
 * so we reinterpret the context as our own state. Built in the -w private lib.
 */

#include <string.h>
#include <stdint.h>

#include <mbedtls/aes.h>

/* AES-128: 10 rounds, 11 round keys (44 words). Fits inside mbedtls_aes_context
 * (buf is uint32_t[68]/[44]); we overlay our state on the context storage. */
typedef struct {
    uint32_t rk[44];
} AesShimState;

_Static_assert(sizeof(AesShimState) <= sizeof(mbedtls_aes_context), "ctx too small");

static const uint8_t kSbox[256] = {
    0x63, 0x7c, 0x77, 0x7b, 0xf2, 0x6b, 0x6f, 0xc5, 0x30, 0x01, 0x67, 0x2b, 0xfe, 0xd7, 0xab, 0x76,
    0xca, 0x82, 0xc9, 0x7d, 0xfa, 0x59, 0x47, 0xf0, 0xad, 0xd4, 0xa2, 0xaf, 0x9c, 0xa4, 0x72, 0xc0,
    0xb7, 0xfd, 0x93, 0x26, 0x36, 0x3f, 0xf7, 0xcc, 0x34, 0xa5, 0xe5, 0xf1, 0x71, 0xd8, 0x31, 0x15,
    0x04, 0xc7, 0x23, 0xc3, 0x18, 0x96, 0x05, 0x9a, 0x07, 0x12, 0x80, 0xe2, 0xeb, 0x27, 0xb2, 0x75,
    0x09, 0x83, 0x2c, 0x1a, 0x1b, 0x6e, 0x5a, 0xa0, 0x52, 0x3b, 0xd6, 0xb3, 0x29, 0xe3, 0x2f, 0x84,
    0x53, 0xd1, 0x00, 0xed, 0x20, 0xfc, 0xb1, 0x5b, 0x6a, 0xcb, 0xbe, 0x39, 0x4a, 0x4c, 0x58, 0xcf,
    0xd0, 0xef, 0xaa, 0xfb, 0x43, 0x4d, 0x33, 0x85, 0x45, 0xf9, 0x02, 0x7f, 0x50, 0x3c, 0x9f, 0xa8,
    0x51, 0xa3, 0x40, 0x8f, 0x92, 0x9d, 0x38, 0xf5, 0xbc, 0xb6, 0xda, 0x21, 0x10, 0xff, 0xf3, 0xd2,
    0xcd, 0x0c, 0x13, 0xec, 0x5f, 0x97, 0x44, 0x17, 0xc4, 0xa7, 0x7e, 0x3d, 0x64, 0x5d, 0x19, 0x73,
    0x60, 0x81, 0x4f, 0xdc, 0x22, 0x2a, 0x90, 0x88, 0x46, 0xee, 0xb8, 0x14, 0xde, 0x5e, 0x0b, 0xdb,
    0xe0, 0x32, 0x3a, 0x0a, 0x49, 0x06, 0x24, 0x5c, 0xc2, 0xd3, 0xac, 0x62, 0x91, 0x95, 0xe4, 0x79,
    0xe7, 0xc8, 0x37, 0x6d, 0x8d, 0xd5, 0x4e, 0xa9, 0x6c, 0x56, 0xf4, 0xea, 0x65, 0x7a, 0xae, 0x08,
    0xba, 0x78, 0x25, 0x2e, 0x1c, 0xa6, 0xb4, 0xc6, 0xe8, 0xdd, 0x74, 0x1f, 0x4b, 0xbd, 0x8b, 0x8a,
    0x70, 0x3e, 0xb5, 0x66, 0x48, 0x03, 0xf6, 0x0e, 0x61, 0x35, 0x57, 0xb9, 0x86, 0xc1, 0x1d, 0x9e,
    0xe1, 0xf8, 0x98, 0x11, 0x69, 0xd9, 0x8e, 0x94, 0x9b, 0x1e, 0x87, 0xe9, 0xce, 0x55, 0x28, 0xdf,
    0x8c, 0xa1, 0x89, 0x0d, 0xbf, 0xe6, 0x42, 0x68, 0x41, 0x99, 0x2d, 0x0f, 0xb0, 0x54, 0xbb, 0x16};

static const uint8_t kRcon[10] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x1b, 0x36};

static uint8_t xtime(uint8_t x) {
    return (uint8_t)((x << 1) ^ ((x >> 7) * 0x1b));
}

static uint8_t mul(uint8_t a, uint8_t b) {
    uint8_t r = 0;
    while(b) {
        if(b & 1) r ^= a;
        a = xtime(a);
        b >>= 1;
    }
    return r;
}

/* Expand a 16-byte key into 11 round keys stored as 44 words (column-major
 * bytes, matching the AES state layout used below). */
static void key_expand(AesShimState* st, const uint8_t key[16]) {
    uint8_t* rk = (uint8_t*)st->rk;
    memcpy(rk, key, 16);
    for(int i = 4; i < 44; i++) {
        uint8_t t[4];
        memcpy(t, rk + (i - 1) * 4, 4);
        if(i % 4 == 0) {
            uint8_t tmp = t[0];
            t[0] = kSbox[t[1]] ^ kRcon[i / 4 - 1];
            t[1] = kSbox[t[2]];
            t[2] = kSbox[t[3]];
            t[3] = kSbox[tmp];
        }
        for(int j = 0; j < 4; j++) {
            rk[i * 4 + j] = rk[(i - 4) * 4 + j] ^ t[j];
        }
    }
}

static void add_round_key(uint8_t s[16], const uint8_t* rk) {
    for(int i = 0; i < 16; i++) s[i] ^= rk[i];
}

static void sub_bytes(uint8_t s[16]) {
    for(int i = 0; i < 16; i++) s[i] = kSbox[s[i]];
}

static void shift_rows(uint8_t s[16]) {
    uint8_t t[16];
    /* State is column-major: s[c*4 + r]. Row r shifts left by r. */
    for(int r = 0; r < 4; r++) {
        for(int c = 0; c < 4; c++) {
            t[c * 4 + r] = s[((c + r) % 4) * 4 + r];
        }
    }
    memcpy(s, t, 16);
}

static void mix_columns(uint8_t s[16]) {
    for(int c = 0; c < 4; c++) {
        uint8_t* col = s + c * 4;
        uint8_t a0 = col[0], a1 = col[1], a2 = col[2], a3 = col[3];
        col[0] = (uint8_t)(mul(a0, 2) ^ mul(a1, 3) ^ a2 ^ a3);
        col[1] = (uint8_t)(a0 ^ mul(a1, 2) ^ mul(a2, 3) ^ a3);
        col[2] = (uint8_t)(a0 ^ a1 ^ mul(a2, 2) ^ mul(a3, 3));
        col[3] = (uint8_t)(mul(a0, 3) ^ a1 ^ a2 ^ mul(a3, 2));
    }
}

static void aes128_encrypt_block(const AesShimState* st, const uint8_t in[16], uint8_t out[16]) {
    const uint8_t* rk = (const uint8_t*)st->rk;
    uint8_t s[16];
    memcpy(s, in, 16);
    add_round_key(s, rk);
    for(int round = 1; round < 10; round++) {
        sub_bytes(s);
        shift_rows(s);
        mix_columns(s);
        add_round_key(s, rk + round * 16);
    }
    sub_bytes(s);
    shift_rows(s);
    add_round_key(s, rk + 160);
    memcpy(out, s, 16);
}

/* --- mbedTLS AES surface used by ble_sm_alg.c (legacy pairing only) --- */

void mbedtls_aes_init(mbedtls_aes_context* ctx) {
    memset(ctx, 0, sizeof(*ctx));
}

void mbedtls_aes_free(mbedtls_aes_context* ctx) {
    if(ctx) memset(ctx, 0, sizeof(*ctx));
}

int mbedtls_aes_setkey_enc(mbedtls_aes_context* ctx, const unsigned char* key, unsigned int keybits) {
    if(keybits != 128) return -1; /* legacy pairing only needs AES-128 */
    key_expand((AesShimState*)ctx, key);
    return 0;
}

int mbedtls_aes_crypt_ecb(
    mbedtls_aes_context* ctx,
    int mode,
    const unsigned char input[16],
    unsigned char output[16]) {
    if(mode != MBEDTLS_AES_ENCRYPT) return -1; /* only encrypt is used */
    aes128_encrypt_block((const AesShimState*)ctx, input, output);
    return 0;
}
