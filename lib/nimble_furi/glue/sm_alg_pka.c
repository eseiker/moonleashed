/* P-256 for Secure Connections on the PKA accelerator instead of mbedtls.
 * lib/nimble.scons renames upstream's two ECC functions out of the way. PKA
 * works big-endian and NimBLE little-endian, so every value is swapped. CPU2
 * shares the PKA and takes CFG_HW_PKA_SEMID too. furi_hal_bt keeps the PKA
 * clock on. */

#include <furi.h>
#include <stm32wbxx_ll_hsem.h>
#include <hsem_map.h>
#include <stm32wbxx_hal.h>

#include <string.h>

#include "syscfg/syscfg.h"
#include "nimble/nimble_opt.h"

/* The only HAL core function the PKA driver calls; it is built with or without SC. */
uint32_t HAL_GetTick(void) {
    return furi_get_tick();
}

#if NIMBLE_BLE_CONNECT && NIMBLE_BLE_SM && MYNEWT_VAL(BLE_SM_SC)

#include "host/ble_hs.h"
#include "host/ble_hs_hci.h"

#define TAG "NimbleSmPka"

#define PKA_KEY_LEN    32
#define PKA_OP_TIMEOUT 1000 /* ms */

/* NIST P-256, big-endian, as PKA wants them. */
static const uint8_t p256_p[PKA_KEY_LEN] = {0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x01,
                                            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                            0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff,
                                            0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

/* a = -3 mod p, so PKA takes |a| = 3 with a negative sign. */
static const uint8_t p256_abs_a[PKA_KEY_LEN] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03};

static const uint8_t p256_b[PKA_KEY_LEN] = {0x5a, 0xc6, 0x35, 0xd8, 0xaa, 0x3a, 0x93, 0xe7,
                                            0xb3, 0xeb, 0xbd, 0x55, 0x76, 0x98, 0x86, 0xbc,
                                            0x65, 0x1d, 0x06, 0xb0, 0xcc, 0x53, 0xb0, 0xf6,
                                            0x3b, 0xce, 0x3c, 0x3e, 0x27, 0xd2, 0x60, 0x4b};

static const uint8_t p256_n[PKA_KEY_LEN] = {0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00,
                                            0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                                            0xbc, 0xe6, 0xfa, 0xad, 0xa7, 0x17, 0x9e, 0x84,
                                            0xf3, 0xb9, 0xca, 0xc2, 0xfc, 0x63, 0x25, 0x51};

static const uint8_t p256_gx[PKA_KEY_LEN] = {0x6b, 0x17, 0xd1, 0xf2, 0xe1, 0x2c, 0x42, 0x47,
                                             0xf8, 0xbc, 0xe6, 0xe5, 0x63, 0xa4, 0x40, 0xf2,
                                             0x77, 0x03, 0x7d, 0x81, 0x2d, 0xeb, 0x33, 0xa0,
                                             0xf4, 0xa1, 0x39, 0x45, 0xd8, 0x98, 0xc2, 0x96};

static const uint8_t p256_gy[PKA_KEY_LEN] = {0x4f, 0xe3, 0x42, 0xe2, 0xfe, 0x1a, 0x7f, 0x9b,
                                             0x8e, 0xe7, 0xeb, 0x4a, 0x7c, 0x0f, 0x9e, 0x16,
                                             0x2b, 0xce, 0x33, 0x57, 0x6b, 0x31, 0x5e, 0xce,
                                             0xcb, 0xb6, 0x40, 0x68, 0x37, 0xbf, 0x51, 0xf5};

static void pka_swap(uint8_t* dst, const uint8_t* src, size_t len) {
    for(size_t i = 0; i < len; i++) {
        dst[len - 1 - i] = src[i];
    }
}

/* The HSEM lock does not tell CPU1 threads apart. */
static FuriMutex* pka_mutex(void) {
    static FuriMutex* mutex;
    if(!mutex) {
        FuriMutex* created = furi_mutex_alloc(FuriMutexTypeNormal);
        FURI_CRITICAL_ENTER();
        if(!mutex) {
            mutex = created;
            created = NULL;
        }
        FURI_CRITICAL_EXIT();
        if(created) furi_mutex_free(created);
    }
    return mutex;
}

/* value < p */
static bool pka_below_p(const uint8_t* be_value) {
    return memcmp(be_value, p256_p, PKA_KEY_LEN) < 0;
}

/* 0 < value < n */
static bool pka_scalar_in_range(const uint8_t* be_value) {
    bool nonzero = false;
    for(size_t i = 0; i < PKA_KEY_LEN; i++) {
        if(be_value[i] != 0) {
            nonzero = true;
            break;
        }
    }
    if(!nonzero) return false;
    return memcmp(be_value, p256_n, PKA_KEY_LEN) < 0;
}

static bool pka_acquire(PKA_HandleTypeDef* hpka) {
    furi_check(furi_mutex_acquire(pka_mutex(), FuriWaitForever) == FuriStatusOk);
    while(LL_HSEM_1StepLock(HSEM, CFG_HW_PKA_SEMID)) {
        furi_delay_tick(1);
    }

    memset(hpka, 0, sizeof(*hpka));
    hpka->Instance = PKA;
    if(HAL_PKA_Init(hpka) != HAL_OK) {
        FURI_LOG_E(TAG, "PKA init failed");
        LL_HSEM_ReleaseLock(HSEM, CFG_HW_PKA_SEMID, 0);
        furi_mutex_release(pka_mutex());
        return false;
    }
    return true;
}

/* Key material must not outlive its use; volatile keeps the stores. */
static void pka_wipe(void* buffer, size_t length) {
    volatile uint8_t* bytes = buffer;
    while(length--) {
        *bytes++ = 0;
    }
}

static void pka_release(PKA_HandleTypeDef* hpka) {
    HAL_PKA_RAMReset(hpka);
    HAL_PKA_DeInit(hpka);
    LL_HSEM_ReleaseLock(HSEM, CFG_HW_PKA_SEMID, 0);
    furi_mutex_release(pka_mutex());
}

/* out_x/out_y = scalar * (in_x, in_y), all big-endian. */
static bool pka_ecc_mul(
    const uint8_t* scalar,
    const uint8_t* in_x,
    const uint8_t* in_y,
    uint8_t* out_x,
    uint8_t* out_y) {
    PKA_HandleTypeDef hpka;
    if(!pka_acquire(&hpka)) return false;

    PKA_ECCMulInTypeDef in = {
        .scalarMulSize = PKA_KEY_LEN,
        .modulusSize = PKA_KEY_LEN,
        .coefSign = 1, /* a is negative: |a| = 3 */
        .coefA = p256_abs_a,
        .modulus = p256_p,
        .pointX = in_x,
        .pointY = in_y,
        .scalarMul = scalar,
    };

    bool ok = HAL_PKA_ECCMul(&hpka, &in, PKA_OP_TIMEOUT) == HAL_OK;
    if(ok) {
        PKA_ECCMulOutTypeDef out = {.ptX = out_x, .ptY = out_y};
        HAL_PKA_ECCMul_GetResult(&hpka, &out);
    } else {
        FURI_LOG_E(TAG, "PKA scalar multiply failed");
    }

    pka_release(&hpka);
    return ok;
}

/* An off-curve peer key would leak the private key (invalid-curve attack). */
static bool pka_point_on_curve(const uint8_t* be_x, const uint8_t* be_y) {
    PKA_HandleTypeDef hpka;
    if(!pka_acquire(&hpka)) return false;

    PKA_PointCheckInTypeDef in = {
        .modulusSize = PKA_KEY_LEN,
        .coefSign = 1,
        .coefA = p256_abs_a,
        .coefB = p256_b,
        .modulus = p256_p,
        .pointX = be_x,
        .pointY = be_y,
    };

    bool ok = HAL_PKA_PointCheck(&hpka, &in, PKA_OP_TIMEOUT) == HAL_OK;
    if(ok) ok = HAL_PKA_PointCheck_IsOnCurve(&hpka) == 1UL;
    pka_release(&hpka);
    return ok;
}

bool sm_alg_pka_selftest(void) {
    /* NIST P-256 key pair vector */
    static const uint8_t d[PKA_KEY_LEN] = {0xc9, 0xaf, 0xa9, 0xd8, 0x45, 0xba, 0x75, 0x16,
                                           0x6b, 0x5c, 0x21, 0x57, 0x67, 0xb1, 0xd6, 0x93,
                                           0x4e, 0x50, 0xc3, 0xdb, 0x36, 0xe8, 0x9b, 0x12,
                                           0x7b, 0x8a, 0x62, 0x2b, 0x12, 0x0f, 0x67, 0x21};
    static const uint8_t qx[PKA_KEY_LEN] = {0x60, 0xfe, 0xd4, 0xba, 0x25, 0x5a, 0x9d, 0x31,
                                            0xc9, 0x61, 0xeb, 0x74, 0xc6, 0x35, 0x6d, 0x68,
                                            0xc0, 0x49, 0xb8, 0x92, 0x3b, 0x61, 0xfa, 0x6c,
                                            0xe6, 0x69, 0x62, 0x2e, 0x60, 0xf2, 0x9f, 0xb6};
    static const uint8_t qy[PKA_KEY_LEN] = {0x79, 0x03, 0xfe, 0x10, 0x08, 0xb8, 0xbc, 0x99,
                                            0xa4, 0x1a, 0xe9, 0xe9, 0x56, 0x28, 0xbc, 0x64,
                                            0xf2, 0xf1, 0xb2, 0x0c, 0x2d, 0x7e, 0x9f, 0x51,
                                            0x77, 0xa3, 0xc2, 0x94, 0xd4, 0x46, 0x22, 0x99};

    uint8_t x[PKA_KEY_LEN];
    uint8_t y[PKA_KEY_LEN];
    if(!pka_ecc_mul(d, p256_gx, p256_gy, x, y)) {
        FURI_LOG_E(TAG, "selftest: scalar multiply failed");
        return false;
    }
    if(memcmp(x, qx, PKA_KEY_LEN) != 0 || memcmp(y, qy, PKA_KEY_LEN) != 0) {
        FURI_LOG_E(TAG, "selftest: wrong public key");
        return false;
    }
    if(!pka_point_on_curve(qx, qy)) {
        FURI_LOG_E(TAG, "selftest: valid point rejected");
        return false;
    }
    uint8_t bad_y[PKA_KEY_LEN];
    memcpy(bad_y, qy, PKA_KEY_LEN);
    bad_y[PKA_KEY_LEN - 1] ^= 0x01;
    if(pka_point_on_curve(qx, bad_y)) {
        FURI_LOG_E(TAG, "selftest: off-curve point accepted");
        return false;
    }
    FURI_LOG_I(TAG, "selftest passed");
    return true;
}

/* pub: X then Y, little-endian; priv: little-endian */
int ble_sm_alg_gen_key_pair(uint8_t* pub, uint8_t* priv) {
    uint8_t be_priv[PKA_KEY_LEN];
    uint8_t be_x[PKA_KEY_LEN];
    uint8_t be_y[PKA_KEY_LEN];

    int rc = 0;
    do {
        if(ble_hs_hci_rand(be_priv, sizeof(be_priv)) != 0) {
            rc = BLE_HS_EUNKNOWN;
            break;
        }
    } while(!pka_scalar_in_range(be_priv));

    if(rc == 0 && !pka_ecc_mul(be_priv, p256_gx, p256_gy, be_x, be_y)) {
        rc = BLE_HS_EUNKNOWN;
    }
    if(rc == 0) {
        pka_swap(priv, be_priv, PKA_KEY_LEN);
        pka_swap(pub, be_x, PKA_KEY_LEN);
        pka_swap(pub + PKA_KEY_LEN, be_y, PKA_KEY_LEN);
    }
    pka_wipe(be_priv, sizeof(be_priv));
    return rc;
}

int ble_sm_alg_gen_dhkey(
    const uint8_t* peer_pub_key_x,
    const uint8_t* peer_pub_key_y,
    const uint8_t* our_priv_key,
    uint8_t* out_dhkey) {
    uint8_t be_x[PKA_KEY_LEN];
    uint8_t be_y[PKA_KEY_LEN];
    uint8_t be_priv[PKA_KEY_LEN];
    uint8_t shared_x[PKA_KEY_LEN];
    uint8_t shared_y[PKA_KEY_LEN];

    pka_swap(be_x, peer_pub_key_x, PKA_KEY_LEN);
    pka_swap(be_y, peer_pub_key_y, PKA_KEY_LEN);
    pka_swap(be_priv, our_priv_key, PKA_KEY_LEN);

    int rc = 0;
    if(!pka_below_p(be_x) || !pka_below_p(be_y) || !pka_point_on_curve(be_x, be_y)) {
        FURI_LOG_E(TAG, "peer public key is not on the curve");
        rc = BLE_HS_EUNKNOWN;
    } else if(!pka_ecc_mul(be_priv, be_x, be_y, shared_x, shared_y)) {
        rc = BLE_HS_EUNKNOWN;
    } else {
        pka_swap(out_dhkey, shared_x, PKA_KEY_LEN);
    }
    pka_wipe(be_priv, sizeof(be_priv));
    pka_wipe(shared_x, sizeof(shared_x));
    pka_wipe(shared_y, sizeof(shared_y));
    return rc;
}

#endif /* NIMBLE_BLE_CONNECT && NIMBLE_BLE_SM && MYNEWT_VAL(BLE_SM_SC) */
