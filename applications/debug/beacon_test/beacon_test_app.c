/*
 * Drives the furi_hal_bt_extra_beacon_* API the way third-party apps do, so the
 * NimBLE-backed extra beacon can be checked with a scanner (TASK-810).
 *
 *   loader open /ext/apps/Debug/beacon_test.fap        one beacon for 10 s
 *   loader open /ext/apps/Debug/beacon_test.fap spam   40 short beacons, each
 *                                                       from a new address
 *
 * Every beacon advertises the name "FlipBeacon" and manufacturer data
 * FF FF <counter>, so a scanner can tell the rounds apart.
 */

#include <furi.h>
#include <furi_hal_bt.h>
#include <furi_hal_random.h>
#include <string.h>

#define TAG "BeaconTest"

static uint8_t beacon_payload(uint8_t* p, uint8_t counter) {
    static const char name[] = "FlipBeacon";
    uint8_t n = 0;
    p[n++] = 2; /* flags: LE General Discoverable, BR/EDR not supported */
    p[n++] = 0x01;
    p[n++] = 0x06;
    p[n++] = sizeof(name); /* type byte plus the name, without its NUL */
    p[n++] = 0x09;
    memcpy(p + n, name, sizeof(name) - 1);
    n += sizeof(name) - 1;
    p[n++] = 4; /* manufacturer data: test company 0xFFFF, then the counter */
    p[n++] = 0xFF;
    p[n++] = 0xFF;
    p[n++] = 0xFF;
    p[n++] = counter;
    return n;
}

static bool beacon_start(uint8_t counter, uint16_t min_ms, uint16_t max_ms) {
    GapExtraBeaconConfig config = {
        .min_adv_interval_ms = min_ms,
        .max_adv_interval_ms = max_ms,
        .adv_channel_map = GapAdvChannelMapAll,
        .adv_power_level = GapAdvPowerLevel_0dBm,
        .address_type = GapAddressTypeRandom,
    };
    furi_hal_random_fill_buf(config.address, sizeof(config.address));
    config.address[5] |= 0xC0; /* static random */
    if(!furi_hal_bt_extra_beacon_set_config(&config)) return false;

    uint8_t payload[EXTRA_BEACON_MAX_DATA_SIZE];
    uint8_t len = beacon_payload(payload, counter);
    furi_hal_bt_extra_beacon_set_data(payload, len);
    return furi_hal_bt_extra_beacon_start();
}

int32_t beacon_test_app(void* context) {
    const char* args = context;
    bool spam = args && strcmp(args, "spam") == 0;

    if(furi_hal_bt_extra_beacon_is_active()) furi_hal_bt_extra_beacon_stop();

    if(spam) {
        uint8_t started = 0;
        for(uint8_t i = 0; i < 40; i++) {
            if(beacon_start(i, 20, 40)) started++;
            furi_delay_ms(100);
            furi_hal_bt_extra_beacon_stop();
        }
        FURI_LOG_I(TAG, "spam: %u of 40 beacons started", started);
    } else {
        bool ok = beacon_start(0, 100, 150);
        FURI_LOG_I(TAG, "beacon start %d, active %d", ok, furi_hal_bt_extra_beacon_is_active());
        furi_delay_ms(10000);
        furi_hal_bt_extra_beacon_stop();
        FURI_LOG_I(TAG, "beacon stopped, active %d", furi_hal_bt_extra_beacon_is_active());
    }
    return 0;
}
