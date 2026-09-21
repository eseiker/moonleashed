/* Values match targets/f7/ble_glue/services/dev_info_service.c and
 * battery_service.c. */

#include <furi.h>
#include <protobuf_version.h>
#include <lib/toolbox/version.h>

#include "nimble/nimble_npl.h"
#include "nimble/nimble_port.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"

#include "info_gatt.h"

#define UUID_SVC_DIS         0x180A
#define UUID_CHR_MFG_NAME    0x2A29
#define UUID_CHR_SERIAL      0x2A25
#define UUID_CHR_FW_REV      0x2A26
#define UUID_CHR_SW_REV      0x2A28
#define UUID_SVC_BAS         0x180F
#define UUID_CHR_LEVEL       0x2A19
#define UUID_CHR_POWER_STATE 0x2A1A

/* Present, (not) discharging, (not) charging, level unsupported. */
#define POWER_STATE_CHARGING    0x7B
#define POWER_STATE_DISCHARGING 0x6F

static const ble_uuid128_t uuid_rpc_version = BLE_UUID128_INIT(
    0x33,
    0xa9,
    0xb5,
    0x3e,
    0x87,
    0x5d,
    0x1a,
    0x8e,
    0xc8,
    0x47,
    0x5e,
    0xae,
    0x6d,
    0x66,
    0xf6,
    0x03);

static const char mfg_name[] = "Flipper Devices Inc.";
static const char serial_num[] = "1.0";
static const char rpc_version[] = TOSTRING(PROTOBUF_MAJOR_VERSION.PROTOBUF_MINOR_VERSION);
static char hardware_revision[4];
static char software_revision[40];

static uint8_t battery_level = 100;
static uint8_t power_state = POWER_STATE_DISCHARGING;
static uint16_t h_level;
static uint16_t h_power_state;

static struct ble_npl_event level_event;
static struct ble_npl_event power_state_event;

static int str_access(uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt* ctxt, void* arg) {
    UNUSED(conn);
    UNUSED(attr);
    const char* str = arg;
    return os_mbuf_append(ctxt->om, str, strlen(str)) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int
    byte_access(uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt* ctxt, void* arg) {
    UNUSED(conn);
    UNUSED(attr);
    return os_mbuf_append(ctxt->om, arg, 1) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

#define READ_AUTHEN (BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC | BLE_GATT_CHR_F_READ_AUTHEN)

#define STR_CHR(uuid_ptr, str) \
    {.uuid = (uuid_ptr), .access_cb = str_access, .arg = (void*)(str), .flags = READ_AUTHEN}

static const struct ble_gatt_svc_def svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(UUID_SVC_DIS),
        .characteristics =
            (struct ble_gatt_chr_def[]){
                STR_CHR(BLE_UUID16_DECLARE(UUID_CHR_MFG_NAME), mfg_name),
                STR_CHR(BLE_UUID16_DECLARE(UUID_CHR_SERIAL), serial_num),
                STR_CHR(BLE_UUID16_DECLARE(UUID_CHR_FW_REV), hardware_revision),
                STR_CHR(BLE_UUID16_DECLARE(UUID_CHR_SW_REV), software_revision),
                STR_CHR(&uuid_rpc_version.u, rpc_version),
                {0},
            },
    },
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(UUID_SVC_BAS),
        .characteristics =
            (struct ble_gatt_chr_def[]){
                {
                    .uuid = BLE_UUID16_DECLARE(UUID_CHR_LEVEL),
                    .access_cb = byte_access,
                    .arg = &battery_level,
                    .val_handle = &h_level,
                    .flags = READ_AUTHEN | BLE_GATT_CHR_F_NOTIFY,
                },
                {
                    .uuid = BLE_UUID16_DECLARE(UUID_CHR_POWER_STATE),
                    .access_cb = byte_access,
                    .arg = &power_state,
                    .val_handle = &h_power_state,
                    .flags = READ_AUTHEN | BLE_GATT_CHR_F_NOTIFY,
                },
                {0},
            },
    },
    {0},
};

static void chr_updated_fn(struct ble_npl_event* ev) {
    ble_gatts_chr_updated(*(uint16_t*)ble_npl_event_get_arg(ev));
}

void info_gatt_init(void) {
    snprintf(
        software_revision,
        sizeof(software_revision),
        "%s %s %s %s",
        version_get_githash(NULL),
        version_get_version(NULL),
        version_get_gitbranchnum(NULL),
        version_get_builddate(NULL));
    snprintf(hardware_revision, sizeof(hardware_revision), "%d", version_get_target(NULL));

    ble_npl_event_init(&level_event, chr_updated_fn, &h_level);
    ble_npl_event_init(&power_state_event, chr_updated_fn, &h_power_state);
}

int info_gatt_register(void) {
    int rc = ble_gatts_count_cfg(svcs);
    if(rc == 0) rc = ble_gatts_add_svcs(svcs);
    return rc;
}

void info_gatt_set_battery_level(uint8_t level) {
    battery_level = level;
    ble_npl_eventq_put(nimble_port_get_dflt_eventq(), &level_event);
}

void info_gatt_set_power_state(bool charging) {
    power_state = charging ? POWER_STATE_CHARGING : POWER_STATE_DISCHARGING;
    ble_npl_eventq_put(nimble_port_get_dflt_eventq(), &power_state_event);
}
