/*
 * Dynamic GATT services on the resident NimBLE host (TASK-632). See dyn_gatt.h.
 *
 * Memory rules: a service block (definitions + NimBLE def arrays) is malloc'd
 * when the service is added. Removing a service or characteristic only marks it
 * unused, because the live NimBLE table still points at that memory; the block
 * and values are freed in dyn_gatt_register_all, which runs after a successful
 * ble_gatts_reset has dropped those pointers. Characteristic ids encode
 * (service slot, characteristic slot) and stay stable across rebuilds.
 */

#include <furi.h>
#include <string.h>

#include "nimble/ble.h"
#include "os/os_mbuf.h"
#include "host/ble_hs.h"
#include "host/ble_gatt.h"
#include "host/ble_uuid.h"

#include "dyn_gatt.h"

#define TAG "DynGatt"

_Static_assert(DYN_GATT_F_READ == BLE_GATT_CHR_F_READ, "flag mismatch");
_Static_assert(DYN_GATT_F_WRITE_NO_RSP == BLE_GATT_CHR_F_WRITE_NO_RSP, "flag mismatch");
_Static_assert(DYN_GATT_F_WRITE == BLE_GATT_CHR_F_WRITE, "flag mismatch");
_Static_assert(DYN_GATT_F_NOTIFY == BLE_GATT_CHR_F_NOTIFY, "flag mismatch");
_Static_assert(DYN_GATT_F_INDICATE == BLE_GATT_CHR_F_INDICATE, "flag mismatch");
_Static_assert(DYN_GATT_F_READ_ENC == BLE_GATT_CHR_F_READ_ENC, "flag mismatch");
_Static_assert(DYN_GATT_F_READ_AUTHEN == BLE_GATT_CHR_F_READ_AUTHEN, "flag mismatch");
_Static_assert(DYN_GATT_F_WRITE_ENC == BLE_GATT_CHR_F_WRITE_ENC, "flag mismatch");
_Static_assert(DYN_GATT_F_WRITE_AUTHEN == BLE_GATT_CHR_F_WRITE_AUTHEN, "flag mismatch");

#define DYN_GATT_FLAGS_ALLOWED                                                                  \
    (DYN_GATT_F_BROADCAST | DYN_GATT_F_READ | DYN_GATT_F_WRITE_NO_RSP | DYN_GATT_F_WRITE |       \
     DYN_GATT_F_NOTIFY | DYN_GATT_F_INDICATE | DYN_GATT_F_AUTH_SIGN_WRITE | DYN_GATT_F_READ_ENC | \
     DYN_GATT_F_READ_AUTHEN | DYN_GATT_F_READ_AUTHOR | DYN_GATT_F_WRITE_ENC |                     \
     DYN_GATT_F_WRITE_AUTHEN | DYN_GATT_F_WRITE_AUTHOR)

typedef struct {
    bool used;
    ble_uuid_any_t uuid;
    uint16_t flags;
    uint16_t max_len;
    uint8_t* value; /* max_len bytes; NULL only for a fully free slot */
    uint16_t len;
    uint16_t val_handle; /* written by ble_gatts_start; 0 until registered */
    bool has_dsc;
    ble_uuid_any_t dsc_uuid;
    uint8_t dsc_value[DYN_GATT_DSC_VALUE_MAX];
    uint8_t dsc_len;
} DynChr;

typedef struct {
    bool used;
    bool primary;
    ble_uuid_any_t uuid;
    DynChr chrs[DYN_GATT_MAX_CHRS_PER_SVC];
    struct ble_gatt_chr_def chr_defs[DYN_GATT_MAX_CHRS_PER_SVC + 1];
    struct ble_gatt_dsc_def dsc_defs[DYN_GATT_MAX_CHRS_PER_SVC][2];
} DynSvc;

static DynSvc* s_svcs[DYN_GATT_MAX_SVCS];
static struct ble_gatt_svc_def s_svc_defs[DYN_GATT_MAX_SVCS + 1];
static DynGattHooks s_hooks;
static FuriMutex* s_mutex;
static volatile bool s_dirty;
static volatile bool s_live;

static void dyn_lock(void) {
    furi_mutex_acquire(s_mutex, FuriWaitForever);
}

static void dyn_unlock(void) {
    furi_mutex_release(s_mutex);
}

static bool dyn_to_nimble_uuid(const DynGattUuid* in, ble_uuid_any_t* out) {
    memset(out, 0, sizeof(*out));
    if(in->type == 16) {
        out->u16.u.type = BLE_UUID_TYPE_16;
        out->u16.value = in->u16;
        return true;
    }
    if(in->type == 128) {
        out->u128.u.type = BLE_UUID_TYPE_128;
        memcpy(out->u128.value, in->u128, 16);
        return true;
    }
    return false;
}

static int dyn_chr_id(int svc, int chr) {
    return svc * DYN_GATT_MAX_CHRS_PER_SVC + chr;
}

static DynChr* dyn_chr_at(int chr_id) {
    if(chr_id < 0) return NULL;
    int svc = chr_id / DYN_GATT_MAX_CHRS_PER_SVC;
    int idx = chr_id % DYN_GATT_MAX_CHRS_PER_SVC;
    if(svc >= DYN_GATT_MAX_SVCS || !s_svcs[svc]) return NULL;
    return &s_svcs[svc]->chrs[idx];
}

/* ---- NimBLE access callbacks (host thread, or the notifying thread) ------- */

static int dyn_chr_access(
    uint16_t conn_handle,
    uint16_t attr_handle,
    struct ble_gatt_access_ctxt* ctxt,
    void* arg) {
    int chr_id = (int)(uintptr_t)arg;
    DynChr* c = dyn_chr_at(chr_id);
    if(!c || !c->used) return BLE_ATT_ERR_UNLIKELY;

    switch(ctxt->op) {
    case BLE_GATT_ACCESS_OP_READ_CHR: {
        dyn_lock();
        int rc = os_mbuf_append(ctxt->om, c->value, c->len);
        dyn_unlock();
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    case BLE_GATT_ACCESS_OP_WRITE_CHR: {
        uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
        if(len > c->max_len) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        uint8_t buf[DYN_GATT_VALUE_MAX];
        if(len && os_mbuf_copydata(ctxt->om, 0, len, buf) != 0) return BLE_ATT_ERR_UNLIKELY;
        dyn_lock();
        memcpy(c->value, buf, len);
        c->len = len;
        dyn_unlock();
        if(s_hooks.on_write)
            s_hooks.on_write(
                chr_id, DynGattWriteValue, conn_handle, attr_handle, buf, len, s_hooks.ctx);
        return 0;
    }
    default:
        return BLE_ATT_ERR_UNLIKELY;
    }
}

static int dyn_dsc_access(
    uint16_t conn_handle,
    uint16_t attr_handle,
    struct ble_gatt_access_ctxt* ctxt,
    void* arg) {
    UNUSED(conn_handle);
    UNUSED(attr_handle);
    DynChr* c = dyn_chr_at((int)(uintptr_t)arg);
    if(!c || !c->used || !c->has_dsc) return BLE_ATT_ERR_UNLIKELY;
    if(ctxt->op != BLE_GATT_ACCESS_OP_READ_DSC) return BLE_ATT_ERR_READ_NOT_PERMITTED;
    int rc = os_mbuf_append(ctxt->om, c->dsc_value, c->dsc_len);
    return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

/* ---- Definition API -------------------------------------------------------- */

void dyn_gatt_init(void) {
    if(!s_mutex) s_mutex = furi_mutex_alloc(FuriMutexTypeNormal);
}

void dyn_gatt_set_hooks(const DynGattHooks* hooks) {
    if(hooks) {
        s_hooks = *hooks;
    } else {
        memset(&s_hooks, 0, sizeof(s_hooks));
    }
}

int dyn_gatt_service_add(const DynGattUuid* uuid, bool primary) {
    if(!uuid || !s_mutex) return -1;
    ble_uuid_any_t nuuid;
    if(!dyn_to_nimble_uuid(uuid, &nuuid)) return -1;

    dyn_lock();
    int slot = -1;
    for(int i = 0; i < DYN_GATT_MAX_SVCS; i++) {
        /* Only fully free slots: a removed-but-live block is still referenced. */
        if(!s_svcs[i]) {
            slot = i;
            break;
        }
    }
    if(slot >= 0) {
        DynSvc* s = malloc(sizeof(DynSvc));
        memset(s, 0, sizeof(DynSvc));
        s->used = true;
        s->primary = primary;
        s->uuid = nuuid;
        s_svcs[slot] = s;
        s_dirty = true;
    }
    dyn_unlock();
    if(slot < 0) FURI_LOG_E(TAG, "no free service slot (max %d)", DYN_GATT_MAX_SVCS);
    return slot;
}

bool dyn_gatt_service_remove(int svc_id) {
    if(svc_id < 0 || svc_id >= DYN_GATT_MAX_SVCS) return false;
    dyn_lock();
    DynSvc* s = s_svcs[svc_id];
    bool ok = s && s->used;
    if(ok) {
        s->used = false;
        for(int j = 0; j < DYN_GATT_MAX_CHRS_PER_SVC; j++) s->chrs[j].used = false;
        s_dirty = true;
    }
    dyn_unlock();
    return ok;
}

int dyn_gatt_char_add(
    int svc_id,
    const DynGattUuid* uuid,
    uint16_t flags,
    uint16_t max_len,
    const uint8_t* init_value,
    uint16_t init_len,
    const DynGattUuid* dsc_uuid,
    const uint8_t* dsc_value,
    uint8_t dsc_len) {
    if(svc_id < 0 || svc_id >= DYN_GATT_MAX_SVCS || !uuid) return -1;
    ble_uuid_any_t nuuid;
    if(!dyn_to_nimble_uuid(uuid, &nuuid)) return -1;
    ble_uuid_any_t ndsc;
    if(dsc_uuid && !dyn_to_nimble_uuid(dsc_uuid, &ndsc)) return -1;
    if(dsc_len > DYN_GATT_DSC_VALUE_MAX) dsc_len = DYN_GATT_DSC_VALUE_MAX;
    if(max_len == 0) max_len = 1;
    if(max_len > DYN_GATT_VALUE_MAX) max_len = DYN_GATT_VALUE_MAX;
    if(init_len > max_len) init_len = max_len;

    uint8_t* value = malloc(max_len);
    memset(value, 0, max_len);
    if(init_value && init_len) memcpy(value, init_value, init_len);

    dyn_lock();
    DynSvc* s = s_svcs[svc_id];
    int slot = -1;
    if(s && s->used) {
        for(int j = 0; j < DYN_GATT_MAX_CHRS_PER_SVC; j++) {
            if(!s->chrs[j].used && !s->chrs[j].value) {
                slot = j;
                break;
            }
        }
    }
    if(slot >= 0) {
        DynChr* c = &s->chrs[slot];
        memset(c, 0, sizeof(*c));
        c->used = true;
        c->uuid = nuuid;
        c->flags = flags & DYN_GATT_FLAGS_ALLOWED;
        c->max_len = max_len;
        c->value = value;
        c->len = init_len;
        c->val_handle = 0;
        if(dsc_uuid) {
            c->has_dsc = true;
            c->dsc_uuid = ndsc;
            if(dsc_value && dsc_len) memcpy(c->dsc_value, dsc_value, dsc_len);
            c->dsc_len = dsc_len;
        }
        s_dirty = true;
    }
    dyn_unlock();

    if(slot < 0) {
        free(value);
        FURI_LOG_E(TAG, "no free characteristic slot in service %d", svc_id);
        return -1;
    }
    return dyn_chr_id(svc_id, slot);
}

bool dyn_gatt_char_remove(int chr_id) {
    dyn_lock();
    DynChr* c = dyn_chr_at(chr_id);
    bool ok = c && c->used;
    if(ok) {
        c->used = false; /* value stays allocated until the next rebuild */
        s_dirty = true;
    }
    dyn_unlock();
    return ok;
}

bool dyn_gatt_char_set_value(int chr_id, const uint8_t* data, uint16_t len) {
    uint16_t val_handle = 0;
    dyn_lock();
    DynChr* c = dyn_chr_at(chr_id);
    bool ok = c && c->used && len <= c->max_len && (len == 0 || data);
    if(ok) {
        if(len) memcpy(c->value, data, len);
        c->len = len;
        val_handle = c->val_handle;
    }
    dyn_unlock();
    /* Outside the lock: chr_updated reads the value back through dyn_chr_access. */
    if(ok && s_live && val_handle) ble_gatts_chr_updated(val_handle);
    return ok;
}

bool dyn_gatt_char_handles(int chr_id, uint16_t* decl_handle, uint16_t* dsc_handle) {
    dyn_lock();
    DynChr* c = dyn_chr_at(chr_id);
    bool ok = c && c->used && c->val_handle;
    if(ok) {
        if(decl_handle) *decl_handle = c->val_handle - 1;
        if(dsc_handle) {
            /* Value, then CCCD when notify/indicate, then our descriptor. */
            uint16_t cccd = (c->flags & (DYN_GATT_F_NOTIFY | DYN_GATT_F_INDICATE)) ? 1 : 0;
            *dsc_handle = c->has_dsc ? (uint16_t)(c->val_handle + 1 + cccd) : 0;
        }
    }
    dyn_unlock();
    return ok;
}

bool dyn_gatt_dirty(void) {
    return s_dirty;
}

/* ---- Rebuild (nimble_glue, host thread) ------------------------------------ */

void dyn_gatt_register_all(void) {
    s_live = false;
    int n = 0;

    dyn_lock();
    /* Clear dirty at the snapshot, not after start: a definition that changes
     * while the rebuild runs sets it again and triggers another rebuild. */
    s_dirty = false;
    for(int i = 0; i < DYN_GATT_MAX_SVCS; i++) {
        DynSvc* s = s_svcs[i];
        if(!s) continue;

        /* The table was just reset: removed definitions can be freed now. */
        for(int j = 0; j < DYN_GATT_MAX_CHRS_PER_SVC; j++) {
            DynChr* c = &s->chrs[j];
            c->val_handle = 0;
            if(!c->used && c->value) {
                free(c->value);
                memset(c, 0, sizeof(*c));
            }
        }
        if(!s->used) {
            free(s);
            s_svcs[i] = NULL;
            continue;
        }

        int k = 0;
        for(int j = 0; j < DYN_GATT_MAX_CHRS_PER_SVC; j++) {
            DynChr* c = &s->chrs[j];
            if(!c->used) continue;
            struct ble_gatt_chr_def* d = &s->chr_defs[k++];
            memset(d, 0, sizeof(*d));
            d->uuid = &c->uuid.u;
            d->access_cb = dyn_chr_access;
            d->arg = (void*)(uintptr_t)dyn_chr_id(i, j);
            d->flags = c->flags;
            d->val_handle = &c->val_handle;
            if(c->has_dsc) {
                struct ble_gatt_dsc_def* dd = s->dsc_defs[j];
                memset(dd, 0, 2 * sizeof(*dd));
                dd[0].uuid = &c->dsc_uuid.u;
                dd[0].att_flags = BLE_ATT_F_READ;
                dd[0].access_cb = dyn_dsc_access;
                dd[0].arg = (void*)(uintptr_t)dyn_chr_id(i, j);
                d->descriptors = dd;
            }
        }
        memset(&s->chr_defs[k], 0, sizeof(s->chr_defs[k]));

        struct ble_gatt_svc_def* sd = &s_svc_defs[n++];
        memset(sd, 0, sizeof(*sd));
        sd->type = s->primary ? BLE_GATT_SVC_TYPE_PRIMARY : BLE_GATT_SVC_TYPE_SECONDARY;
        sd->uuid = &s->uuid.u;
        sd->characteristics = k ? s->chr_defs : NULL;
    }
    memset(&s_svc_defs[n], 0, sizeof(s_svc_defs[n]));
    dyn_unlock();

    if(n == 0) return;
    /* Outside the lock: these take the NimBLE host lock. */
    int rc = ble_gatts_count_cfg(s_svc_defs);
    if(rc == 0) rc = ble_gatts_add_svcs(s_svc_defs);
    FURI_LOG_I(TAG, "registered %d dynamic service(s), rc=%d", n, rc);
}

void dyn_gatt_after_start(void) {
    s_live = true;
    if(s_hooks.on_committed) s_hooks.on_committed(s_hooks.ctx);
}

void dyn_gatt_on_subscribe(uint16_t conn_handle, uint16_t attr_handle, bool notify, bool indicate) {
    int chr_id = -1;
    dyn_lock();
    for(int i = 0; i < DYN_GATT_MAX_SVCS && chr_id < 0; i++) {
        if(!s_svcs[i]) continue;
        for(int j = 0; j < DYN_GATT_MAX_CHRS_PER_SVC; j++) {
            DynChr* c = &s_svcs[i]->chrs[j];
            if(c->used && c->val_handle && c->val_handle == attr_handle) {
                chr_id = dyn_chr_id(i, j);
                break;
            }
        }
    }
    dyn_unlock();
    if(chr_id < 0 || !s_hooks.on_write) return;
    uint8_t cccd[2] = {(uint8_t)((notify ? 0x01 : 0) | (indicate ? 0x02 : 0)), 0x00};
    s_hooks.on_write(
        chr_id, DynGattWriteCccd, conn_handle, attr_handle + 1, cccd, sizeof(cccd), s_hooks.ctx);
}

void dyn_gatt_on_notify_tx(uint16_t conn_handle, uint16_t attr_handle, int status, bool indication) {
    if(!indication || status != BLE_HS_EDONE || !s_hooks.on_indicate_done) return;
    int chr_id = -1;
    dyn_lock();
    for(int i = 0; i < DYN_GATT_MAX_SVCS && chr_id < 0; i++) {
        if(!s_svcs[i]) continue;
        for(int j = 0; j < DYN_GATT_MAX_CHRS_PER_SVC; j++) {
            DynChr* c = &s_svcs[i]->chrs[j];
            if(c->used && c->val_handle && c->val_handle == attr_handle) {
                chr_id = dyn_chr_id(i, j);
                break;
            }
        }
    }
    dyn_unlock();
    if(chr_id >= 0) s_hooks.on_indicate_done(chr_id, conn_handle, s_hooks.ctx);
}
