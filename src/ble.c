#include <freertos/FreeRTOSConfig.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <nvs_flash.h>
#include <esp_log.h>
#include <host/ble_uuid.h>
#include <services/gap/ble_svc_gap.h>
#include <services/gatt/ble_svc_gatt.h>
#include <nimble/ble.h>
#include <nimble/nimble_port.h>
#include <nimble/nimble_port_freertos.h>
#include "ble.h"

#define ENCODER_SERVICE_UUID 0xF1F1
#define ENCODER_UPDATE_UUID 0xF1F2

static const char *TAG = "ble_task";
static const char *DEVICE_NAME = "gas_on";
bool notify_state = false;

static uint8_t ble_addr_type;
static void ble_on_sync(void);
static void ble_on_reset(int reason);
static void ble_advertise(void);
static int ble_gap_event(struct ble_gap_event *event, void *arg);
static int gatt_svr_init(void);
static int gatt_svr_clbk(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);

uint16_t conn_handle;
uint16_t encoder_handle;


static void ble_host_task(void *param) {
    ESP_LOGI(TAG, "BLE Host Task Started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static const struct ble_gatt_svc_def GATT_SVR_SVCS[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(ENCODER_SERVICE_UUID),
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                .uuid = BLE_UUID16_DECLARE(ENCODER_UPDATE_UUID),
                .access_cb = gatt_svr_clbk,
                .val_handle = &encoder_handle,
                .flags = BLE_GATT_CHR_F_NOTIFY,
            },
            {
                0,
            },
        },
    },
    {
        0,
    },
};

uint8_t ble_init(void) {
    ESP_LOGI(TAG, "Initializing BLE");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init nimble %d", ret);
        return 1;
    }

    ble_hs_cfg.sync_cb = ble_on_sync;
    ble_hs_cfg.reset_cb = ble_on_reset;

    int rc = gatt_svr_init();
    assert(rc == 0);

    rc = ble_svc_gap_device_name_set(DEVICE_NAME);
    assert(rc == 0);

    nimble_port_freertos_init(ble_host_task);

    ESP_LOGI(TAG, "BLE successfully initialized");
    return 0;
}

static void ble_on_sync(void) {
    int rc = ble_hs_id_infer_auto(0, &ble_addr_type);
    assert(rc == 0);

    uint8_t addr_val[6] = {0};
    rc = ble_hs_id_copy_addr(ble_addr_type, addr_val, NULL);
    assert(rc == 0);

    ESP_LOGI(TAG, "Device Address: %02x:%02x:%02x:%02x:%02x:%02x", addr_val[5], addr_val[4], addr_val[3], addr_val[2], addr_val[1], addr_val[0]);

    ble_advertise();
}

static void ble_on_reset(int reason) {
    ESP_LOGE(TAG, "Resetting state; reason=%d", reason);
}

static void ble_advertise(void) {
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    fields.uuids16 = (ble_uuid16_t[]){
        BLE_UUID16_INIT(ENCODER_SERVICE_UUID),
    };
    fields.num_uuids16 = 1;
    fields.uuids16_is_complete = 1;

    fields.name = (uint8_t *)DEVICE_NAME;
    fields.name_len = strlen(DEVICE_NAME);
    fields.name_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "error setting advertisement data; rc=%d", rc);
        return;
    }

    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "error enabling advertisement; rc=%d", rc);
    }
}

static int ble_gap_event(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            ESP_LOGI(TAG, "connection %s; status=%d", event->connect.status == 0 ? "established" : "failed", event->connect.status);
            if (event->connect.status != 0) {
                ble_advertise();
            }
            conn_handle = event->connect.conn_handle;
            break;

        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "disconnect; reason=%d", event->disconnect.reason);
            ble_advertise();
            break;

        case BLE_GAP_EVENT_ADV_COMPLETE:
            ESP_LOGI(TAG, "adv complete");
            ble_advertise();
            break;

        case BLE_GAP_EVENT_SUBSCRIBE:
            ESP_LOGI(TAG, "subscribe event; cur_notify=%d\n val_handle=%d", event->subscribe.cur_notify, encoder_handle);
            notify_state = event->subscribe.cur_notify;
            break;

        case BLE_GAP_EVENT_MTU:
            ESP_LOGI(TAG, "mtu update event; conn_handle=%d mtu=%d", event->mtu.conn_handle, event->mtu.value);
            break;
    }

    return 0;
}

static int gatt_svr_init(void) {
    ble_svc_gap_init();
    ble_svc_gatt_init();

    int rc = ble_gatts_count_cfg(GATT_SVR_SVCS);
    if (rc != 0) {
        return rc;
    }

    rc = ble_gatts_add_svcs(GATT_SVR_SVCS);
    return rc;
}

static int gatt_svr_clbk(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    assert(0);
    return BLE_ATT_ERR_UNLIKELY;
}
