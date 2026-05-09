#include "bt_sender.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include <string.h>

/* NimBLE Includes */
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "BLE_SERVER";

static uint16_t conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t sensor_char_val_handle;
static ble_subscription_cb_t sub_callback = NULL;

/* Custom 128-bit UUIDs for Service and Characteristic */
/* Service: 5c1b9a0d-b5be-4a40-8f7a-66b36d0a5176 */
static const ble_uuid128_t sensor_svc_uuid =
    BLE_UUID128_INIT(0x76, 0x51, 0x0a, 0x6d, 0xb3, 0x66, 0x7a, 0x8f,
                     0x40, 0x4a, 0xbe, 0xb5, 0x0d, 0x9a, 0x1b, 0x5c);

/* Characteristic: 5c1b9a0d-b5be-4a40-8f7a-66b36d0a5177 */
static const ble_uuid128_t sensor_chr_uuid =
    BLE_UUID128_INIT(0x77, 0x51, 0x0a, 0x6d, 0xb3, 0x66, 0x7a, 0x8f,
                     0x40, 0x4a, 0xbe, 0xb5, 0x0d, 0x9a, 0x1b, 0x5c);

static void ble_server_advertise(void);

/* Characteristic Access Callback */
static int sensor_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    /* We primarily push data via NOTIFY. */
    return 0; 
}

/* GATT Server Definition */
static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &sensor_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &sensor_chr_uuid.u,
                .access_cb = sensor_chr_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &sensor_char_val_handle,
            },
            {
                0, /* No more characteristics */
            }
        },
    },
    {
        0, /* No more services */
    },
};

/* GAP Event Callback: Handles connections, disconnections, and subscriptions */
static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                ESP_LOGI(TAG, "Client connected!");
                conn_handle = event->connect.conn_handle;
            } else {
                ESP_LOGE(TAG, "Connection failed; status=%d", event->connect.status);
                ble_server_advertise(); /* Resume advertising */
            }
            break;

        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "Client disconnected!");
            conn_handle = BLE_HS_CONN_HANDLE_NONE;
            if (sub_callback) sub_callback(false); /* Notify main to PAUSE reading */
            ble_server_advertise(); /* Resume advertising */
            break;

        case BLE_GAP_EVENT_SUBSCRIBE:
            /* Check if they subscribed to our specific sensor characteristic */
            if (event->subscribe.attr_handle == sensor_char_val_handle) {
                bool is_subscribed = (event->subscribe.cur_notify != 0);
                ESP_LOGI(TAG, "Client %s to notifications!", is_subscribed ? "subscribed" : "unsubscribed");
                
                if (sub_callback) sub_callback(is_subscribed); /* Notify main to START/STOP reading */
            }
            break;
    }
    return 0;
}

/* Configure and start advertising */
static void ble_server_advertise(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));

    /* Advertise as Discoverable and BLE only (no Classic Bluetooth) */
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    
    fields.name = (const uint8_t *)"ESP32_IMU";
    fields.name_len = strlen("ESP32_IMU");
    fields.name_is_complete = 1;

    ble_gap_adv_set_fields(&fields);

    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND; /* Undirected connectable */
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN; /* General discoverable */

    ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
}

/* Callback when NimBLE host and controller are synced */
static void ble_on_sync(void)
{
    ble_svc_gap_device_name_set("ESP32_IMU");
    ble_server_advertise();
    ESP_LOGI(TAG, "BLE Host Synced and Advertising started");
}

/* The FreeRTOS task that runs the NimBLE host stack */
static void ble_host_task(void *param)
{
    ESP_LOGI(TAG, "BLE Host Task Started");
    nimble_port_run(); 
    nimble_port_freertos_deinit();
}

void ble_server_init(ble_subscription_cb_t cb)
{
    sub_callback = cb;

    /* Initialize NVS — it is required to store BLE security & identity data */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(nimble_port_init());

    /* Initialize standard GAP and GATT services */
    ble_svc_gap_init();
    ble_svc_gatt_init();

    /* Load our custom GATT Server configuration */
    ble_gatts_count_cfg(gatt_svr_svcs);
    ble_gatts_add_svcs(gatt_svr_svcs);

    /* Set the sync callback */
    ble_hs_cfg.sync_cb = ble_on_sync;

    /* Start the host task */
    nimble_port_freertos_init(ble_host_task);
}

void ble_server_send_sensor_msg(sensorMsg *msg)
{
    if (conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        /* Allocate a memory buffer (mbuf) with the data to send */
        struct os_mbuf *om = ble_hs_mbuf_from_flat(msg->bytes, sizeof(msg->bytes));
        if (om) {
            /* Push the notification to the client */
            ble_gatts_notify_custom(conn_handle, sensor_char_val_handle, om);
        } else {
            ESP_LOGW(TAG, "Failed to allocate mbuf for BLE notification");
        }
    }
}