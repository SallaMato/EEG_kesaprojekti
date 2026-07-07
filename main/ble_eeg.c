#include "ble_eeg.h"

#include <string.h>

#include "esp_log.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"

#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "BLE_EEG";


// ======================================================
// UUID:t
// ======================================================

#define EEG_SERVICE_UUID      0xFFF0
#define EEG_CHARACTERISTIC_UUID 0xFFF1
#define BATCH_SIZE 5


// ======================================================
// Yhteystila
// ======================================================

static uint16_t g_conn_handle = BLE_HS_CONN_HANDLE_NONE;

static uint16_t g_char_handle = 0;

static bool g_notifications_enabled = false;


// ======================================================
// EEG datapaketti
// ======================================================

typedef struct
{
    uint32_t sample;

    int32_t ch[8];

} eeg_packet_t;


// ======================================================
// Forward declarations
// ======================================================

static int ble_gap_event(struct ble_gap_event *event, void *arg);

static void ble_advertise(void);

static void ble_host_task(void *param);

// ======================================================
// GATT Access callback
// ======================================================

static int eeg_access_cb(uint16_t conn_handle,
                         uint16_t attr_handle,
                         struct ble_gatt_access_ctxt *ctxt,
                         void *arg)
{
    return 0;
}

// ======================================================
// GATT Service
// ======================================================

static const struct ble_gatt_svc_def gatt_svcs[] =
{
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,

        .uuid = BLE_UUID16_DECLARE(EEG_SERVICE_UUID),

        .characteristics =
        (struct ble_gatt_chr_def[])
        {
            {
                .uuid = BLE_UUID16_DECLARE(EEG_CHARACTERISTIC_UUID),

                .access_cb = eeg_access_cb,

                .flags =
                    BLE_GATT_CHR_F_NOTIFY |
                    BLE_GATT_CHR_F_READ,

                .val_handle = &g_char_handle,
            },

            {0}
        }
    },

    {0}
};

// ======================================================
// Host task
// ======================================================

static void ble_host_task(void *param)
{
    nimble_port_run();

    nimble_port_freertos_deinit();
}

// ======================================================
// Sync callback
// ======================================================

static void ble_on_sync(void)
{
    ESP_LOGI(TAG, "BLE stack ready");

    ble_advertise();
}

// ======================================================
// BLE Init
// ======================================================

void ble_init(void)
{
    esp_err_t ret;

    ret = nimble_port_init();

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "nimble_port_init failed");

        return;
    }

    ble_hs_cfg.sync_cb = ble_on_sync;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    ble_gatts_count_cfg(gatt_svcs);
    ble_gatts_add_svcs(gatt_svcs);

    ble_svc_gap_device_name_set("OpenEEG");

    nimble_port_freertos_init(ble_host_task);

    ESP_LOGI(TAG, "BLE initialized");
}

// ======================================================
// Advertising
// ======================================================

static void ble_advertise(void)
{
    struct ble_gap_adv_params adv_params;

    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    struct ble_hs_adv_fields fields;

    memset(&fields, 0, sizeof(fields));

    fields.flags =
        BLE_HS_ADV_F_DISC_GEN |
        BLE_HS_ADV_F_BREDR_UNSUP;

    const char *name = ble_svc_gap_device_name();

    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    ble_gap_adv_set_fields(&fields);

    ble_gap_adv_start(
        BLE_OWN_ADDR_PUBLIC,
        NULL,
        BLE_HS_FOREVER,
        &adv_params,
        ble_gap_event,
        NULL);

    ESP_LOGI(TAG, "Advertising started");
}

// ======================================================
// GAP events
// ======================================================

static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type)
    {

    case BLE_GAP_EVENT_CONNECT:

        if (event->connect.status == 0)
        {
            g_conn_handle = event->connect.conn_handle;

            ESP_LOGI(TAG, "BLE connected (handle=%d)", g_conn_handle);
        }
        else
        {
            ESP_LOGI(TAG, "Connection failed");

            ble_advertise();
        }

        return 0;


    case BLE_GAP_EVENT_DISCONNECT:

        ESP_LOGI(TAG, "BLE disconnected");

        g_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        g_notifications_enabled = false;

        ble_advertise();

        return 0;


    case BLE_GAP_EVENT_SUBSCRIBE:

        if (event->subscribe.attr_handle == g_char_handle)
        {
            g_notifications_enabled = event->subscribe.cur_notify;

            ESP_LOGI(TAG,
                     "Notifications %s",
                     g_notifications_enabled ? "enabled" : "disabled");
        }

        return 0;


    case BLE_GAP_EVENT_MTU:

        ESP_LOGI(TAG,
                 "MTU updated = %d",
                 event->mtu.value);

        return 0;


    default:
        return 0;
    }
}

// ======================================================
// Send EEG sample
// ======================================================

void ble_send_sample(int32_t ch2, int32_t ch3)
{
    static uint32_t counter = 0;

    if ((counter++ % 250) == 0)
    {
        ESP_LOGI(TAG,
             "conn=%d notify=%d",
             g_conn_handle,
             g_notifications_enabled);
    }
    if (g_conn_handle == BLE_HS_CONN_HANDLE_NONE)
        return;

    if (!g_notifications_enabled)
        return;

    // --- UUSI PUSKUROINTILOGIIKKA ALKAA ---
    static eeg_packet_t batch_buffer[BATCH_SIZE];
    static int batch_idx = 0;
    static uint32_t sample_counter = 0;

    // Täytetään puskurista seuraava vapaa paikka
    batch_buffer[batch_idx].sample = sample_counter++;
    batch_buffer[batch_idx].ch[0] = ch2;
    batch_buffer[batch_idx].ch[1] = ch3;
    
    // Nollataan loput kanavat (koodisi on valmis 8 kanavalle)
    for (int i = 2; i < 8; i++) {
        batch_buffer[batch_idx].ch[i] = 0;
    }
    
    batch_idx++;

    // Lähetetään vasta, kun puskuri on täynnä (esim. joka 5. näyte)
    if (batch_idx >= BATCH_SIZE) 
    {
        struct os_mbuf *om =
            ble_hs_mbuf_from_flat(
                batch_buffer,
                sizeof(batch_buffer)); // Lähetetään koko 5 näytteen taulukko (180 tavua)

        if (om == NULL) {
            batch_idx = 0; // Nollaus varmuuden vuoksi
            return;
        }

        int rc = ble_gatts_notify_custom(
                    g_conn_handle,
                    g_char_handle,
                    om);

        if (rc != 0)
        {
            ESP_LOGW(TAG,
                     "notify failed rc=%d",
                     rc);
        }

        // Tyhjennetään puskuri-indeksi seuraavaa erää varten
        batch_idx = 0;
    }
    // --- UUSI PUSKUROINTILOGIIKKA PÄÄTTYY ---
}