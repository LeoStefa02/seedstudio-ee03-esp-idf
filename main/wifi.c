#include "wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include <string.h>

#define WIFI_MAXIMUM_RETRIES    5

// Event group bits
#define WIFI_CONNECTED_BIT      BIT0
#define WIFI_FAILED_BIT         BIT1

static const char         *TAG            = "WIFI";
static EventGroupHandle_t  s_wifi_events  = NULL;
static int                 s_retry_count  = 0;
static char                s_ip_str[16]   = {0};

// ----------------------------------------------------------
// Event handler — called by WiFi/IP stack on state changes
// ----------------------------------------------------------
static void wifi_event_handler(void            *arg,
                               esp_event_base_t event_base,
                               int32_t          event_id,
                               void            *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        // Station started — attempt connection
        esp_wifi_connect();
        ESP_LOGI(TAG, "Station started, connecting...");

    } else if (event_base == WIFI_EVENT &&
               event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_count < WIFI_MAXIMUM_RETRIES) {
            esp_wifi_connect();
            s_retry_count++;
            ESP_LOGW(TAG, "Retrying... (%d/%d)",
                     s_retry_count, WIFI_MAXIMUM_RETRIES);
        } else {
            ESP_LOGE(TAG, "Max retries reached — giving up");
            xEventGroupSetBits(s_wifi_events, WIFI_FAILED_BIT);
        }

    } else if (event_base == IP_EVENT &&
               event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        // Store IP string for later retrieval
        snprintf(s_ip_str, sizeof(s_ip_str), IPSTR,
                 IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "Connected! IP: %s", s_ip_str);
        s_retry_count = 0;
        xEventGroupSetBits(s_wifi_events, WIFI_CONNECTED_BIT);
    }
}

esp_err_t wifi_connect(const char *ssid, const char *password)
{
    // --- Step 1: NVS flash init (required by WiFi stack) ---
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated — erase and retry
        ESP_LOGW(TAG, "NVS truncated, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    // --- Step 2: TCP/IP stack + event loop ---
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    // --- Step 3: WiFi driver init ---
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // --- Step 4: Event group for synchronisation ---
    s_wifi_events = xEventGroupCreate();

    // --- Step 5: Register event handlers ---
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID,
        &wifi_event_handler, NULL, NULL));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP,
        &wifi_event_handler, NULL, NULL));

    // --- Step 6: Configure STA mode ---
    wifi_config_t wifi_cfg = {0};
    strncpy((char *)wifi_cfg.sta.ssid,     ssid,     sizeof(wifi_cfg.sta.ssid)     - 1);
    strncpy((char *)wifi_cfg.sta.password, password, sizeof(wifi_cfg.sta.password) - 1);
    // Use WPA2/WPA3 — most home routers
    wifi_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));

    // --- Step 7: Start WiFi ---
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_LOGI(TAG, "Connecting to SSID: %s", ssid);

    // --- Step 8: Block until connected or failed ---
    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_events,
        WIFI_CONNECTED_BIT | WIFI_FAILED_BIT,
        pdFALSE,            // do not clear bits
        pdFALSE,            // wait for ANY bit
        pdMS_TO_TICKS(30000) // 30 second timeout
    );

    if (bits & WIFI_CONNECTED_BIT) {
        return ESP_OK;
    } else if (bits & WIFI_FAILED_BIT) {
        ESP_LOGE(TAG, "Failed to connect to SSID: %s", ssid);
        return ESP_FAIL;
    } else {
        ESP_LOGE(TAG, "Connection timed out after 30 seconds");
        return ESP_ERR_TIMEOUT;
    }
}

const char *wifi_get_ip(void)
{
    return s_ip_str;
}