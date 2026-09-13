#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

// ESP-IDF headers
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "mdns.h"

#include "esp_log.h"
#include "esp_system.h" // System initialization and management
#include "esp_event.h"  // Event loop library (like an interrupt but for software events)
#include "nvs_flash.h"  // NVS initialization

#include "esp_netif.h" // Network interface
#include "esp_wifi.h"  // Wi-Fi initialization
#include "wifi_provisioning/manager.h"
#include "wifi_provisioning/scheme_softap.h"

// User defined headers
#include "network.h"

static const char *TAG = "network";

EventGroupHandle_t wifi_event_group;
const int WIFI_CONNECTED_EVENT = BIT0; // Event bit for Wi-Fi connection status

// Static function prototypes (internal to this file)
static void get_device_service_name(char *service_name, size_t max);

static bool mdns_started = false;

static void start_mdns(void)
{
    if (mdns_started)
        return;

    ESP_ERROR_CHECK(mdns_init());
    ESP_ERROR_CHECK(mdns_hostname_set("thermo"));
    ESP_ERROR_CHECK(mdns_instance_name_set("NEWT-ESP-WIFI Thermostat"));

    ESP_ERROR_CHECK(mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0));

    mdns_started = true;

    ESP_LOGI(TAG, "mDNS started: http://thermo.local");
}

static void wifi_event_handler(void *args, esp_event_base_t event_base,
                               int32_t event_id, void *event_data);

void wifi_init()
{
    ESP_LOGI(TAG, "Initializing Wi-Fi stack");
    ESP_ERROR_CHECK(esp_netif_init()); // TCP/IP stack initialization

    ESP_ERROR_CHECK(esp_event_loop_create_default()); // Create event loop for Wi-Fi and IP events
    wifi_event_group = xEventGroupCreate();

    // Register event handlers for Wi-Fi and IP events
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_PROV_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    esp_netif_t *netif_sta = esp_netif_create_default_wifi_sta(); // Set up default Wi-Fi station interface
    assert(netif_sta != NULL);
    esp_netif_t *netif_ap = esp_netif_create_default_wifi_ap(); // Set up default Wi-Fi softAP interface
    assert(netif_ap != NULL);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_prov_mgr_config_t prov_cfg = {
        .scheme = wifi_prov_scheme_softap,
        .scheme_event_handler = WIFI_PROV_EVENT_HANDLER_NONE,
    };
    ESP_ERROR_CHECK(wifi_prov_mgr_init(prov_cfg)); // Initialize Wi-Fi provisioning

    bool provisioned = false;
    ESP_ERROR_CHECK(wifi_prov_mgr_is_provisioned(&provisioned));

    if (!provisioned)
    {
        ESP_LOGI(TAG, "Device is not provisioned, starting provisioning service");

        char service_name[12];
        get_device_service_name(service_name, sizeof(service_name));

        wifi_prov_security_t security = WIFI_PROV_SECURITY_0; // No security for simplicity
        const void *wifi_prov_sec_params = NULL;              // No security parameters for simplicity
        const char *service_key = NULL;                       // No service key for simplicity

        ESP_ERROR_CHECK(wifi_prov_mgr_start_provisioning(security, wifi_prov_sec_params, service_name, service_key));
    }
    else
    {
        ESP_LOGI(TAG, "Device is already provisioned, connecting to Wi-Fi");
        wifi_prov_mgr_deinit(); // Deinitialize provisioning manager because it's no longer needed

        // Start Wi-Fi in station mode
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_start());
    }
}

bool wifi_connected(TickType_t timeout)
{
    EventBits_t bits = xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_EVENT, false, true, timeout);
    bool connected = (bits & WIFI_CONNECTED_EVENT) != 0;

    if (connected)
        ESP_LOGI(TAG, "Connected!");
    else
        ESP_LOGW(TAG, "Timed out.");

    return connected;
}

static void get_device_service_name(char *service_name, size_t max)
{
    uint8_t eth_mac[6];
    const char *ssid_prefix = "PROV_";
    esp_wifi_get_mac(WIFI_IF_STA, eth_mac);
    snprintf(service_name, max, "%s%02X%02X%02X",
             ssid_prefix, eth_mac[3], eth_mac[4], eth_mac[5]);
}

static void wifi_event_handler(void *args, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_PROV_EVENT)
    {
        switch (event_id)
        {
        case WIFI_PROV_START:
            ESP_LOGI(TAG, "Provisioning started");
            break;
        case WIFI_PROV_CRED_RECV:
            wifi_sta_config_t *wifi_sta_cfg = (wifi_sta_config_t *)event_data;
            ESP_LOGI(TAG, "Received Wi-Fi credentials"
                          "\n\tSSID     : %s\n\tPassword : %s",
                     (const char *)wifi_sta_cfg->ssid,
                     (const char *)wifi_sta_cfg->password);
            break;
        case WIFI_PROV_CRED_FAIL:
            wifi_prov_sta_fail_reason_t *reason = (wifi_prov_sta_fail_reason_t *)event_data;
            ESP_LOGE(TAG, "Provisioning failed!\n\tReason : %s"
                          "\n\tPlease reset to factory and retry provisioning",
                     (*reason == WIFI_PROV_STA_AUTH_ERROR) ? "Wi-Fi station authentication failed" : "Wi-Fi access-point not found");
            break;
        case WIFI_PROV_CRED_SUCCESS:
            ESP_LOGI(TAG, "Provisioning successful");
            break;
        case WIFI_PROV_END:
            ESP_LOGI(TAG, "Provisioning ended");
            wifi_prov_mgr_deinit(); // Deinitialize provisioning manager
            break;
        default:
            ESP_LOGW(TAG, "Unknown provisioning event: %d", event_id);
            break;
        }
    }
    else if (event_base == WIFI_EVENT)
    {
        switch (event_id)
        {
        case WIFI_EVENT_STA_START:
            ESP_LOGI(TAG, "Wi-Fi STA started");
            esp_wifi_connect(); // Attempt to connect to the configured Wi-Fi network
            break;
        case WIFI_EVENT_STA_DISCONNECTED:
            ESP_LOGI(TAG, "Wi-Fi STA disconnected, attempting to reconnect...");
            esp_wifi_connect(); // Attempt to reconnect
            break;
        default:
            ESP_LOGW(TAG, "Unknown Wi-Fi event: %d", event_id);
            break;
        }
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP address: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_EVENT);
        start_mdns();
    }
}