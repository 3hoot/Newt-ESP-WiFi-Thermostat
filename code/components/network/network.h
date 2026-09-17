#ifndef NETWORK_H
#define NETWORK_H

#include <stdbool.h>

// ESP-IDF headers
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

// Self explanatory
extern EventGroupHandle_t wifi_event_group;
extern const int WIFI_CONNECTED_EVENT;

#define WIFI_PROV_MAX_RETRY (5)

/**
 * @brief Initializes the Wi-Fi stack and sets up event handlers for Wi-Fi and IP events.
 *
 * This function initializes the TCP/IP stack, creates an event loop, and registers event handlers
 * for Wi-Fi and IP events. It also sets up the default Wi-Fi station and softAP interfaces.
 * If the device is not provisioned, it starts the provisioning service; otherwise, it connects to Wi-Fi.
 */
void wifi_init();

/**
 * @brief Checks if the device is connected to Wi-Fi within a specified timeout.
 *
 * This function waits for the Wi-Fi connection event for a specified timeout duration.
 * If the device connects to Wi-Fi within the timeout, it returns true; otherwise, it returns false.
 *
 * @param timeout The maximum time to wait for the Wi-Fi connection event (in ticks).
 * @return true if connected to Wi-Fi within the timeout, false otherwise.
 */
bool wifi_connected(TickType_t timeout);

#endif // NETWORK_H