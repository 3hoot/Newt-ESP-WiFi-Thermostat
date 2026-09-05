#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

// ESP-IDF headers
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_system.h" // System initialization and management
#include "nvs_flash.h"  // NVS initialization

// User defined headers
#include "network.h"
#include "regulator.h"

static const char *TAG = "main";

static regulator_t regulator;
regulator_args_t regulator_args;

TaskHandle_t regulator_task_handle = NULL;
TaskHandle_t ntc_readout_task_handle = NULL;
TaskHandle_t regulator_pid_tune_task_handle = NULL;

static void nvs_init()
{
    ESP_LOGI(TAG, "Initializing NVS flash");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
}

void app_main(void)
{
    nvs_init(); // before initializing Wi-Fi, initialize NVS (Non-Volatile Storage) to store Wi-Fi credentials
    wifi_init();

    // Wait for connection
    if (!wifi_connected(portMAX_DELAY))
        esp_restart(); // Restart the device if not connected within the timeout

    regulator_args.regulator = &regulator;
    regulator_init(&regulator_args);

    xTaskCreate(ntc_readout_task, "ntc_readout_task", 4096, &regulator_args, 5, &ntc_readout_task_handle);
    xTaskCreate(regulator_pid_tune_task, "regulator_pid_tune_task", 4096, &regulator_args, 5, &regulator_pid_tune_task_handle);
    // xTaskCreate(regulator_task, "regulator_task", 4096, &regulator_args, 5, &regulator_task_handle);

    while (true)
    {
        ESP_LOGI(TAG, "Hello, world! The device is connected to Wi-Fi.");
        vTaskDelay(pdMS_TO_TICKS(5000)); // Delay for 5 seconds
    }
}