#ifndef SERVER_H
#define SERVER_H

#include "esp_http_server.h"

typedef void (*setting_update)(double *setpoint, double *deadband);

// Structure to hold pointers to the regulator's parameters
typedef struct server_regulator {
    setting_update get;
    setting_update set;
    double *current_temp;
    int *output_hot;
    int *output_cold;
} server_regulator_t;

// Function to start the web server
httpd_handle_t start_webserver(server_regulator_t *shared_regulator);

#endif // SERVER_H