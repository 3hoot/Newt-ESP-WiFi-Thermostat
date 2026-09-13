#include <stdio.h>
#include <string.h>

// ESP-IDF headers
#include "esp_log.h"
#include "esp_http_server.h"
#include "cJSON.h"

// User defined headers
#include "server.h"

static const char *TAG = "server";

extern const unsigned char web_html_start[] asm("_binary_web_html_start");
extern const unsigned char web_html_end[]   asm("_binary_web_html_end");

extern const unsigned char style_css_start[] asm("_binary_style_css_start");
extern const unsigned char style_css_end[]   asm("_binary_style_css_end");

static server_regulator_t *shared_regulator = NULL;

// GET / -> Serve Dashboard HTML
static esp_err_t root_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");

    httpd_resp_send(
        req, 
        (const char *)web_html_start,
        web_html_end - web_html_start
    );
    return ESP_OK;
}

static esp_err_t style_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/css");

    httpd_resp_send(
        req,
        (const char *)style_css_start,
        style_css_end - style_css_start
    );

    return ESP_OK;
}

// GET /api/status -> Return JSON state
static esp_err_t status_get_handler(httpd_req_t *req)
{
    double setpoint, deadband;
    shared_regulator->get(&setpoint, &deadband);

    char response[128];
    snprintf(response, sizeof(response),
             "{\"temp\":%.1f,\"setpoint\":%.1f,\"deadband\":%.1f}",
             *(shared_regulator->current_temp), setpoint, deadband);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, response);
    return ESP_OK;
}

// POST /api/settings -> Update setpoint and deadband
static esp_err_t settings_post_handler(httpd_req_t *req)
{
    char buf[128];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    double setpoint, deadband;
    if (ret <= 0)
        return ESP_FAIL;
    buf[ret] = '\0'; // Null terminate string

    cJSON *json = cJSON_Parse(buf);
    if (json)
    {
        cJSON *sp = cJSON_GetObjectItem(json, "setpoint");
        cJSON *db = cJSON_GetObjectItem(json, "deadband");

        if (cJSON_IsNumber(sp))
            setpoint = (float)sp->valuedouble;
        if (cJSON_IsNumber(db))
            deadband = (float)db->valuedouble;

        shared_regulator->set(&setpoint, &deadband);

        cJSON_Delete(json);
        ESP_LOGI(TAG, "Updated -> Setpoint: %.1f | Deadband: %.1f", setpoint, deadband);

        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"status\":\"ok\"}");
        return ESP_OK;
    }

    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
    return ESP_FAIL;
}

// Server Initialization
httpd_handle_t start_webserver(server_regulator_t *regulator)
{
    shared_regulator = regulator;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK)
    {
        httpd_uri_t root_uri = {.uri = "/", .method = HTTP_GET, .handler = root_get_handler};
        httpd_uri_t style_uri = {.uri = "/style.css", .method = HTTP_GET, .handler = style_get_handler};
        httpd_uri_t status_uri = {.uri = "/api/status", .method = HTTP_GET, .handler = status_get_handler};
        httpd_uri_t settings_uri = {.uri = "/api/settings", .method = HTTP_POST, .handler = settings_post_handler};

        httpd_register_uri_handler(server, &root_uri);
        httpd_register_uri_handler(server, &style_uri);
        httpd_register_uri_handler(server, &status_uri);
        httpd_register_uri_handler(server, &settings_uri);
    }
    return server;
}