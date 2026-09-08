#include <stdio.h>
#include <string.h>

// ESP-IDF headers
#include "esp_log.h"
#include "esp_http_server.h"
#include "cJSON.h"

// User defined headers
#include "server.h"

static const char *TAG = "server";

static const char *html_page = 
"<!DOCTYPE html><html><head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
"<style>"
"body{font-family:sans-serif;margin:20px;max-width:360px;}"
".card{padding:15px;border:1px solid #ddd;border-radius:8px;margin-bottom:15px;}"
"label{display:block;margin-top:10px;font-weight:bold;}"
"input{width:100%;padding:8px;margin-top:4px;box-sizing:border-box;}"
"button{margin-top:15px;padding:10px;width:100%;background:#007bff;color:#fff;border:none;border-radius:4px;cursor:pointer;}"
"</style></head><body>"
"<h2>NEWT Thermostat</h2>"
"<div class=\"card\">"
"  <h3>Current Temp: <span id=\"temp\">--</span> &deg;C</h3>"
"</div>"
"<div class=\"card\">"
"  <label>Setpoint (&deg;C)</label>"
"  <input type=\"number\" id=\"setpoint\" step=\"0.1\">"
"  <label>Deadband (&deg;C)</label>"
"  <input type=\"number\" id=\"deadband\" step=\"0.1\">"
"  <button onclick=\"saveSettings()\">Update Settings</button>"
"</div>"
"<script>"
"async function fetchStatus(){"
"  try{"
"    const res = await fetch('/api/status');"
"    const data = await res.json();"
"    document.getElementById('temp').innerText = data.temp.toFixed(1);"
"    if(document.activeElement.id !== 'setpoint') document.getElementById('setpoint').value = data.setpoint;"
"    if(document.activeElement.id !== 'deadband') document.getElementById('deadband').value = data.deadband;"
"  }catch(e){console.error(e);}"
"}"
"async function saveSettings(){"
"  const sp = parseFloat(document.getElementById('setpoint').value);"
"  const db = parseFloat(document.getElementById('deadband').value);"
"  await fetch('/api/settings',{"
"    method:'POST',"
"    headers:{'Content-Type':'application/json'},"
"    body:JSON.stringify({setpoint:sp, deadband:db})"
"  });"
"  fetchStatus();"
"}"
"fetchStatus();"
"setInterval(fetchStatus, 2000);" // Poll every 2 seconds
"</script></body></html>";

static server_regulator_t *shared_regulator = NULL;

// GET / -> Serve Dashboard HTML
static esp_err_t root_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_sendstr(req, html_page);
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
        httpd_uri_t status_uri = {.uri = "/api/status", .method = HTTP_GET, .handler = status_get_handler};
        httpd_uri_t settings_uri = {.uri = "/api/settings", .method = HTTP_POST, .handler = settings_post_handler};

        httpd_register_uri_handler(server, &root_uri);
        httpd_register_uri_handler(server, &status_uri);
        httpd_register_uri_handler(server, &settings_uri);
    }
    return server;
}