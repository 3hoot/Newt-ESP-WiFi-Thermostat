idf_component_register(
    SRCS "server.c"
    INCLUDE_DIRS "."
    REQUIRES esp_http_server json
)