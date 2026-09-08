idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES network esp_driver_gpio regulator server
)
