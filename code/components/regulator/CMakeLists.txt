idf_component_register(
    SRCS "regulator.c"
    INCLUDE_DIRS "."
    REQUIRES nvs_flash esp_adc esp_driver_ledc
)