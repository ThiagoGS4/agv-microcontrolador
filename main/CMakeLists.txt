idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_gpio
    REQUIRES esp_wifi
    REQUIRES nvs_flash
    REQUIRES mqtt
    REQUIRES esp_driver_i2c
    PRIV_REQUIRES esp_driver_ledc
    REQUIRES esp_adc
)
