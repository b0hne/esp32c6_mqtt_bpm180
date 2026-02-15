idf_component_register(
    SRCS "app_main.c" "bmp180.c"
    PRIV_REQUIRES esp-tls esp_wifi esp_event nvs_flash driver
    INCLUDE_DIRS "."
    EMBED_TXTFILES "mosquitto.org.crt"
    REQUIRES esp_driver_i2c esp_driver_gpio
)
