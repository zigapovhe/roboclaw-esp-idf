idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES roboclaw
    PRIV_REQUIRES esp_driver_gpio
)