idf_component_register(SRCS 
    "roboclaw.c"
    "roboclaw_uart.c"
    "roboclaw_crc.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_uart esp_driver_gpio
    PRIV_REQUIRES freertos esp_common)
