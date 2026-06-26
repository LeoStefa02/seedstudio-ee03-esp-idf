idf_component_register(SRCS 
                        "main.c"
                        "sht40.c"
                        "it8951.c"
                        "wifi.c"
                        "tcp_client.c"
                        "button.c"
                        "mdns_discover.c"
                    INCLUDE_DIRS "."
                    REQUIRES mdns esp_driver_i2c esp_driver_spi esp_driver_gpio esp_wifi nvs_flash
                    )
