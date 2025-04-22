idf_component_register(SRCS "main.cpp"
                    PRIV_REQUIRES spi_flash
                    esp_driver_gpio
                    driver
                    esp_wifi
                    nvs_flash
                    esp_http_server
                    mpu6050
                    esp_netif
                    esp_event
                    arduino
                    ESPAsyncWebServer
                    INCLUDE_DIRS "")