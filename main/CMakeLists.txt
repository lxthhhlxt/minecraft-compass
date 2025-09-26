idf_component_register(SRCS "main.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES MahonyAHRS imu_sensor esp_timer led_strip compressed_led nvs_flash gps_sensor)