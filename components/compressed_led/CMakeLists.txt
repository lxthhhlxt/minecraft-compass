idf_component_register(SRCS "src/compressed_led.cpp"
                       INCLUDE_DIRS "include"
                       REQUIRES led_strip status_manager data_manager esp_timer)