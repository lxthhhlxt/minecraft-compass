idf_component_register(SRC_DIRS "src"
                       INCLUDE_DIRS "include"
                       REQUIRES MahonyAHRS driver nvs_flash esp_timer status_manager data_manager)