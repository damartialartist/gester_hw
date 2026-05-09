idf_component_register(SRCS "bt_sender.c"
                    INCLUDE_DIRS "include"
                    REQUIRES driver bt sensorMsg nvs_flash)
