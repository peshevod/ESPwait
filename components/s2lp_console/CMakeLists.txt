idf_component_register(SRCS "s2lp_console.c"
                    INCLUDE_DIRS "." cmd_nvs
                    REQUIRES console fatfs nvs_flash)