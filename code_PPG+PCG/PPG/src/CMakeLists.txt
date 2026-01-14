idf_component_register(SRCS "max30102_test.c"
                       INCLUDE_DIRS "."
                       REQUIRES MAX30102 i2cdev esp_idf_lib_helpers)