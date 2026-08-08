set(AVI_ESP_LIBS_SRCS
    "src/bus/CANCREATE.cpp"
    "src/bus/I2CCREATE.cpp"
    "src/bus/SPICREATE.cpp"
    "src/bus/STSCREATE.cpp"
    "src/compatibility/time.cpp"
    "src/compatibility/version.cpp"
    "src/radio/NEC920.cpp"
    "src/sensor/AS5047D.cpp"
    "src/sensor/H3LIS331.cpp"
    "src/sensor/ICM20602.cpp"
    "src/sensor/ICM20948.cpp"
    "src/sensor/ICM42688.cpp"
    "src/sensor/LPS25HB.cpp"
    "src/sensor/SSCDRRN005PD2A5.cpp"
    "src/storage/S25FL127S.cpp"
    "src/storage/S25FL512S.cpp")
set(AVI_ESP_LIBS_REQUIRES driver esp_timer freertos heap)
if(IDF_VERSION_MAJOR GREATER_EQUAL 6)
    list(APPEND AVI_ESP_LIBS_REQUIRES esp_driver_gpio esp_driver_i2c esp_driver_spi esp_driver_twai esp_driver_uart)
endif()
idf_component_register(SRCS ${AVI_ESP_LIBS_SRCS} INCLUDE_DIRS "include" REQUIRES ${AVI_ESP_LIBS_REQUIRES})
target_compile_options(${COMPONENT_LIB} PRIVATE "$<$<COMPILE_LANGUAGE:CXX>:-std=gnu++17>")
