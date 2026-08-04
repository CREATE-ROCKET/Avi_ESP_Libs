set(AVI_ESP_LIBS_SRCS
    "src/bus/CANCREATE.cpp"
    "src/bus/SPICREATE.cpp"
    "src/compatibility/time.cpp"
    "src/compatibility/version.cpp"
    "src/radio/NEC920.cpp"
    "src/sensor/H3LIS331.cpp"
    "src/sensor/ICM20602.cpp"
    "src/sensor/ICM20948.cpp"
    "src/sensor/ICM42688.cpp"
    "src/sensor/LPS25HB.cpp"
    "src/storage/S25FL127S.cpp"
    "src/storage/S25FL512S.cpp")
set(AVI_ESP_LIBS_REQUIRES driver esp_timer)
if(IDF_VERSION_MAJOR GREATER_EQUAL 6)
    list(APPEND AVI_ESP_LIBS_REQUIRES esp_driver_gpio esp_driver_spi esp_driver_twai esp_driver_uart)
endif()
idf_component_register(SRCS ${AVI_ESP_LIBS_SRCS} INCLUDE_DIRS "include" PRIV_INCLUDE_DIRS "src/sensor" REQUIRES ${AVI_ESP_LIBS_REQUIRES})
target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_17)
