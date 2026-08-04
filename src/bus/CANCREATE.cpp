#include "CANCREATE.h"
#include <cstring>
#include "freertos/FreeRTOS.h"
CANCREATE::~CANCREATE(){if(initialized_)(void)end();}
esp_err_t CANCREATE::begin(gpio_num_t tx,gpio_num_t rx,uint32_t bitrate){
    if(initialized_)return ESP_ERR_INVALID_STATE;if(!GPIO_IS_VALID_OUTPUT_GPIO(tx)||!GPIO_IS_VALID_GPIO(rx))return ESP_ERR_INVALID_ARG;
    twai_general_config_t general=TWAI_GENERAL_CONFIG_DEFAULT(tx,rx,TWAI_MODE_NORMAL);twai_timing_config_t timing{};
    switch(bitrate){case 1000000: timing=TWAI_TIMING_CONFIG_1MBITS();break;case 500000: timing=TWAI_TIMING_CONFIG_500KBITS();break;case 250000: timing=TWAI_TIMING_CONFIG_250KBITS();break;case 125000: timing=TWAI_TIMING_CONFIG_125KBITS();break;default:return ESP_ERR_INVALID_ARG;}
    const twai_filter_config_t filter=TWAI_FILTER_CONFIG_ACCEPT_ALL();esp_err_t r=twai_driver_install(&general,&timing,&filter);if(r!=ESP_OK)return r;r=twai_start();if(r!=ESP_OK){(void)twai_driver_uninstall();return r;}initialized_=true;return ESP_OK;
}
esp_err_t CANCREATE::end(){if(!initialized_)return ESP_ERR_INVALID_STATE;esp_err_t r=twai_stop();if(r!=ESP_OK)return r;r=twai_driver_uninstall();if(r==ESP_OK)initialized_=false;return r;}
esp_err_t CANCREATE::write(const CANCREATEFrame& f,uint32_t timeout){if(!initialized_)return ESP_ERR_INVALID_STATE;if(f.data_length>8)return ESP_ERR_INVALID_ARG;twai_message_t m{};m.identifier=f.identifier;m.data_length_code=f.data_length;m.extd=f.extended;m.rtr=f.remote;std::memcpy(m.data,f.data,f.data_length);return twai_transmit(&m,pdMS_TO_TICKS(timeout));}
esp_err_t CANCREATE::read(CANCREATEFrame& f,uint32_t timeout){if(!initialized_)return ESP_ERR_INVALID_STATE;twai_message_t m{};auto r=twai_receive(&m,pdMS_TO_TICKS(timeout));if(r!=ESP_OK)return r;f.identifier=m.identifier;f.data_length=m.data_length_code;f.extended=m.extd;f.remote=m.rtr;std::memcpy(f.data,m.data,m.data_length_code);return ESP_OK;}
