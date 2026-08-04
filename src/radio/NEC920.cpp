#include "NEC920.h"
#include "avi_esp_libs/compatibility.h"
NEC920::~NEC920(){if(initialized_)(void)end();}
esp_err_t NEC920::begin(uart_port_t port,int baud,gpio_num_t rx,gpio_num_t tx,gpio_num_t reset,gpio_num_t wakeup,gpio_num_t mode){
    if(initialized_)return ESP_ERR_INVALID_STATE;if(port<0||port>=UART_NUM_MAX||baud<=0||!GPIO_IS_VALID_GPIO(rx)||!GPIO_IS_VALID_OUTPUT_GPIO(tx))return ESP_ERR_INVALID_ARG;
    uart_config_t config{};config.baud_rate=baud;config.data_bits=UART_DATA_8_BITS;config.parity=UART_PARITY_DISABLE;config.stop_bits=UART_STOP_BITS_1;config.flow_ctrl=UART_HW_FLOWCTRL_DISABLE;config.source_clk=UART_SCLK_DEFAULT;
    esp_err_t r=uart_param_config(port,&config);if(r!=ESP_OK)return r;r=uart_set_pin(port,tx,rx,UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE);if(r!=ESP_OK)return r;r=uart_driver_install(port,512,0,0,nullptr,0);if(r!=ESP_OK)return r;
    if(reset!=GPIO_NUM_NC){if(!GPIO_IS_VALID_OUTPUT_GPIO(reset)){(void)uart_driver_delete(port);return ESP_ERR_INVALID_ARG;}gpio_set_direction(reset,GPIO_MODE_OUTPUT);gpio_set_level(reset,1);}if(wakeup!=GPIO_NUM_NC){if(!GPIO_IS_VALID_OUTPUT_GPIO(wakeup)){(void)uart_driver_delete(port);return ESP_ERR_INVALID_ARG;}gpio_set_direction(wakeup,GPIO_MODE_OUTPUT);gpio_set_level(wakeup,1);}if(mode!=GPIO_NUM_NC){if(!GPIO_IS_VALID_GPIO(mode)){(void)uart_driver_delete(port);return ESP_ERR_INVALID_ARG;}gpio_set_direction(mode,GPIO_MODE_INPUT);}port_=port;reset_=reset;wakeup_=wakeup;initialized_=true;return ESP_OK;
}
esp_err_t NEC920::end(){if(!initialized_)return ESP_ERR_INVALID_STATE;auto r=uart_driver_delete(port_);if(r==ESP_OK){initialized_=false;port_=UART_NUM_MAX;}return r;}
esp_err_t NEC920::read(uint8_t* data,size_t capacity,size_t& received,uint32_t timeout){received=0;if(!initialized_)return ESP_ERR_INVALID_STATE;if(!data||capacity==0)return ESP_ERR_INVALID_ARG;int n=uart_read_bytes(port_,data,capacity,pdMS_TO_TICKS(timeout));if(n<0)return ESP_FAIL;received=n;return n==0?ESP_ERR_TIMEOUT:ESP_OK;}
esp_err_t NEC920::write(const uint8_t* data,size_t length,uint32_t timeout){if(!initialized_)return ESP_ERR_INVALID_STATE;if(!data||length==0)return ESP_ERR_INVALID_ARG;int n=uart_write_bytes(port_,data,length);if(n<0||size_t(n)!=length)return ESP_FAIL;return uart_wait_tx_done(port_,pdMS_TO_TICKS(timeout));}
esp_err_t NEC920::sleep(){if(!initialized_||wakeup_==GPIO_NUM_NC)return ESP_ERR_INVALID_STATE;return gpio_set_level(wakeup_,0);}
esp_err_t NEC920::wake(){if(!initialized_||wakeup_==GPIO_NUM_NC)return ESP_ERR_INVALID_STATE;return gpio_set_level(wakeup_,1);}
esp_err_t NEC920::reboot(uint32_t pulse){if(!initialized_||reset_==GPIO_NUM_NC||pulse==0)return ESP_ERR_INVALID_ARG;auto r=gpio_set_level(reset_,0);if(r!=ESP_OK)return r;avi_delay_ms(pulse);return gpio_set_level(reset_,1);}
