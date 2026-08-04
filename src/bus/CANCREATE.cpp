#include "CANCREATE.h"
#include <cstring>
#include <new>
#include "esp_idf_version.h"
#include "freertos/FreeRTOS.h"
#if ESP_IDF_VERSION_MAJOR >= 6
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "freertos/queue.h"
namespace {
struct RawFrame { twai_frame_header_t header; uint8_t data[8]; };
struct Backend { twai_node_handle_t node{}; QueueHandle_t queue{}; };
bool receiveFrame(twai_node_handle_t node, const twai_rx_done_event_data_t*, void* context) {
    // contextはbegin()で生成しend()まで保持するQueueHandle_tで、ISRからのみ参照される。
    RawFrame raw{}; twai_frame_t frame{raw.header, raw.data, sizeof(raw.data)};
    if (twai_node_receive_from_isr(node, &frame) != ESP_OK) return false;
    raw.header = frame.header; BaseType_t awake = pdFALSE;
    xQueueSendFromISR(static_cast<QueueHandle_t>(context), &raw, &awake);
    return awake == pdTRUE;
}
}
#else
#include "driver/twai.h"
#endif
CANCREATE::~CANCREATE(){if(initialized_)(void)end();}
esp_err_t CANCREATE::begin(gpio_num_t tx,gpio_num_t rx,uint32_t bitrate){
    if(initialized_)return ESP_ERR_INVALID_STATE;
    if(!GPIO_IS_VALID_OUTPUT_GPIO(tx)||!GPIO_IS_VALID_GPIO(rx))return ESP_ERR_INVALID_ARG;
#if ESP_IDF_VERSION_MAJOR >= 6
    auto* b=new(std::nothrow) Backend{}; if(!b)return ESP_ERR_NO_MEM;
    b->queue=xQueueCreate(8,sizeof(RawFrame)); if(!b->queue){delete b;return ESP_ERR_NO_MEM;}
    twai_onchip_node_config_t c{}; c.io_cfg.tx=tx;c.io_cfg.rx=rx;c.io_cfg.quanta_clk_out=GPIO_NUM_NC;c.io_cfg.bus_off_indicator=GPIO_NUM_NC;c.bit_timing.bitrate=bitrate;c.tx_queue_depth=8;c.fail_retry_cnt=-1;
    esp_err_t r=twai_new_node_onchip(&c,&b->node);
    if(r==ESP_OK){twai_event_callbacks_t callbacks{};callbacks.on_rx_done=receiveFrame;r=twai_node_register_event_callbacks(b->node,&callbacks,b->queue);}
    if(r==ESP_OK)r=twai_node_enable(b->node);
    if(r!=ESP_OK){if(b->node)(void)twai_node_delete(b->node);vQueueDelete(b->queue);delete b;return r;} backend_=b;
#else
    twai_general_config_t general=TWAI_GENERAL_CONFIG_DEFAULT(tx,rx,TWAI_MODE_NORMAL);twai_timing_config_t timing{};
    switch(bitrate){case 1000000:timing=TWAI_TIMING_CONFIG_1MBITS();break;case 500000:timing=TWAI_TIMING_CONFIG_500KBITS();break;case 250000:timing=TWAI_TIMING_CONFIG_250KBITS();break;case 125000:timing=TWAI_TIMING_CONFIG_125KBITS();break;default:return ESP_ERR_INVALID_ARG;}
    const twai_filter_config_t filter=TWAI_FILTER_CONFIG_ACCEPT_ALL();esp_err_t r=twai_driver_install(&general,&timing,&filter);if(r!=ESP_OK)return r;r=twai_start();if(r!=ESP_OK){(void)twai_driver_uninstall();return r;}
#endif
    initialized_=true;return ESP_OK;
}
esp_err_t CANCREATE::end(){if(!initialized_)return ESP_ERR_INVALID_STATE;
#if ESP_IDF_VERSION_MAJOR >= 6
    // backend_はbegin()で確保したBackendを所有し、node停止後にだけ解放する。
    auto* b=static_cast<Backend*>(backend_);esp_err_t r=twai_node_disable(b->node);if(r==ESP_OK)r=twai_node_delete(b->node);if(r==ESP_OK){vQueueDelete(b->queue);delete b;backend_=nullptr;initialized_=false;}return r;
#else
    esp_err_t r=twai_stop();if(r==ESP_OK)r=twai_driver_uninstall();if(r==ESP_OK)initialized_=false;return r;
#endif
}
esp_err_t CANCREATE::write(const CANCREATEFrame& f,uint32_t timeout){if(!initialized_)return ESP_ERR_INVALID_STATE;if(f.data_length>8)return ESP_ERR_INVALID_ARG;
#if ESP_IDF_VERSION_MAJOR >= 6
    twai_frame_t frame{};frame.header.id=f.identifier;frame.header.dlc=f.data_length;frame.header.ide=f.extended;frame.header.rtr=f.remote;frame.buffer=const_cast<uint8_t*>(f.data);frame.buffer_len=f.data_length;return twai_node_transmit(static_cast<Backend*>(backend_)->node,&frame,timeout);
#else
    twai_message_t m{};m.identifier=f.identifier;m.data_length_code=f.data_length;m.extd=f.extended;m.rtr=f.remote;std::memcpy(m.data,f.data,f.data_length);return twai_transmit(&m,pdMS_TO_TICKS(timeout));
#endif
}
esp_err_t CANCREATE::read(CANCREATEFrame& f,uint32_t timeout){if(!initialized_)return ESP_ERR_INVALID_STATE;
#if ESP_IDF_VERSION_MAJOR >= 6
    RawFrame raw{};if(xQueueReceive(static_cast<Backend*>(backend_)->queue,&raw,pdMS_TO_TICKS(timeout))!=pdTRUE)return ESP_ERR_TIMEOUT;f.identifier=raw.header.id;f.data_length=raw.header.dlc;f.extended=raw.header.ide;f.remote=raw.header.rtr;std::memcpy(f.data,raw.data,f.data_length);return ESP_OK;
#else
    twai_message_t m{};auto r=twai_receive(&m,pdMS_TO_TICKS(timeout));if(r!=ESP_OK)return r;f.identifier=m.identifier;f.data_length=m.data_length_code;f.extended=m.extd;f.remote=m.rtr;std::memcpy(f.data,m.data,m.data_length_code);return ESP_OK;
#endif
}
