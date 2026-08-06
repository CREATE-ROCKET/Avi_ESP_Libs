#include "CANCREATE.h"

#include <climits>
#include <cstring>
#include <new>

#include "avi_esp_libs/compatibility.h"
#include "esp_idf_version.h"
#include "freertos/FreeRTOS.h"

namespace {

constexpr uint32_t kStandardIdMask = 0x7FF;
constexpr uint32_t kExtendedIdMask = 0x1FFFFFFF;

bool validBitrate(uint32_t bitrate) {
  switch (bitrate) {
  case 25000:
  case 50000:
  case 100000:
  case 125000:
  case 250000:
  case 500000:
  case 800000:
  case 1000000:
    return true;
  default:
    return false;
  }
}

bool validIdentifier(uint32_t identifier, bool extended) {
  return identifier <= (extended ? kExtendedIdMask : kStandardIdMask);
}

bool timeoutToTicks(uint32_t timeout_ms, TickType_t &ticks) {
  if (timeout_ms > INT_MAX)
    return false;
  uint64_t value =
      (uint64_t{timeout_ms} * configTICK_RATE_HZ + 999U) / 1000U;
  if (timeout_ms != 0 && value == 0)
    value = 1;
  if (value >= portMAX_DELAY)
    return false;
  ticks = static_cast<TickType_t>(value);
  return true;
}

bool validConfig(const CANCREATE::Config &config) {
  if (!GPIO_IS_VALID_OUTPUT_GPIO(config.tx) ||
      !GPIO_IS_VALID_GPIO(config.rx) || !validBitrate(config.bitrate) ||
      config.rx_queue_depth == 0)
    return false;
  if (config.mode != CANCREATE::Mode::normal &&
      config.mode != CANCREATE::Mode::no_ack &&
      config.mode != CANCREATE::Mode::listen_only)
    return false;
  return !config.filter.enabled ||
         (validIdentifier(config.filter.identifier, config.filter.extended) &&
          validIdentifier(config.filter.mask, config.filter.extended));
}

} // namespace

#if ESP_IDF_VERSION_MAJOR >= 6

#include "esp_attr.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

namespace {

struct RawFrame {
  twai_frame_header_t header{};
  uint8_t data[8]{};
};

struct Backend {
  twai_node_handle_t node{};
  QueueHandle_t rx_queue{};
  SemaphoreHandle_t tx_available{};
  twai_frame_t tx_frame{};
  uint8_t tx_data[8]{};
  uint32_t dropped_rx{};
};

void destroyBackend(Backend *backend) {
  if (backend->tx_available != nullptr)
    vSemaphoreDelete(backend->tx_available);
  if (backend->rx_queue != nullptr)
    vQueueDelete(backend->rx_queue);
  delete backend;
}

esp_err_t releaseNode(Backend *backend) {
  if (backend->node == nullptr) {
    destroyBackend(backend);
    return ESP_OK;
  }
  const esp_err_t disable_result = twai_node_disable(backend->node);
  const esp_err_t delete_result = twai_node_delete(backend->node);
  if (delete_result != ESP_OK)
    return delete_result;
  backend->node = nullptr;
  destroyBackend(backend);
  (void)disable_result;
  return ESP_OK;
}

bool IRAM_ATTR receiveFrame(twai_node_handle_t node,
                            const twai_rx_done_event_data_t *, void *context) {
  // SAFETY: contextはcallback解除を含むnode削除完了まで生存するBackendであり、
  // ISR内では固定長データのqueue送信とdrop数更新だけを行う。
  auto *backend = static_cast<Backend *>(context);
  RawFrame raw{};
  twai_frame_t frame{};
  frame.buffer = raw.data;
  frame.buffer_len = sizeof(raw.data);
  if (twai_node_receive_from_isr(node, &frame) != ESP_OK ||
      frame.header.dlc > sizeof(raw.data)) {
    __atomic_fetch_add(&backend->dropped_rx, 1U, __ATOMIC_RELAXED);
    return false;
  }
  raw.header = frame.header;
  BaseType_t task_awoken = pdFALSE;
  if (xQueueSendFromISR(backend->rx_queue, &raw, &task_awoken) != pdTRUE)
    __atomic_fetch_add(&backend->dropped_rx, 1U, __ATOMIC_RELAXED);
  return task_awoken == pdTRUE;
}

bool IRAM_ATTR transmitDone(twai_node_handle_t,
                            const twai_tx_done_event_data_t *, void *context) {
  // SAFETY: contextと送信slotはnode削除完了まで生存し、ISRではsemaphoreを
  // 返すだけでSPI通信、ログ、動的確保、blocking処理を行わない。
  auto *backend = static_cast<Backend *>(context);
  BaseType_t task_awoken = pdFALSE;
  (void)xSemaphoreGiveFromISR(backend->tx_available, &task_awoken);
  return task_awoken == pdTRUE;
}

CANCREATE::State stateFrom(twai_error_state_t state) {
  return state == TWAI_ERROR_BUS_OFF ? CANCREATE::State::bus_off
                                    : CANCREATE::State::running;
}

} // namespace

#else

#include "driver/twai.h"

namespace {

struct Backend {
  bool running{false};
};

twai_mode_t modeFrom(CANCREATE::Mode mode) {
  switch (mode) {
  case CANCREATE::Mode::no_ack:
    return TWAI_MODE_NO_ACK;
  case CANCREATE::Mode::listen_only:
    return TWAI_MODE_LISTEN_ONLY;
  case CANCREATE::Mode::normal:
  default:
    return TWAI_MODE_NORMAL;
  }
}

bool timingFrom(uint32_t bitrate, twai_timing_config_t &timing) {
  switch (bitrate) {
  case 25000:
    timing = TWAI_TIMING_CONFIG_25KBITS();
    return true;
  case 50000:
    timing = TWAI_TIMING_CONFIG_50KBITS();
    return true;
  case 100000:
    timing = TWAI_TIMING_CONFIG_100KBITS();
    return true;
  case 125000:
    timing = TWAI_TIMING_CONFIG_125KBITS();
    return true;
  case 250000:
    timing = TWAI_TIMING_CONFIG_250KBITS();
    return true;
  case 500000:
    timing = TWAI_TIMING_CONFIG_500KBITS();
    return true;
  case 800000:
    timing = TWAI_TIMING_CONFIG_800KBITS();
    return true;
  case 1000000:
    timing = TWAI_TIMING_CONFIG_1MBITS();
    return true;
  default:
    return false;
  }
}

twai_filter_config_t filterFrom(const CANCREATE::Filter &filter) {
  if (!filter.enabled)
    return TWAI_FILTER_CONFIG_ACCEPT_ALL();
  const uint8_t shift = filter.extended ? 3 : 21;
  const uint32_t format_bit = filter.extended ? (1U << 1) : (1U << 19);
  twai_filter_config_t result{};
  result.acceptance_code =
      (filter.identifier << shift) | (filter.extended ? format_bit : 0U);
  result.acceptance_mask = ~((filter.mask << shift) | format_bit);
  result.single_filter = true;
  return result;
}

CANCREATE::State stateFrom(twai_state_t state) {
  switch (state) {
  case TWAI_STATE_RUNNING:
    return CANCREATE::State::running;
  case TWAI_STATE_BUS_OFF:
    return CANCREATE::State::bus_off;
  case TWAI_STATE_RECOVERING:
    return CANCREATE::State::recovering;
  case TWAI_STATE_STOPPED:
  default:
    return CANCREATE::State::stopped;
  }
}

} // namespace

#endif

CANCREATE::~CANCREATE() {
  if (backend_ != nullptr)
    (void)end();
}

esp_err_t CANCREATE::begin(gpio_num_t tx, gpio_num_t rx, uint32_t bitrate) {
  Config config{};
  config.tx = tx;
  config.rx = rx;
  config.bitrate = bitrate;
  return begin(config);
}

esp_err_t CANCREATE::begin(const Config &config) {
  if (backend_ != nullptr)
    return ESP_ERR_INVALID_STATE;
  if (!validConfig(config))
    return ESP_ERR_INVALID_ARG;

#if ESP_IDF_VERSION_MAJOR >= 6
  auto *backend = new (std::nothrow) Backend{};
  if (backend == nullptr)
    return ESP_ERR_NO_MEM;
  backend->rx_queue = xQueueCreate(config.rx_queue_depth, sizeof(RawFrame));
  backend->tx_available = xSemaphoreCreateBinary();
  if (backend->rx_queue == nullptr || backend->tx_available == nullptr) {
    destroyBackend(backend);
    return ESP_ERR_NO_MEM;
  }
  (void)xSemaphoreGive(backend->tx_available);

  twai_onchip_node_config_t node_config{};
  node_config.io_cfg.tx = config.tx;
  node_config.io_cfg.rx = config.rx;
  node_config.io_cfg.quanta_clk_out = GPIO_NUM_NC;
  node_config.io_cfg.bus_off_indicator = GPIO_NUM_NC;
  node_config.bit_timing.bitrate = config.bitrate;
  node_config.tx_queue_depth = 1;
  node_config.fail_retry_cnt = -1;
  node_config.flags.enable_self_test = config.mode == Mode::no_ack;
  node_config.flags.enable_listen_only = config.mode == Mode::listen_only;

  esp_err_t result = twai_new_node_onchip(&node_config, &backend->node);
  if (result != ESP_OK) {
    destroyBackend(backend);
    return result;
  }

  twai_event_callbacks_t callbacks{};
  callbacks.on_rx_done = receiveFrame;
  callbacks.on_tx_done = transmitDone;
  result = twai_node_register_event_callbacks(backend->node, &callbacks,
                                               backend);
  if (result == ESP_OK && config.filter.enabled) {
    twai_mask_filter_config_t filter{};
    filter.id = config.filter.identifier;
    filter.mask = config.filter.mask;
    filter.is_ext = config.filter.extended;
    result = twai_node_config_mask_filter(backend->node, 0, &filter);
  }
  if (result == ESP_OK)
    result = twai_node_enable(backend->node);
  if (result != ESP_OK) {
    const esp_err_t cleanup_result = releaseNode(backend);
    if (cleanup_result != ESP_OK)
      backend_ = backend;
    return result;
  }
  backend_ = backend;
#else
  auto *backend = new (std::nothrow) Backend{};
  if (backend == nullptr)
    return ESP_ERR_NO_MEM;

  twai_timing_config_t timing{};
  if (!timingFrom(config.bitrate, timing)) {
    delete backend;
    return ESP_ERR_INVALID_ARG;
  }
  twai_general_config_t general =
      TWAI_GENERAL_CONFIG_DEFAULT(config.tx, config.rx, modeFrom(config.mode));
  general.tx_queue_len = 1;
  general.rx_queue_len = config.rx_queue_depth;
  const twai_filter_config_t filter = filterFrom(config.filter);
  esp_err_t result = twai_driver_install(&general, &timing, &filter);
  if (result != ESP_OK) {
    delete backend;
    return result;
  }
  backend_ = backend;
  result = twai_start();
  if (result != ESP_OK) {
    if (twai_driver_uninstall() == ESP_OK) {
      backend_ = nullptr;
      delete backend;
    }
    return result;
  }
  backend->running = true;
#endif

  initialized_ = true;
  return ESP_OK;
}

esp_err_t CANCREATE::end() {
  if (backend_ == nullptr)
    return ESP_ERR_INVALID_STATE;

#if ESP_IDF_VERSION_MAJOR >= 6
  auto *backend = static_cast<Backend *>(backend_);
  const esp_err_t result = releaseNode(backend);
  if (result == ESP_OK) {
    backend_ = nullptr;
    initialized_ = false;
  }
  return result;
#else
  auto *backend = static_cast<Backend *>(backend_);
  esp_err_t stop_result = ESP_OK;
  if (backend->running) {
    stop_result = twai_stop();
    if (stop_result == ESP_OK || stop_result == ESP_ERR_INVALID_STATE)
      backend->running = false;
  }
  const esp_err_t uninstall_result = twai_driver_uninstall();
  if (uninstall_result != ESP_OK)
    return uninstall_result;
  backend_ = nullptr;
  initialized_ = false;
  delete backend;
  return stop_result == ESP_OK || stop_result == ESP_ERR_INVALID_STATE
             ? ESP_OK
             : stop_result;
#endif
}

esp_err_t CANCREATE::write(const Frame &frame, uint32_t timeout_ms) {
  if (!initialized_ || backend_ == nullptr)
    return ESP_ERR_INVALID_STATE;
  TickType_t timeout_ticks{};
  if (!timeoutToTicks(timeout_ms, timeout_ticks) || frame.data_length > 8 ||
      !validIdentifier(frame.identifier, frame.extended))
    return ESP_ERR_INVALID_ARG;

#if ESP_IDF_VERSION_MAJOR >= 6
  auto *backend = static_cast<Backend *>(backend_);
  if (xSemaphoreTake(backend->tx_available, timeout_ticks) != pdTRUE)
    return ESP_ERR_TIMEOUT;
  backend->tx_frame = {};
  backend->tx_frame.header.id = frame.identifier;
  backend->tx_frame.header.dlc = frame.data_length;
  backend->tx_frame.header.ide = frame.extended;
  backend->tx_frame.header.rtr = frame.remote;
  std::memcpy(backend->tx_data, frame.data, frame.data_length);
  backend->tx_frame.buffer = backend->tx_data;
  backend->tx_frame.buffer_len = frame.data_length;
  const esp_err_t result = twai_node_transmit(
      backend->node, &backend->tx_frame, static_cast<int>(timeout_ms));
  if (result != ESP_OK)
    (void)xSemaphoreGive(backend->tx_available);
  return result;
#else
  twai_message_t message{};
  message.identifier = frame.identifier;
  message.data_length_code = frame.data_length;
  message.extd = frame.extended;
  message.rtr = frame.remote;
  std::memcpy(message.data, frame.data, frame.data_length);
  return twai_transmit(&message, timeout_ticks);
#endif
}

esp_err_t CANCREATE::read(Frame &frame, uint32_t timeout_ms) {
  if (!initialized_ || backend_ == nullptr)
    return ESP_ERR_INVALID_STATE;
  TickType_t timeout_ticks{};
  if (!timeoutToTicks(timeout_ms, timeout_ticks))
    return ESP_ERR_INVALID_ARG;

  Frame next{};
#if ESP_IDF_VERSION_MAJOR >= 6
  RawFrame raw{};
  auto *backend = static_cast<Backend *>(backend_);
  if (xQueueReceive(backend->rx_queue, &raw, timeout_ticks) != pdTRUE)
    return ESP_ERR_TIMEOUT;
  if (raw.header.dlc > sizeof(next.data) ||
      !validIdentifier(raw.header.id, raw.header.ide))
    return ESP_ERR_INVALID_SIZE;
  next.identifier = raw.header.id;
  next.data_length = raw.header.dlc;
  next.extended = raw.header.ide;
  next.remote = raw.header.rtr;
  std::memcpy(next.data, raw.data, next.data_length);
#else
  twai_message_t message{};
  const esp_err_t result = twai_receive(&message, timeout_ticks);
  if (result != ESP_OK)
    return result;
  if (message.data_length_code > sizeof(next.data) ||
      !validIdentifier(message.identifier, message.extd))
    return ESP_ERR_INVALID_SIZE;
  next.identifier = message.identifier;
  next.data_length = message.data_length_code;
  next.extended = message.extd;
  next.remote = message.rtr;
  std::memcpy(next.data, message.data, next.data_length);
#endif
  frame = next;
  return ESP_OK;
}

esp_err_t CANCREATE::available(std::size_t &count) const {
  if (!initialized_ || backend_ == nullptr)
    return ESP_ERR_INVALID_STATE;
#if ESP_IDF_VERSION_MAJOR >= 6
  count = uxQueueMessagesWaiting(static_cast<Backend *>(backend_)->rx_queue);
  return ESP_OK;
#else
  twai_status_info_t info{};
  const esp_err_t result = twai_get_status_info(&info);
  if (result == ESP_OK)
    count = info.msgs_to_rx;
  return result;
#endif
}

esp_err_t CANCREATE::getStatus(Status &status) const {
  if (!initialized_ || backend_ == nullptr)
    return ESP_ERR_INVALID_STATE;
  Status next{};
#if ESP_IDF_VERSION_MAJOR >= 6
  auto *backend = static_cast<Backend *>(backend_);
  twai_node_status_t node_status{};
  twai_node_record_t record{};
  const esp_err_t result =
      twai_node_get_info(backend->node, &node_status, &record);
  if (result != ESP_OK)
    return result;
  next.state = stateFrom(node_status.state);
  next.pending_tx = uxSemaphoreGetCount(backend->tx_available) == 0 ? 1 : 0;
  next.pending_rx = uxQueueMessagesWaiting(backend->rx_queue);
  next.tx_error_count = node_status.tx_error_count;
  next.rx_error_count = node_status.rx_error_count;
  next.bus_error_count = record.bus_err_num;
  next.dropped_rx_count =
      __atomic_load_n(&backend->dropped_rx, __ATOMIC_RELAXED);
#else
  twai_status_info_t info{};
  const esp_err_t result = twai_get_status_info(&info);
  if (result != ESP_OK)
    return result;
  next.state = stateFrom(info.state);
  next.pending_tx = info.msgs_to_tx;
  next.pending_rx = info.msgs_to_rx;
  next.tx_error_count = info.tx_error_counter;
  next.rx_error_count = info.rx_error_counter;
  next.bus_error_count = info.bus_error_count;
  next.dropped_rx_count = info.rx_missed_count + info.rx_overrun_count;
#endif
  status = next;
  return ESP_OK;
}

esp_err_t CANCREATE::recover(uint32_t timeout_ms) {
  if (!initialized_ || backend_ == nullptr)
    return ESP_ERR_INVALID_STATE;
  TickType_t ignored{};
  if (!timeoutToTicks(timeout_ms, ignored))
    return ESP_ERR_INVALID_ARG;

#if ESP_IDF_VERSION_MAJOR >= 6
  auto *backend = static_cast<Backend *>(backend_);
  twai_node_status_t initial{};
  esp_err_t result = twai_node_get_info(backend->node, &initial, nullptr);
  if (result != ESP_OK)
    return result;
  if (initial.state != TWAI_ERROR_BUS_OFF)
    return ESP_ERR_INVALID_STATE;
  result = twai_node_recover(backend->node);
  if (result != ESP_OK)
    return result;
#else
  twai_status_info_t initial{};
  esp_err_t result = twai_get_status_info(&initial);
  if (result != ESP_OK)
    return result;
  if (initial.state == TWAI_STATE_RUNNING)
    return ESP_ERR_INVALID_STATE;
  if (initial.state == TWAI_STATE_STOPPED) {
    result = twai_start();
    if (result == ESP_OK)
      static_cast<Backend *>(backend_)->running = true;
    return result;
  }
  if (initial.state == TWAI_STATE_BUS_OFF) {
    result = twai_initiate_recovery();
    if (result != ESP_OK)
      return result;
  }
#endif

  const int64_t deadline = avi_micros() + int64_t{timeout_ms} * 1000;
  do {
#if ESP_IDF_VERSION_MAJOR >= 6
    twai_node_status_t info{};
    result = twai_node_get_info(static_cast<Backend *>(backend_)->node, &info,
                                nullptr);
    if (result != ESP_OK)
      return result;
    if (info.state != TWAI_ERROR_BUS_OFF)
      return ESP_OK;
#else
    twai_status_info_t info{};
    result = twai_get_status_info(&info);
    if (result != ESP_OK)
      return result;
    if (info.state == TWAI_STATE_STOPPED) {
      result = twai_start();
      if (result == ESP_OK)
        static_cast<Backend *>(backend_)->running = true;
      return result;
    }
#endif
    if (timeout_ms == 0)
      break;
    avi_delay_ms(1);
  } while (avi_micros() < deadline);
  return ESP_ERR_TIMEOUT;
}
