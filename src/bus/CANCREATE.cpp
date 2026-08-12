#include "CANCREATE.h"

#include <cstring>
#include <new>

#include "../compatibility/timeout_internal.h"
#include "avi_esp_libs/compatibility.h"
#include "esp_idf_version.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

constexpr uint32_t kStandardIdMask = 0x7FF;
constexpr uint32_t kExtendedIdMask = 0x1FFFFFFF;
constexpr auto kTestTimeout = avi::Timeout::milliseconds(1000);

bool validBitrate(CANCREATE::Bitrate bitrate) {
  switch (bitrate) {
  case CANCREATE::Bitrate::kbps25:
  case CANCREATE::Bitrate::kbps50:
  case CANCREATE::Bitrate::kbps100:
  case CANCREATE::Bitrate::kbps125:
  case CANCREATE::Bitrate::kbps250:
  case CANCREATE::Bitrate::kbps500:
  case CANCREATE::Bitrate::kbps800:
  case CANCREATE::Bitrate::mbps1:
    return true;
  default:
    return false;
  }
}

bool validIdentifier(uint32_t identifier, bool extended) {
  return identifier <= (extended ? kExtendedIdMask : kStandardIdMask);
}

bool applicationIdentifier(uint32_t identifier, bool extended) {
  return extended || identifier <= CANCREATE::kApplicationIdMax;
}

bool validConfig(const CANCREATE::Config &config) {
  if (!GPIO_IS_VALID_OUTPUT_GPIO(config.tx) || !GPIO_IS_VALID_GPIO(config.rx) ||
      !validBitrate(config.bitrate) || config.rx_queue_depth == 0)
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
#include "esp_heap_caps.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "freertos/idf_additions.h"
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
  uint32_t recovering{};
  uint32_t tx_success{};
  uint32_t allow_diagnostic{};
};

Backend *createBackend() {
  void *memory =
      heap_caps_malloc(sizeof(Backend), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  return memory == nullptr ? nullptr : new (memory) Backend{};
}

void destroyBackend(Backend *backend) {
  if (backend->tx_available != nullptr)
    vSemaphoreDeleteWithCaps(backend->tx_available);
  if (backend->rx_queue != nullptr)
    vQueueDeleteWithCaps(backend->rx_queue);
  backend->~Backend();
  heap_caps_free(backend);
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
  // SAFETY: contextはコールバック解除を含むノード削除完了まで生存する
  // Backendであり、ISR内では固定長データのキュー送信と破棄数更新だけを行う。
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
  const bool allow_diagnostic =
      __atomic_load_n(&backend->allow_diagnostic, __ATOMIC_ACQUIRE) != 0;
  if (!allow_diagnostic && !frame.header.ide &&
      frame.header.id > CANCREATE::kApplicationIdMax)
    return false;
  raw.header = frame.header;
  BaseType_t task_awoken = pdFALSE;
  if (xQueueSendFromISR(backend->rx_queue, &raw, &task_awoken) != pdTRUE)
    __atomic_fetch_add(&backend->dropped_rx, 1U, __ATOMIC_RELAXED);
  return task_awoken == pdTRUE;
}

bool IRAM_ATTR transmitDone(twai_node_handle_t,
                            const twai_tx_done_event_data_t *event,
                            void *context) {
  // SAFETY: contextと送信領域はnode delete完了まで生存する。eventはcallback中
  // だけ有効であり、値を固定領域へコピーする。ISRではFromISR APIだけを使い、
  // heap確保、ログ、ブロッキング処理を行わない。
  auto *backend = static_cast<Backend *>(context);
  __atomic_store_n(&backend->tx_success, event->is_tx_success ? 1U : 0U,
                   __ATOMIC_RELEASE);
  BaseType_t task_awoken = pdFALSE;
  (void)xSemaphoreGiveFromISR(backend->tx_available, &task_awoken);
  return task_awoken == pdTRUE;
}

bool IRAM_ATTR stateChanged(twai_node_handle_t,
                            const twai_state_change_event_data_t *event,
                            void *context) {
  // SAFETY: contextはノード削除完了まで内部RAM上で生存するBackendである。
  // ISRでは不可分な状態更新とセマフォ返却だけを行い、通信やブロッキング処理は
  // 行わない。BUS_OFFではドライバがTX完了を通知しない場合にも送信領域を回収する。
  auto *backend = static_cast<Backend *>(context);
  if (event->new_sta != TWAI_ERROR_BUS_OFF) {
    __atomic_store_n(&backend->recovering, 0U, __ATOMIC_RELEASE);
    return false;
  }

  BaseType_t task_awoken = pdFALSE;
  (void)xSemaphoreGiveFromISR(backend->tx_available, &task_awoken);
  return task_awoken == pdTRUE;
}

CANCREATE::State stateFrom(twai_error_state_t state, bool recovering) {
  if (state == TWAI_ERROR_BUS_OFF)
    return recovering ? CANCREATE::State::recovering
                      : CANCREATE::State::bus_off;
  return CANCREATE::State::running;
}

} // namespace

#else

#include "driver/twai.h"

namespace {

struct Backend {
  bool running{false};
  CANCREATE::Filter filter{};
  CANCREATE::Frame *prefetched{};
  std::size_t prefetch_capacity{};
  std::size_t prefetch_head{};
  std::size_t prefetch_count{};
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

bool timingFrom(CANCREATE::Bitrate bitrate, twai_timing_config_t &timing) {
  switch (bitrate) {
  case CANCREATE::Bitrate::kbps25:
    timing = TWAI_TIMING_CONFIG_25KBITS();
    return true;
  case CANCREATE::Bitrate::kbps50:
    timing = TWAI_TIMING_CONFIG_50KBITS();
    return true;
  case CANCREATE::Bitrate::kbps100:
    timing = TWAI_TIMING_CONFIG_100KBITS();
    return true;
  case CANCREATE::Bitrate::kbps125:
    timing = TWAI_TIMING_CONFIG_125KBITS();
    return true;
  case CANCREATE::Bitrate::kbps250:
    timing = TWAI_TIMING_CONFIG_250KBITS();
    return true;
  case CANCREATE::Bitrate::kbps500:
    timing = TWAI_TIMING_CONFIG_500KBITS();
    return true;
  case CANCREATE::Bitrate::kbps800:
    timing = TWAI_TIMING_CONFIG_800KBITS();
    return true;
  case CANCREATE::Bitrate::mbps1:
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
  twai_filter_config_t result{};
  result.acceptance_code = filter.identifier << shift;
  result.acceptance_mask = ~(filter.mask << shift);
  result.single_filter = true;
  return result;
}

bool filterAccepts(const CANCREATE::Filter &filter,
                   const twai_message_t &message) {
  return !filter.enabled ||
         (message.extd == filter.extended &&
          ((message.identifier ^ filter.identifier) & filter.mask) == 0);
}

void copyFrame(const twai_message_t &message, CANCREATE::Frame &frame) {
  frame = {};
  frame.identifier = message.identifier;
  frame.data_length = message.data_length_code;
  frame.extended = message.extd;
  frame.remote = message.rtr;
  std::memcpy(frame.data, message.data, frame.data_length);
}

esp_err_t drainReceiveQueue(Backend &backend) {
  twai_message_t message{};
  while (backend.prefetch_count < backend.prefetch_capacity &&
         twai_receive(&message, 0) == ESP_OK) {
    if (message.data_length_code > 8 ||
        !validIdentifier(message.identifier, message.extd))
      return ESP_ERR_INVALID_SIZE;
    if (!applicationIdentifier(message.identifier, message.extd) ||
        !filterAccepts(backend.filter, message))
      continue;
    const std::size_t slot = (backend.prefetch_head + backend.prefetch_count) %
                             backend.prefetch_capacity;
    copyFrame(message, backend.prefetched[slot]);
    ++backend.prefetch_count;
  }
  return ESP_OK;
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

esp_err_t CANCREATE::begin(gpio_num_t tx, gpio_num_t rx, Bitrate bitrate) {
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
  const esp_err_t result =
      start(config, config.mode == Mode::no_ack, false, -1);
  if (result == ESP_OK)
    config_ = config;
  return result;
}

esp_err_t CANCREATE::start(const Config &config, bool self_test, bool loopback,
                           int8_t retry_count) {

#if ESP_IDF_VERSION_MAJOR >= 6
  auto *backend = createBackend();
  if (backend == nullptr)
    return ESP_ERR_NO_MEM;
  backend->rx_queue =
      xQueueCreateWithCaps(config.rx_queue_depth, sizeof(RawFrame),
                           MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  backend->tx_available =
      xSemaphoreCreateBinaryWithCaps(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
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
  node_config.bit_timing.bitrate = static_cast<uint32_t>(config.bitrate);
  node_config.tx_queue_depth = 1;
  node_config.fail_retry_cnt = retry_count;
  node_config.flags.enable_self_test = self_test;
  node_config.flags.enable_loopback = loopback;
  node_config.flags.enable_listen_only = config.mode == Mode::listen_only;

  esp_err_t result = twai_new_node_onchip(&node_config, &backend->node);
  if (result != ESP_OK) {
    destroyBackend(backend);
    return result;
  }

  twai_event_callbacks_t callbacks{};
  callbacks.on_rx_done = receiveFrame;
  callbacks.on_tx_done = transmitDone;
  callbacks.on_state_change = stateChanged;
  result =
      twai_node_register_event_callbacks(backend->node, &callbacks, backend);
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
  backend->filter = config.filter;
  backend->prefetched =
      new (std::nothrow) Frame[static_cast<std::size_t>(config.rx_queue_depth)];
  if (backend->prefetched == nullptr) {
    delete backend;
    return ESP_ERR_NO_MEM;
  }
  backend->prefetch_capacity = config.rx_queue_depth;

  twai_timing_config_t timing{};
  if (!timingFrom(config.bitrate, timing)) {
    delete[] backend->prefetched;
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
    delete[] backend->prefetched;
    delete backend;
    return result;
  }
  backend_ = backend;
  result = twai_start();
  if (result != ESP_OK) {
    if (twai_driver_uninstall() == ESP_OK) {
      backend_ = nullptr;
      delete[] backend->prefetched;
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
  initialized_ = false;

#if ESP_IDF_VERSION_MAJOR >= 6
  auto *backend = static_cast<Backend *>(backend_);
  const esp_err_t result = releaseNode(backend);
  if (result == ESP_OK) {
    backend_ = nullptr;
    config_ = Config{};
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
  delete[] backend->prefetched;
  delete backend;
  config_ = Config{};
  return stop_result == ESP_OK || stop_result == ESP_ERR_INVALID_STATE
             ? ESP_OK
             : stop_result;
#endif
}

esp_err_t CANCREATE::write(const Frame &frame, avi::Timeout timeout) {
  if (!initialized_ || backend_ == nullptr)
    return ESP_ERR_INVALID_STATE;
  TickType_t timeout_ticks{};
  if (frame.data_length > 8)
    return ESP_ERR_INVALID_SIZE;
  if (avi::internal::timeoutToTicks(timeout, timeout_ticks) != ESP_OK ||
      !validIdentifier(frame.identifier, frame.extended) ||
      !applicationIdentifier(frame.identifier, frame.extended))
    return ESP_ERR_INVALID_ARG;

#if ESP_IDF_VERSION_MAJOR >= 6
  auto *backend = static_cast<Backend *>(backend_);
  if (xSemaphoreTake(backend->tx_available, timeout_ticks) != pdTRUE)
    return timeout.isNoWait() ? ESP_ERR_NOT_FINISHED : ESP_ERR_TIMEOUT;
  backend->tx_frame = {};
  backend->tx_frame.header.id = frame.identifier;
  backend->tx_frame.header.dlc = frame.data_length;
  backend->tx_frame.header.ide = frame.extended;
  backend->tx_frame.header.rtr = frame.remote;
  std::memcpy(backend->tx_data, frame.data, frame.data_length);
  backend->tx_frame.buffer = backend->tx_data;
  backend->tx_frame.buffer_len = frame.data_length;
  int timeout_ms{};
  if (avi::internal::timeoutToIntMilliseconds(timeout, timeout_ms) != ESP_OK) {
    (void)xSemaphoreGive(backend->tx_available);
    return ESP_ERR_INVALID_ARG;
  }
  const esp_err_t result =
      twai_node_transmit(backend->node, &backend->tx_frame, timeout_ms);
  if (result != ESP_OK)
    (void)xSemaphoreGive(backend->tx_available);
  return timeout.isNoWait() && result == ESP_ERR_TIMEOUT ? ESP_ERR_NOT_FINISHED
                                                         : result;
#else
  twai_message_t message{};
  message.identifier = frame.identifier;
  message.data_length_code = frame.data_length;
  message.extd = frame.extended;
  message.rtr = frame.remote;
  std::memcpy(message.data, frame.data, frame.data_length);
  const esp_err_t result = twai_transmit(&message, timeout_ticks);
  return timeout.isNoWait() && result == ESP_ERR_TIMEOUT ? ESP_ERR_NOT_FINISHED
                                                         : result;
#endif
}

esp_err_t CANCREATE::write(uint32_t identifier, uint8_t value,
                           avi::Timeout timeout) {
  return write(identifier, &value, 1, timeout);
}

esp_err_t CANCREATE::write(uint32_t identifier, const uint8_t *data,
                           std::size_t length, avi::Timeout timeout) {
  if ((data == nullptr && length != 0) || length > 8 ||
      identifier > kApplicationIdMax)
    return length > 8 ? ESP_ERR_INVALID_SIZE : ESP_ERR_INVALID_ARG;
  Frame frame{};
  frame.identifier = identifier;
  frame.data_length = static_cast<uint8_t>(length);
  if (length != 0)
    std::memcpy(frame.data, data, length);
  return write(frame, timeout);
}

esp_err_t CANCREATE::writeText(uint32_t identifier, std::string_view text,
                               avi::Timeout timeout) {
  if (text.size() > 8)
    return ESP_ERR_INVALID_SIZE;
  return write(identifier, reinterpret_cast<const uint8_t *>(text.data()),
               text.size(), timeout);
}

esp_err_t CANCREATE::read(Frame &frame, avi::Timeout timeout) {
  if (!initialized_ || backend_ == nullptr)
    return ESP_ERR_INVALID_STATE;
  TickType_t timeout_ticks{};
  if (avi::internal::timeoutToTicks(timeout, timeout_ticks) != ESP_OK)
    return ESP_ERR_INVALID_ARG;

  Frame next{};
#if ESP_IDF_VERSION_MAJOR >= 6
  RawFrame raw{};
  auto *backend = static_cast<Backend *>(backend_);
  if (xQueueReceive(backend->rx_queue, &raw, timeout_ticks) != pdTRUE)
    return timeout.isNoWait() ? ESP_ERR_NOT_FINISHED : ESP_ERR_TIMEOUT;
  if (raw.header.dlc > sizeof(next.data) ||
      !validIdentifier(raw.header.id, raw.header.ide))
    return ESP_ERR_INVALID_SIZE;
  next.identifier = raw.header.id;
  next.data_length = raw.header.dlc;
  next.extended = raw.header.ide;
  next.remote = raw.header.rtr;
  std::memcpy(next.data, raw.data, next.data_length);
#else
  auto *backend = static_cast<Backend *>(backend_);
  if (backend->prefetch_count != 0) {
    next = backend->prefetched[backend->prefetch_head];
    backend->prefetch_head =
        (backend->prefetch_head + 1) % backend->prefetch_capacity;
    --backend->prefetch_count;
    frame = next;
    return ESP_OK;
  }
  const TickType_t started_at = xTaskGetTickCount();
  TickType_t remaining_ticks = timeout_ticks;
  twai_message_t message{};
  for (;;) {
    const esp_err_t result = twai_receive(&message, remaining_ticks);
    if (result != ESP_OK)
      return timeout.isNoWait() && result == ESP_ERR_TIMEOUT
                 ? ESP_ERR_NOT_FINISHED
                 : result;
    if (message.data_length_code > sizeof(next.data) ||
        !validIdentifier(message.identifier, message.extd))
      return ESP_ERR_INVALID_SIZE;
    if (applicationIdentifier(message.identifier, message.extd) &&
        filterAccepts(backend->filter, message))
      break;

    if (!timeout.isForever() && !timeout.isNoWait()) {
      const TickType_t elapsed = xTaskGetTickCount() - started_at;
      if (elapsed >= timeout_ticks)
        return ESP_ERR_TIMEOUT;
      remaining_ticks = timeout_ticks - elapsed;
    }
  }
  copyFrame(message, next);
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
  auto &backend = *static_cast<Backend *>(backend_);
  const esp_err_t result = drainReceiveQueue(backend);
  if (result == ESP_OK)
    count = backend.prefetch_count;
  return result;
#endif
}

std::size_t CANCREATE::available() const {
  std::size_t count{};
  return available(count) == ESP_OK ? count : 0;
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
  next.state =
      stateFrom(node_status.state,
                __atomic_load_n(&backend->recovering, __ATOMIC_ACQUIRE) != 0);
  next.pending_tx = uxSemaphoreGetCount(backend->tx_available) == 0 ? 1 : 0;
  next.pending_rx = uxQueueMessagesWaiting(backend->rx_queue);
  next.tx_error_count = node_status.tx_error_count;
  next.rx_error_count = node_status.rx_error_count;
  next.bus_error_count = record.bus_err_num;
  next.dropped_rx_count =
      __atomic_load_n(&backend->dropped_rx, __ATOMIC_RELAXED);
#else
  auto &backend = *static_cast<Backend *>(backend_);
  const esp_err_t drain = drainReceiveQueue(backend);
  if (drain != ESP_OK)
    return drain;
  twai_status_info_t info{};
  const esp_err_t result = twai_get_status_info(&info);
  if (result != ESP_OK)
    return result;
  next.state = stateFrom(info.state);
  next.pending_tx = info.msgs_to_tx;
  next.pending_rx = backend.prefetch_count;
  next.tx_error_count = info.tx_error_counter;
  next.rx_error_count = info.rx_error_counter;
  next.bus_error_count = info.bus_error_count;
  next.dropped_rx_count = info.rx_missed_count + info.rx_overrun_count;
#endif
  status = next;
  return ESP_OK;
}

esp_err_t CANCREATE::recover(avi::Timeout timeout) {
  if (!initialized_ || backend_ == nullptr)
    return ESP_ERR_INVALID_STATE;
  TickType_t ignored{};
  if (avi::internal::timeoutToTicks(timeout, ignored) != ESP_OK)
    return ESP_ERR_INVALID_ARG;

#if ESP_IDF_VERSION_MAJOR >= 6
  auto *backend = static_cast<Backend *>(backend_);
  twai_node_status_t initial{};
  esp_err_t result = twai_node_get_info(backend->node, &initial, nullptr);
  if (result != ESP_OK)
    return result;
  if (initial.state != TWAI_ERROR_BUS_OFF)
    return ESP_ERR_INVALID_STATE;
  uint32_t expected = 0;
  if (__atomic_compare_exchange_n(&backend->recovering, &expected, 1U, false,
                                  __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE)) {
    result = twai_node_recover(backend->node);
    if (result != ESP_OK) {
      __atomic_store_n(&backend->recovering, 0U, __ATOMIC_RELEASE);
      return result;
    }
  }
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

  avi::internal::Deadline deadline{};
  if (avi::internal::makeDeadline(timeout, deadline) != ESP_OK)
    return ESP_ERR_INVALID_ARG;
  do {
#if ESP_IDF_VERSION_MAJOR >= 6
    twai_node_status_t info{};
    result = twai_node_get_info(static_cast<Backend *>(backend_)->node, &info,
                                nullptr);
    if (result != ESP_OK)
      return result;
    if (info.state != TWAI_ERROR_BUS_OFF) {
      __atomic_store_n(&static_cast<Backend *>(backend_)->recovering, 0U,
                       __ATOMIC_RELEASE);
      return ESP_OK;
    }
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
    if (timeout.isNoWait())
      return ESP_ERR_NOT_FINISHED;
    avi_delay_ms(1);
  } while (!avi::internal::expired(deadline));
  return ESP_ERR_TIMEOUT;
}

esp_err_t CANCREATE::test(TestResult &result) {
  if (!initialized_ || backend_ == nullptr)
    return ESP_ERR_INVALID_STATE;
  avi::internal::Deadline deadline{};
  if (avi::internal::makeDeadline(kTestTimeout, deadline) != ESP_OK)
    return ESP_ERR_INVALID_STATE;
  const auto remainingTimeout = [&deadline](avi::Timeout &remaining) {
    const int64_t now = avi_micros();
    if (now >= deadline.microseconds)
      return ESP_ERR_TIMEOUT;
    const uint64_t microseconds =
        static_cast<uint64_t>(deadline.microseconds - now);
    remaining = avi::Timeout::milliseconds((microseconds + 999U) / 1000U);
    return ESP_OK;
  };
  Status status{};
  esp_err_t test_error = getStatus(status);
  if (test_error != ESP_OK)
    return test_error;
  if (status.state == State::bus_off || status.state == State::recovering)
    return ESP_ERR_INVALID_STATE;

  const Config saved = config_;
  TestResult next{};
  test_error = end();
  bool normal_success = false;
  Config temporary = saved;
  temporary.mode = Mode::normal;
  temporary.filter = {};
  if (test_error == ESP_OK)
    test_error = start(temporary, false, false, 0);

  if (test_error == ESP_OK) {
    avi::Timeout remaining = avi::Timeout::milliseconds(1);
    test_error = remainingTimeout(remaining);
    TickType_t timeout_ticks{};
    if (test_error == ESP_OK)
      test_error = avi::internal::timeoutToTicks(remaining, timeout_ticks);
#if ESP_IDF_VERSION_MAJOR >= 6
    auto *backend = static_cast<Backend *>(backend_);
    if (test_error == ESP_OK &&
        xSemaphoreTake(backend->tx_available, timeout_ticks) == pdTRUE) {
      backend->tx_frame = {};
      backend->tx_frame.header.id = kTestIdentifier;
      backend->tx_frame.header.dlc = 0;
      backend->tx_frame.buffer = backend->tx_data;
      backend->tx_frame.buffer_len = 0;
      __atomic_store_n(&backend->tx_success, 0U, __ATOMIC_RELEASE);
      int timeout_value{};
      test_error = remainingTimeout(remaining);
      if (test_error == ESP_OK)
        test_error =
            avi::internal::timeoutToIntMilliseconds(remaining, timeout_value);
      if (test_error == ESP_OK)
        test_error = twai_node_transmit(backend->node, &backend->tx_frame,
                                        timeout_value);
      if (test_error == ESP_OK)
        test_error = remainingTimeout(remaining);
      if (test_error == ESP_OK)
        test_error = avi::internal::timeoutToTicks(remaining, timeout_ticks);
      if (test_error == ESP_OK &&
          xSemaphoreTake(backend->tx_available, timeout_ticks) == pdTRUE)
        normal_success =
            __atomic_load_n(&backend->tx_success, __ATOMIC_ACQUIRE) != 0;
      if (test_error != ESP_OK)
        (void)xSemaphoreGive(backend->tx_available);
    }
#else
    uint32_t alerts{};
    test_error = twai_reconfigure_alerts(
        TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED | TWAI_ALERT_BUS_ERROR,
        nullptr);
    twai_message_t message{};
    message.identifier = kTestIdentifier;
    message.ss = 1;
    if (test_error == ESP_OK)
      test_error = twai_transmit(&message, timeout_ticks);
    if (test_error == ESP_OK)
      test_error = remainingTimeout(remaining);
    if (test_error == ESP_OK)
      test_error = avi::internal::timeoutToTicks(remaining, timeout_ticks);
    if (test_error == ESP_OK) {
      const esp_err_t alert_result = twai_read_alerts(&alerts, timeout_ticks);
      if (alert_result == ESP_OK)
        normal_success = (alerts & TWAI_ALERT_TX_SUCCESS) != 0;
      else if (alert_result != ESP_ERR_TIMEOUT)
        test_error = alert_result;
    }
#endif
  }

  if (backend_ != nullptr) {
    const esp_err_t stop = end();
    if (test_error == ESP_OK && stop != ESP_OK)
      test_error = stop;
  }

  bool self_reception_success = false;
  if (test_error == ESP_OK && !normal_success) {
    temporary.mode = Mode::no_ack;
    test_error = start(temporary, true, true, 0);
    if (test_error == ESP_OK) {
      avi::Timeout remaining = avi::Timeout::milliseconds(1);
      test_error = remainingTimeout(remaining);
      TickType_t timeout_ticks{};
      if (test_error == ESP_OK)
        test_error = avi::internal::timeoutToTicks(remaining, timeout_ticks);
#if ESP_IDF_VERSION_MAJOR >= 6
      auto *backend = static_cast<Backend *>(backend_);
      __atomic_store_n(&backend->allow_diagnostic, 1U, __ATOMIC_RELEASE);
      if (test_error == ESP_OK &&
          xSemaphoreTake(backend->tx_available, timeout_ticks) == pdTRUE) {
        backend->tx_frame = {};
        backend->tx_frame.header.id = kTestIdentifier;
        backend->tx_frame.header.dlc = 0;
        backend->tx_frame.buffer = backend->tx_data;
        int timeout_value{};
        test_error = remainingTimeout(remaining);
        if (test_error == ESP_OK)
          test_error =
              avi::internal::timeoutToIntMilliseconds(remaining, timeout_value);
        if (test_error == ESP_OK)
          test_error = twai_node_transmit(backend->node, &backend->tx_frame,
                                          timeout_value);
        if (test_error == ESP_OK)
          test_error = remainingTimeout(remaining);
        if (test_error == ESP_OK)
          test_error = avi::internal::timeoutToTicks(remaining, timeout_ticks);
        if (test_error == ESP_OK &&
            xSemaphoreTake(backend->tx_available, timeout_ticks) == pdTRUE) {
          test_error = remainingTimeout(remaining);
          if (test_error == ESP_OK)
            test_error =
                avi::internal::timeoutToTicks(remaining, timeout_ticks);
          RawFrame received{};
          self_reception_success = test_error == ESP_OK &&
                                   xQueueReceive(backend->rx_queue, &received,
                                                 timeout_ticks) == pdTRUE &&
                                   !received.header.ide &&
                                   received.header.id == kTestIdentifier;
        }
        if (test_error != ESP_OK)
          (void)xSemaphoreGive(backend->tx_available);
      }
#else
      twai_message_t message{};
      message.identifier = kTestIdentifier;
      message.ss = 1;
      message.self = 1;
      test_error = twai_transmit(&message, timeout_ticks);
      twai_message_t received{};
      if (test_error == ESP_OK)
        test_error = remainingTimeout(remaining);
      if (test_error == ESP_OK)
        test_error = avi::internal::timeoutToTicks(remaining, timeout_ticks);
      if (test_error == ESP_OK)
        self_reception_success =
            twai_receive(&received, timeout_ticks) == ESP_OK &&
            !received.extd && received.identifier == kTestIdentifier;
#endif
    }
    if (backend_ != nullptr) {
      const esp_err_t stop = end();
      if (test_error == ESP_OK && stop != ESP_OK)
        test_error = stop;
    }
  }

  next.state = normal_success
                   ? TestState::success
                   : (self_reception_success ? TestState::no_peer_response
                                             : TestState::controller_failure);
  esp_err_t restore = ESP_OK;
  if (backend_ != nullptr)
    restore = ESP_ERR_INVALID_STATE;
  else
    restore = start(saved, saved.mode == Mode::no_ack, false, -1);
  if (restore == ESP_OK) {
    config_ = saved;
    next.restored = true;
  } else {
    initialized_ = false;
    config_ = Config{};
  }
  result = next;
  return restore != ESP_OK ? restore : test_error;
}
