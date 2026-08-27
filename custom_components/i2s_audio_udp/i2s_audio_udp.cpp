#include "i2s_audio_udp.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"

#include <cstring>
#include <cmath>
#include <algorithm>
#include <errno.h>
#include <driver/gpio.h>

// Include AEC if available
#ifdef USE_ESP_AEC
#include "../esp_aec/esp_aec.h"
#endif

#include "esp_heap_caps.h"

namespace esphome {
namespace i2s_audio_udp {

static const char *TAG = "i2s_audio_udp";

// Audio configuration
static const size_t AUDIO_BUFFER_SIZE = 1024;
static const size_t DMA_BUFFER_COUNT = 8;
static const size_t DMA_BUFFER_SIZE = 512;

// Sized to hold well over the prebuffer plus a burst. DMA alone can absorb
// DMA_BUFFER_COUNT * DMA_BUFFER_SIZE frames (8192 bytes at 16-bit mono), so a
// ring anywhere near that leaves no cushion at all.
static const size_t RING_BUFFER_SIZE = 32768;

// FreeRTOS task parameters
static const size_t TASK_STACK_SIZE = 8192;  // Increased for AEC processing
static const ssize_t TASK_PRIORITY = 19;

// Local rather than M_PI: math.h exposes it inconsistently across toolchains
// and this needs no more precision than float carries.
static const float TWO_PI_F = 6.28318530718f;

void I2SAudioUDP::setup() {
  ESP_LOGD(TAG, "Setting up...");

  // Deduce bus mode and audio mode from pin configuration
  this->deduce_modes_();

  // Enable speaker amplifier if configured
  if (this->speaker_enable_pin_ >= 0) {
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << this->speaker_enable_pin_);
    io_conf.mode = GPIO_MODE_OUTPUT;
    gpio_config(&io_conf);
    gpio_set_level((gpio_num_t)this->speaker_enable_pin_, 1);
  }

  ESP_LOGI(TAG, "Setup complete");
}

void I2SAudioUDP::dump_config() {
  ESP_LOGCONFIG(TAG, "I2S Audio UDP:");
  ESP_LOGCONFIG(TAG, "  Bus Mode: %s", this->bus_mode_ == I2S_BUS_SINGLE ? "SINGLE" : "DUAL");
  ESP_LOGCONFIG(TAG, "  Audio Mode: %s", this->get_audio_mode_text());
  ESP_LOGCONFIG(TAG, "  Sample Rate: %d Hz", this->sample_rate_);
  if (this->bus_mode_ == I2S_BUS_DUAL) {
    ESP_LOGCONFIG(TAG, "  Mic Config: %d-bit, channel=%s, gain=%dx",
                  this->mic_bits_per_sample_,
                  this->mic_channel_ == MIC_CHANNEL_LEFT ? "left" : "right",
                  this->mic_gain_);
  }
  if (this->aec_ != nullptr) {
    ESP_LOGCONFIG(TAG, "  AEC: enabled");
  }
}

void I2SAudioUDP::deduce_modes_() {
  // Determine bus mode
  if (this->mic_lrclk_pin_ >= 0 || this->speaker_lrclk_pin_ >= 0) {
    this->bus_mode_ = I2S_BUS_DUAL;
  } else {
    this->bus_mode_ = I2S_BUS_SINGLE;
  }

  // Determine audio mode
  bool has_mic = false;
  bool has_speaker = false;

  if (this->bus_mode_ == I2S_BUS_SINGLE) {
    has_mic = (this->i2s_din_pin_ >= 0);
    has_speaker = (this->i2s_dout_pin_ >= 0);
  } else {
    has_mic = (this->mic_din_pin_ >= 0);
    has_speaker = (this->speaker_dout_pin_ >= 0);
  }

  if (has_mic && has_speaker) {
    this->audio_mode_ = AUDIO_MODE_FULL_DUPLEX;
  } else if (has_mic) {
    this->audio_mode_ = AUDIO_MODE_TX_ONLY;
  } else {
    this->audio_mode_ = AUDIO_MODE_RX_ONLY;
  }
}

const char* I2SAudioUDP::get_audio_mode_text() const {
  switch (this->audio_mode_) {
    case AUDIO_MODE_TX_ONLY: return "TX_ONLY";
    case AUDIO_MODE_RX_ONLY: return "RX_ONLY";
    case AUDIO_MODE_FULL_DUPLEX: return "FULL_DUPLEX";
    default: return "UNKNOWN";
  }
}

void I2SAudioUDP::set_volume(float volume) {
  this->volume_ = std::clamp(volume, 0.0f, 1.0f);
  ESP_LOGI(TAG, "Volume set to %.0f%%", this->volume_ * 100);
}

void I2SAudioUDP::apply_software_volume_(int16_t *buffer, size_t samples) {
  if (this->volume_ >= 0.99f) return;
  for (size_t i = 0; i < samples; i++) {
    int32_t sample = buffer[i];
    sample = (int32_t)(sample * this->volume_);
    buffer[i] = (int16_t)std::clamp(sample, (int32_t)-32768, (int32_t)32767);
  }
}

void I2SAudioUDP::play_tone(uint32_t freq_hz, uint32_t duration_ms, float amplitude) {
  if (this->audio_mode_ == AUDIO_MODE_TX_ONLY) {
    ESP_LOGW(TAG, "play_tone: no speaker configured (TX_ONLY)");
    return;
  }
  if (duration_ms == 0 || freq_hz == 0)
    return;
  if (duration_ms > 5000)
    duration_ms = 5000;
  amplitude = std::clamp(amplitude, 0.0f, 1.0f);

  const uint32_t rate = this->sample_rate_;
  const size_t total = (size_t)((uint64_t)duration_ms * rate / 1000ULL);
  if (total == 0)
    return;
  // 5ms of linear ramp at each end. A tone that starts at full amplitude on a
  // zero crossing still steps the amplifier hard enough to click.
  const size_t ramp = std::min<size_t>(total / 2, rate / 200);

  const bool streaming = this->streaming_;
  bool temp_i2s = false;

  if (streaming) {
    // The audio task owns the I2S channel while streaming, so hand the tone to
    // it the same way the network does rather than writing concurrently.
    if (this->audio_ring_buffer_ == nullptr) {
      ESP_LOGW(TAG, "play_tone: no ring buffer");
      return;
    }
  } else if (this->tx_handle_ == nullptr) {
    const bool ok = (this->bus_mode_ == I2S_BUS_SINGLE) ? this->init_i2s_single_bus_()
                                                        : this->init_i2s_dual_bus_();
    if (!ok) {
      ESP_LOGE(TAG, "play_tone: could not bring I2S up");
      return;
    }
    temp_i2s = true;
  }

  static const size_t CHUNK = 256;
  int16_t chunk[CHUNK];
  const float step = TWO_PI_F * (float)freq_hz / (float)rate;
  float phase = 0.0f;
  size_t done = 0;

  while (done < total) {
    const size_t n = std::min(CHUNK, total - done);
    for (size_t i = 0; i < n; i++) {
      const size_t idx = done + i;
      float env = 1.0f;
      if (ramp > 0) {
        if (idx < ramp) {
          env = (float)idx / (float)ramp;
        } else if (idx >= total - ramp) {
          env = (float)(total - idx) / (float)ramp;
        }
      }
      chunk[i] = (int16_t)(sinf(phase) * amplitude * env * 32767.0f);
      phase += step;
      if (phase > TWO_PI_F)
        phase -= TWO_PI_F;
    }

    if (streaming) {
      this->audio_ring_buffer_->write((void *)chunk, n * sizeof(int16_t));
    } else {
      size_t written = 0;
      i2s_channel_write(this->tx_handle_, chunk, n * sizeof(int16_t), &written,
                        pdMS_TO_TICKS(200));
    }
    done += n;
  }

  if (temp_i2s) {
    // i2s_channel_write returns once the data is queued, not once it has been
    // clocked out. Tearing the channel down immediately truncates the tail.
    vTaskDelay(pdMS_TO_TICKS(120));
    this->deinit_i2s_();
  }

  ESP_LOGI(TAG, "play_tone: %u Hz for %u ms at %.0f%%", (unsigned)freq_hz,
           (unsigned)duration_ms, amplitude * 100.0f);
}

bool I2SAudioUDP::init_i2s_single_bus_() {
  ESP_LOGD(TAG, "Initializing I2S Single Bus...");

  bool need_tx = (this->audio_mode_ == AUDIO_MODE_RX_ONLY || this->audio_mode_ == AUDIO_MODE_FULL_DUPLEX);
  bool need_rx = (this->audio_mode_ == AUDIO_MODE_TX_ONLY || this->audio_mode_ == AUDIO_MODE_FULL_DUPLEX);

  i2s_chan_config_t chan_cfg = {
    .id = I2S_NUM_0,
    .role = I2S_ROLE_MASTER,
    .dma_desc_num = DMA_BUFFER_COUNT,
    .dma_frame_num = DMA_BUFFER_SIZE,
    .auto_clear = true,
    .intr_priority = 0,
  };

  i2s_chan_handle_t *tx_ptr = need_tx ? &this->tx_handle_ : nullptr;
  i2s_chan_handle_t *rx_ptr = need_rx ? &this->rx_handle_ : nullptr;

  esp_err_t err = i2s_new_channel(&chan_cfg, tx_ptr, rx_ptr);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create I2S channel: %s", esp_err_to_name(err));
    return false;
  }

  i2s_std_config_t std_cfg = {
    .clk_cfg = {
      .sample_rate_hz = this->sample_rate_,
      .clk_src = I2S_CLK_SRC_DEFAULT,
      .mclk_multiple = I2S_MCLK_MULTIPLE_256,
    },
    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
    .gpio_cfg = {
      .mclk = (gpio_num_t)this->i2s_mclk_pin_,
      .bclk = (gpio_num_t)this->i2s_bclk_pin_,
      .ws = (gpio_num_t)this->i2s_lrclk_pin_,
      .dout = (gpio_num_t)this->i2s_dout_pin_,
      .din = (gpio_num_t)this->i2s_din_pin_,
      .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
    },
  };

  std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;

  if (need_tx) {
    err = i2s_channel_init_std_mode(this->tx_handle_, &std_cfg);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to init TX channel: %s", esp_err_to_name(err));
      return false;
    }
    i2s_channel_enable(this->tx_handle_);
  }

  if (need_rx) {
    err = i2s_channel_init_std_mode(this->rx_handle_, &std_cfg);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to init RX channel: %s", esp_err_to_name(err));
      return false;
    }
    i2s_channel_enable(this->rx_handle_);
  }

  ESP_LOGD(TAG, "I2S Single Bus initialized");
  return true;
}

bool I2SAudioUDP::init_i2s_dual_bus_() {
  ESP_LOGD(TAG, "Initializing I2S Dual Bus...");

  bool need_mic = (this->audio_mode_ == AUDIO_MODE_TX_ONLY || this->audio_mode_ == AUDIO_MODE_FULL_DUPLEX);
  bool need_speaker = (this->audio_mode_ == AUDIO_MODE_RX_ONLY || this->audio_mode_ == AUDIO_MODE_FULL_DUPLEX);

  // Microphone on I2S_NUM_0
  if (need_mic) {
    i2s_chan_config_t mic_chan_cfg = {
      .id = I2S_NUM_0,
      .role = I2S_ROLE_MASTER,
      .dma_desc_num = DMA_BUFFER_COUNT,
      .dma_frame_num = DMA_BUFFER_SIZE,
      .auto_clear = true,
      .intr_priority = 0,
    };

    esp_err_t err = i2s_new_channel(&mic_chan_cfg, nullptr, &this->rx_handle_);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to create mic I2S channel: %s", esp_err_to_name(err));
      return false;
    }

    i2s_data_bit_width_t mic_bit_width = (this->mic_bits_per_sample_ == 32)
        ? I2S_DATA_BIT_WIDTH_32BIT : I2S_DATA_BIT_WIDTH_16BIT;

    i2s_std_config_t mic_std_cfg = {
      .clk_cfg = {
        .sample_rate_hz = this->sample_rate_,
        .clk_src = I2S_CLK_SRC_DEFAULT,
        .mclk_multiple = I2S_MCLK_MULTIPLE_256,
      },
      .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(mic_bit_width, I2S_SLOT_MODE_MONO),
      .gpio_cfg = {
        .mclk = GPIO_NUM_NC,
        .bclk = (gpio_num_t)this->mic_bclk_pin_,
        .ws = (gpio_num_t)this->mic_lrclk_pin_,
        .dout = GPIO_NUM_NC,
        .din = (gpio_num_t)this->mic_din_pin_,
        .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
      },
    };

    mic_std_cfg.slot_cfg.slot_bit_width = (this->mic_bits_per_sample_ == 32)
        ? I2S_SLOT_BIT_WIDTH_32BIT : I2S_SLOT_BIT_WIDTH_16BIT;
    mic_std_cfg.slot_cfg.slot_mask = (this->mic_channel_ == MIC_CHANNEL_RIGHT)
        ? I2S_STD_SLOT_RIGHT : I2S_STD_SLOT_LEFT;

    err = i2s_channel_init_std_mode(this->rx_handle_, &mic_std_cfg);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to init mic RX channel: %s", esp_err_to_name(err));
      i2s_del_channel(this->rx_handle_);
      return false;
    }

    i2s_channel_enable(this->rx_handle_);
    ESP_LOGD(TAG, "Mic initialized: %d-bit, channel=%s, gain=%dx",
             this->mic_bits_per_sample_,
             this->mic_channel_ == MIC_CHANNEL_LEFT ? "left" : "right",
             this->mic_gain_);
  }

  // Speaker on I2S_NUM_1
  if (need_speaker) {
    i2s_chan_config_t spk_chan_cfg = {
      .id = I2S_NUM_1,
      .role = I2S_ROLE_MASTER,
      .dma_desc_num = DMA_BUFFER_COUNT,
      .dma_frame_num = DMA_BUFFER_SIZE,
      .auto_clear = true,
      .intr_priority = 0,
    };

    esp_err_t err = i2s_new_channel(&spk_chan_cfg, &this->tx_handle_, nullptr);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to create speaker I2S channel: %s", esp_err_to_name(err));
      if (this->rx_handle_) i2s_del_channel(this->rx_handle_);
      return false;
    }

    i2s_std_config_t spk_std_cfg = {
      .clk_cfg = {
        .sample_rate_hz = this->sample_rate_,
        .clk_src = I2S_CLK_SRC_DEFAULT,
        .mclk_multiple = I2S_MCLK_MULTIPLE_256,
      },
      .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
      .gpio_cfg = {
        .mclk = GPIO_NUM_NC,
        .bclk = (gpio_num_t)this->speaker_bclk_pin_,
        .ws = (gpio_num_t)this->speaker_lrclk_pin_,
        .dout = (gpio_num_t)this->speaker_dout_pin_,
        .din = GPIO_NUM_NC,
        .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
      },
    };

    spk_std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;

    err = i2s_channel_init_std_mode(this->tx_handle_, &spk_std_cfg);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to init speaker TX channel: %s", esp_err_to_name(err));
      if (this->rx_handle_) i2s_del_channel(this->rx_handle_);
      i2s_del_channel(this->tx_handle_);
      return false;
    }

    i2s_channel_enable(this->tx_handle_);
  }

  ESP_LOGD(TAG, "I2S Dual Bus initialized");
  return true;
}

void I2SAudioUDP::deinit_i2s_() {
  if (this->tx_handle_ != nullptr) {
    i2s_channel_disable(this->tx_handle_);
    i2s_del_channel(this->tx_handle_);
    this->tx_handle_ = nullptr;
  }
  if (this->rx_handle_ != nullptr) {
    i2s_channel_disable(this->rx_handle_);
    i2s_del_channel(this->rx_handle_);
    this->rx_handle_ = nullptr;
  }
  ESP_LOGD(TAG, "I2S deinitialized");
}

bool I2SAudioUDP::init_sockets_() {
  // Evaluate templatable values now
  if (this->remote_ip_func_) {
    this->remote_ip_ = this->remote_ip_func_();
  }
  if (this->remote_port_func_) {
    this->remote_port_ = this->remote_port_func_();
  }
  if (this->listen_port_func_) {
    this->listen_port_ = this->listen_port_func_();
  }

  ESP_LOGD(TAG, "Network config: remote=%s:%d, listen=%d",
           this->remote_ip_.c_str(), this->remote_port_, this->listen_port_);

  if (this->remote_ip_.empty()) {
    ESP_LOGE(TAG, "Remote IP is empty");
    return false;
  }

  // Setup remote address
  memset(&this->remote_addr_, 0, sizeof(this->remote_addr_));
  this->remote_addr_.sin_family = AF_INET;
  this->remote_addr_.sin_port = htons(this->remote_port_);
  if (inet_pton(AF_INET, this->remote_ip_.c_str(), &this->remote_addr_.sin_addr) != 1) {
    ESP_LOGE(TAG, "Invalid remote IP: %s", this->remote_ip_.c_str());
    return false;
  }

  // Create send socket (for TX)
  if (this->audio_mode_ == AUDIO_MODE_TX_ONLY || this->audio_mode_ == AUDIO_MODE_FULL_DUPLEX) {
    this->send_socket_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (this->send_socket_ < 0) {
      ESP_LOGE(TAG, "Failed to create send socket: %d", errno);
      return false;
    }
    int flags = fcntl(this->send_socket_, F_GETFL, 0);
    fcntl(this->send_socket_, F_SETFL, flags | O_NONBLOCK);
  }

  // Create receive socket (for RX)
  if (this->audio_mode_ == AUDIO_MODE_RX_ONLY || this->audio_mode_ == AUDIO_MODE_FULL_DUPLEX) {
    this->recv_socket_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (this->recv_socket_ < 0) {
      ESP_LOGE(TAG, "Failed to create receive socket: %d", errno);
      if (this->send_socket_ >= 0) close(this->send_socket_);
      return false;
    }

    int reuse = 1;
    setsockopt(this->recv_socket_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    setsockopt(this->recv_socket_, SOL_SOCKET, SO_REUSEPORT, &reuse, sizeof(reuse));

    struct sockaddr_in listen_addr;
    memset(&listen_addr, 0, sizeof(listen_addr));
    listen_addr.sin_family = AF_INET;
    listen_addr.sin_addr.s_addr = INADDR_ANY;
    listen_addr.sin_port = htons(this->listen_port_);

    if (bind(this->recv_socket_, (struct sockaddr *)&listen_addr, sizeof(listen_addr)) < 0) {
      ESP_LOGE(TAG, "Failed to bind to port %d: %s", this->listen_port_, strerror(errno));
      if (this->send_socket_ >= 0) close(this->send_socket_);
      close(this->recv_socket_);
      return false;
    }

    int flags = fcntl(this->recv_socket_, F_GETFL, 0);
    fcntl(this->recv_socket_, F_SETFL, flags | O_NONBLOCK);

    int rcvbuf = 32768;
    setsockopt(this->recv_socket_, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));
  }

  ESP_LOGD(TAG, "UDP sockets initialized");
  return true;
}

void I2SAudioUDP::close_sockets_() {
  if (this->send_socket_ >= 0) {
    close(this->send_socket_);
    this->send_socket_ = -1;
  }
  if (this->recv_socket_ >= 0) {
    close(this->recv_socket_);
    this->recv_socket_ = -1;
  }
  vTaskDelay(pdMS_TO_TICKS(50));
  ESP_LOGD(TAG, "UDP sockets closed");
}

void I2SAudioUDP::audio_task(void *params) {
  I2SAudioUDP *self = (I2SAudioUDP *)params;

  int frame_size = 256;
#ifdef USE_ESP_AEC
  if (self->aec_ != nullptr && self->aec_->is_initialized()) {
    frame_size = self->aec_->get_frame_size();
  }
#endif
  if (frame_size <= 0) frame_size = 256;
  size_t frame_bytes = frame_size * sizeof(int16_t);

  ESP_LOGD(TAG, "Audio task started: frame_size=%d samples", frame_size);

  bool is_dual_bus = (self->bus_mode_ == I2S_BUS_DUAL);
  bool has_tx = (self->audio_mode_ == AUDIO_MODE_TX_ONLY || self->audio_mode_ == AUDIO_MODE_FULL_DUPLEX);
  bool has_rx = (self->audio_mode_ == AUDIO_MODE_RX_ONLY || self->audio_mode_ == AUDIO_MODE_FULL_DUPLEX);

  size_t mic_read_bytes = is_dual_bus && (self->mic_bits_per_sample_ == 32) ?
      (frame_size * sizeof(int32_t)) : frame_bytes;

  // Allocate buffers
  int16_t *mic_buffer = has_tx ?
      (int16_t *)heap_caps_aligned_alloc(16, frame_bytes, MALLOC_CAP_INTERNAL) : nullptr;
  int32_t *mic_buffer_32 = (is_dual_bus && has_tx && self->mic_bits_per_sample_ == 32) ?
      (int32_t *)heap_caps_aligned_alloc(16, mic_read_bytes, MALLOC_CAP_INTERNAL) : nullptr;
  int16_t *spk_buffer = has_rx ?
      (int16_t *)heap_caps_aligned_alloc(16, frame_bytes, MALLOC_CAP_INTERNAL) : nullptr;
  int16_t *aec_output = nullptr;
  int16_t *last_speaker = nullptr;

#ifdef USE_ESP_AEC
  // Allocate AEC buffers only if AEC is present - they're used based on aec_enabled_ at runtime
  if (self->aec_ != nullptr && self->aec_->is_initialized() && has_tx && has_rx) {
    aec_output = (int16_t *)heap_caps_aligned_alloc(16, frame_bytes, MALLOC_CAP_INTERNAL);
    last_speaker = (int16_t *)heap_caps_aligned_alloc(16, frame_bytes, MALLOC_CAP_INTERNAL);
    if (last_speaker) memset(last_speaker, 0, frame_bytes);
  }
#endif

  uint8_t udp_buffer[AUDIO_BUFFER_SIZE];
  size_t bytes_read, bytes_written;

  // Create ring buffer
  self->audio_ring_buffer_ = RingBuffer::create(RING_BUFFER_SIZE);
  if (self->audio_ring_buffer_ == nullptr) {
    ESP_LOGE(TAG, "Failed to allocate ring buffer");
    vTaskDelete(NULL);
    return;
  }

  // Must exceed what the I2S DMA ring can absorb in one go
  // (DMA_BUFFER_COUNT * DMA_BUFFER_SIZE frames = 8192 bytes at 16-bit mono).
  // i2s_channel_write does not block until DMA is full, so a prebuffer smaller
  // than that is drained instantly on the very first pass and playback can
  // never reach steady state. 12288 bytes is 384ms at 16kHz mono.
  const size_t PREBUFFER_THRESHOLD = 12288;

  // How long the ring may stay empty before it counts as a real underrun. DMA
  // still holds up to 256ms of already-queued audio and keeps playing, so
  // treating the first empty pass as an underrun is what creates the gap.
  static const uint32_t UNDERRUN_GRACE_MS = 250;

  bool prebuffering = true;
  uint32_t last_frame_ms = millis();
  uint32_t last_stats_log = 0;

  while (self->streaming_) {
    // Set whenever this pass actually moved audio. Every blocking call in this
    // loop (i2s_channel_write / i2s_channel_read) sits behind a data-available
    // check, so a pass that moves nothing never blocks - see the yield at the
    // bottom of the loop.
    bool did_work = false;

    // ═══════════════════════════════════════════════════════════════════════
    // UDP -> JITTER BUFFER (receive audio from network)
    // ═══════════════════════════════════════════════════════════════════════
    if (has_rx && self->recv_socket_ >= 0) {
      for (int i = 0; i < 10; i++) {
        ssize_t received = recvfrom(self->recv_socket_, udp_buffer, AUDIO_BUFFER_SIZE,
                                     0, nullptr, nullptr);
        if (received > 0) {
          self->rx_packets_++;
          self->audio_ring_buffer_->write((void*)udp_buffer, received);
          did_work = true;
        } else if (received < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
          break;
        } else {
          break;
        }
      }
    }

    // ═══════════════════════════════════════════════════════════════════════
    // RING BUFFER -> SPEAKER
    // ═══════════════════════════════════════════════════════════════════════
    if (has_rx && spk_buffer && self->tx_handle_) {
      memset(spk_buffer, 0, frame_bytes);
      size_t available = self->audio_ring_buffer_->available();

      if (prebuffering) {
        if (available >= PREBUFFER_THRESHOLD) {
          prebuffering = false;
          last_frame_ms = millis();
          ESP_LOGD(TAG, "Prebuffer complete, starting playback");
        }
      }

      if (!prebuffering && available >= frame_bytes) {
        size_t got = self->audio_ring_buffer_->read((void*)spk_buffer, frame_bytes, 0);
        if (got == frame_bytes) {
          self->apply_software_volume_(spk_buffer, frame_size);
          i2s_channel_write(self->tx_handle_, spk_buffer, frame_bytes, &bytes_written, pdMS_TO_TICKS(50));
          last_frame_ms = millis();
          did_work = true;

#ifdef USE_ESP_AEC
          if (last_speaker) {
            memcpy(last_speaker, spk_buffer, frame_bytes);
          }
#endif
        }
      } else if (!prebuffering && available == 0) {
        // Momentarily empty is normal - DMA is still playing. Only a sustained
        // drought means the source has actually stopped.
        if (millis() - last_frame_ms > UNDERRUN_GRACE_MS) {
          prebuffering = true;
          ESP_LOGW(TAG, "Buffer underrun, rebuffering...");
        }
      }
    }

    // ═══════════════════════════════════════════════════════════════════════
    // MICROPHONE -> (AEC) -> UDP
    // ═══════════════════════════════════════════════════════════════════════
    if (has_tx && mic_buffer && self->rx_handle_) {
      esp_err_t err;
      bool is_32bit = (self->mic_bits_per_sample_ == 32);
      int32_t mic_gain = self->mic_gain_;

      if (is_dual_bus && is_32bit && mic_buffer_32) {
        err = i2s_channel_read(self->rx_handle_, mic_buffer_32, mic_read_bytes,
                                &bytes_read, pdMS_TO_TICKS(50));
        if (err == ESP_OK && bytes_read == mic_read_bytes) {
          for (int i = 0; i < frame_size; i++) {
            int32_t sample = mic_buffer_32[i] >> 16;
            sample *= mic_gain;
            mic_buffer[i] = (int16_t)std::clamp(sample, (int32_t)-32768, (int32_t)32767);
          }
          bytes_read = frame_bytes;
        }
      } else {
        err = i2s_channel_read(self->rx_handle_, mic_buffer, frame_bytes,
                                &bytes_read, pdMS_TO_TICKS(50));
        if (err == ESP_OK && bytes_read == frame_bytes && mic_gain > 1) {
          for (int i = 0; i < frame_size; i++) {
            int32_t sample = mic_buffer[i] * mic_gain;
            mic_buffer[i] = (int16_t)std::clamp(sample, (int32_t)-32768, (int32_t)32767);
          }
        }
      }

      if (err == ESP_OK && bytes_read == frame_bytes) {
        // A successful I2S read is itself a blocking, sample-paced operation,
        // so the loop is self-throttling whenever a mic is present.
        did_work = true;

        int16_t *send_buffer = mic_buffer;

#ifdef USE_ESP_AEC
        // Apply AEC only if enabled at runtime
        if (self->aec_ != nullptr && self->aec_->is_initialized() &&
            self->aec_enabled_ && aec_output && last_speaker) {
          self->aec_->process(mic_buffer, last_speaker, aec_output, frame_size);
          send_buffer = aec_output;
        }
#endif

        if (self->send_socket_ >= 0) {
          ssize_t sent = sendto(self->send_socket_, send_buffer, frame_bytes, 0,
                                 (struct sockaddr *)&self->remote_addr_, sizeof(self->remote_addr_));
          if (sent > 0) {
            self->tx_packets_++;
          }
        }
      }
    }

    // Periodic stats
    uint32_t now = millis();
    if (now - last_stats_log > 5000) {
      ESP_LOGD(TAG, "Stats: TX=%u RX=%u buf=%d", self->tx_packets_, self->rx_packets_,
               self->audio_ring_buffer_->available());
      last_stats_log = now;
    }

    // Nothing moved this pass, which means nothing above blocked either: the
    // receive socket is O_NONBLOCK, playback is skipped while prebuffering,
    // and in RX_ONLY mode the mic branch does not run at all. Without an
    // explicit sleep this loop busy-waits at TASK_PRIORITY (19) pinned to a
    // core, starving that core's idle task, and the task watchdog panics the
    // device - which is exactly what happens to an RX_ONLY receiver that is
    // enabled before its sender starts.
    //
    // vTaskDelay(1) and not pdMS_TO_TICKS(n): at the default 100 Hz tick,
    // pdMS_TO_TICKS of anything under 10 ms truncates to 0, and vTaskDelay(0)
    // only yields to tasks of equal priority - a priority-0 idle task would
    // still never run. One tick is the smallest delay that actually blocks.
    if (!did_work) {
      vTaskDelay(1);
    }
  }

  // Cleanup
  if (mic_buffer) heap_caps_free(mic_buffer);
  if (mic_buffer_32) heap_caps_free(mic_buffer_32);
  if (spk_buffer) heap_caps_free(spk_buffer);
  if (aec_output) heap_caps_free(aec_output);
  if (last_speaker) heap_caps_free(last_speaker);
  self->audio_ring_buffer_.reset();

  ESP_LOGD(TAG, "Audio task stopped");
  vTaskDelete(NULL);
}

void I2SAudioUDP::start() {
  if (this->streaming_) {
    ESP_LOGW(TAG, "Already streaming");
    return;
  }

  ESP_LOGI(TAG, "Starting audio streaming...");

  // A play_tone() while idle may have left a channel open; start from a known
  // state rather than trying to reuse it.
  if (this->tx_handle_ != nullptr || this->rx_handle_ != nullptr) {
    this->deinit_i2s_();
  }

  // Initialize I2S
  bool i2s_ok = false;
  if (this->bus_mode_ == I2S_BUS_SINGLE) {
    i2s_ok = this->init_i2s_single_bus_();
  } else {
    i2s_ok = this->init_i2s_dual_bus_();
  }

  if (!i2s_ok) {
    this->on_error_trigger_.trigger("I2S initialization failed");
    return;
  }

  // Initialize sockets
  if (!this->init_sockets_()) {
    this->deinit_i2s_();
    this->on_error_trigger_.trigger("Socket initialization failed");
    return;
  }

  this->streaming_ = true;
  this->tx_packets_ = 0;
  this->rx_packets_ = 0;

  // Create audio task
  BaseType_t result = xTaskCreatePinnedToCore(
    audio_task, "i2s_audio_udp", TASK_STACK_SIZE, this, TASK_PRIORITY, &this->audio_task_handle_, 1
  );

  if (result != pdPASS) {
    ESP_LOGE(TAG, "Failed to create audio task");
    this->streaming_ = false;
    this->close_sockets_();
    this->deinit_i2s_();
    this->on_error_trigger_.trigger("Task creation failed");
    return;
  }

  this->on_start_trigger_.trigger();
  ESP_LOGI(TAG, "Streaming started!");
}

void I2SAudioUDP::stop() {
  if (!this->streaming_) {
    ESP_LOGW(TAG, "Not streaming");
    return;
  }

  ESP_LOGD(TAG, "Stopping audio streaming...");

  this->streaming_ = false;

  // Wait for audio task
  if (this->audio_task_handle_ != nullptr) {
    int wait_count = 0;
    while (eTaskGetState(this->audio_task_handle_) != eDeleted && wait_count < 50) {
      vTaskDelay(pdMS_TO_TICKS(10));
      wait_count++;
    }
    this->audio_task_handle_ = nullptr;
  }

  this->close_sockets_();
  this->deinit_i2s_();

  this->on_stop_trigger_.trigger();
  ESP_LOGI(TAG, "Streaming stopped");
}

}  // namespace i2s_audio_udp
}  // namespace esphome
