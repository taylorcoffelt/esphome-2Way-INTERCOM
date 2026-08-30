#pragma once

#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "esphome/core/helpers.h"
#include "esphome/core/ring_buffer.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/number/number.h"
#include "esphome/components/switch/switch.h"

#include <atomic>
#include <string>
#include <driver/i2s_std.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <lwip/sockets.h>
#include <lwip/netdb.h>

// Forward declare AEC
namespace esphome {
namespace esp_aec {
class EspAec;
}
}

namespace esphome {
namespace i2s_audio_udp {

// I2S bus configuration mode (auto-deduced from pins)
enum I2SBusMode : uint8_t {
  I2S_BUS_SINGLE,  // Shared bus for mic and speaker (ES8311)
  I2S_BUS_DUAL,    // Separate buses (INMP441 + MAX98357A)
};

// Audio direction mode (auto-deduced from pins)
enum AudioMode : uint8_t {
  AUDIO_MODE_TX_ONLY,     // Only send (mic only)
  AUDIO_MODE_RX_ONLY,     // Only receive (speaker only)
  AUDIO_MODE_FULL_DUPLEX, // Both (mic + speaker)
};

// Microphone channel
enum MicChannel : uint8_t {
  MIC_CHANNEL_LEFT,
  MIC_CHANNEL_RIGHT,
};

class I2SAudioUDP : public Component {
 public:
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

  // ─────────────────────────────────────────────────────────────────────────
  // Pin Configuration - Single Bus
  // ─────────────────────────────────────────────────────────────────────────
  void set_i2s_lrclk_pin(int pin) { this->i2s_lrclk_pin_ = pin; }
  void set_i2s_bclk_pin(int pin) { this->i2s_bclk_pin_ = pin; }
  void set_i2s_mclk_pin(int pin) { this->i2s_mclk_pin_ = pin; }
  void set_i2s_din_pin(int pin) { this->i2s_din_pin_ = pin; }
  void set_i2s_dout_pin(int pin) { this->i2s_dout_pin_ = pin; }

  // ─────────────────────────────────────────────────────────────────────────
  // Pin Configuration - Dual Bus
  // ─────────────────────────────────────────────────────────────────────────
  void set_mic_lrclk_pin(int pin) { this->mic_lrclk_pin_ = pin; }
  void set_mic_bclk_pin(int pin) { this->mic_bclk_pin_ = pin; }
  void set_mic_din_pin(int pin) { this->mic_din_pin_ = pin; }
  void set_speaker_lrclk_pin(int pin) { this->speaker_lrclk_pin_ = pin; }
  void set_speaker_bclk_pin(int pin) { this->speaker_bclk_pin_ = pin; }
  void set_speaker_dout_pin(int pin) { this->speaker_dout_pin_ = pin; }

  // ─────────────────────────────────────────────────────────────────────────
  // Audio Configuration
  // ─────────────────────────────────────────────────────────────────────────
  void set_sample_rate(uint32_t rate) { this->sample_rate_ = rate; }
  void set_mic_bits_per_sample(int bits) { this->mic_bits_per_sample_ = bits; }
  void set_mic_channel(MicChannel channel) { this->mic_channel_ = channel; }
  void set_mic_gain(int gain) { this->mic_gain_ = gain; }
  void set_speaker_enable_pin(int pin) { this->speaker_enable_pin_ = pin; }

  // ─────────────────────────────────────────────────────────────────────────
  // Network Configuration (templatable)
  // ─────────────────────────────────────────────────────────────────────────
  void set_remote_ip(const std::string &ip) { this->remote_ip_ = ip; }
  void set_remote_port(uint16_t port) { this->remote_port_ = port; }
  void set_listen_port(uint16_t port) { this->listen_port_ = port; }

  // Lambda setters (evaluated at start())
  void set_remote_ip_lambda(std::function<std::string()> func) { this->remote_ip_func_ = func; }
  void set_remote_port_lambda(std::function<uint16_t()> func) { this->remote_port_func_ = func; }
  void set_listen_port_lambda(std::function<uint16_t()> func) { this->listen_port_func_ = func; }

  // ─────────────────────────────────────────────────────────────────────────
  // AEC Integration
  // ─────────────────────────────────────────────────────────────────────────
  void set_aec(esp_aec::EspAec *aec) { this->aec_ = aec; }
  void set_aec_enabled(bool enabled) { this->aec_enabled_ = enabled; }
  bool is_aec_enabled() const { return this->aec_enabled_; }
  bool has_aec() const { return this->aec_ != nullptr; }

  // ─────────────────────────────────────────────────────────────────────────
  // Control Methods (called from automations)
  // ─────────────────────────────────────────────────────────────────────────
  void start();
  void stop();
  bool is_streaming() const { return this->streaming_; }

  // Bring the I2S channel up, or take it back down, without streaming.
  //
  // This exists for amplifier pop suppression. Enabling an amplifier while the
  // codec's output is unclocked and then starting BCLK/WS makes the amp pass
  // the resulting step as a loud turn-on thump. The cure is ordering: open the
  // channel, let the codec settle at digital silence (auto_clear means an idle
  // enabled channel emits zeros), enable the amp, and only then play. On the
  // way out, mute the amp before the clocks stop.
  //
  // play_tone() cannot do this itself - it owns init and teardown around a
  // blocking call, leaving no point in the middle for an automation to act -
  // so the two halves are exposed separately here.
  //
  // No-op while streaming: the audio task owns the channel then, and start()
  // already fires its on_start trigger after I2S is up, so that path is
  // correctly ordered without help.
  bool hold_i2s_open(bool open) {
    if (this->streaming_)
      return true;
    if (open) {
      if (this->tx_handle_ != nullptr)
        return true;
      const bool ok = (this->bus_mode_ == I2S_BUS_SINGLE) ? this->init_i2s_single_bus_()
                                                          : this->init_i2s_dual_bus_();
      this->i2s_held_open_ = ok;
      return ok;
    }
    if (this->i2s_held_open_) {
      this->deinit_i2s_();
      this->i2s_held_open_ = false;
    }
    return true;
  }

  // Play a sine tone through the speaker path.
  //
  // Exists to answer "is the codec, amplifier and speaker chain alive?" without
  // involving the network at all - otherwise silence has two possible causes
  // and no way to tell them apart.
  //
  // While streaming, the tone goes to the audio task through the same ring
  // buffer the network feeds. While idle it uses the I2S channel directly,
  // reusing one left open by hold_i2s_open() if there is one, and otherwise
  // opening and closing its own; in that case the call blocks for roughly
  // duration_ms.
  //
  // amplitude is 0..1 of full scale and is deliberately independent of
  // set_volume(): a test that is silent because the volume slider happens to be
  // down has tested nothing. The codec's own output level still applies. The
  // tone is ramped in and out over 5ms so the amplifier does not pop.
  //
  // decay_ms shapes the sustain. Left at 0 the note holds flat for its whole
  // duration, which is what a test tone wants and what every existing caller
  // gets. Set non-zero it applies exp(-t / tau) from onset, with tau picked so
  // the amplitude is down to about 5% of peak at decay_ms - a struck-and-ringing
  // shape rather than a held organ note. A decay_ms at or beyond duration_ms
  // just means the note is still audibly decaying when it ends. The 5ms attack
  // ramp applies either way; without it the onset clicks.
  void play_tone(uint32_t freq_hz, uint32_t duration_ms, float amplitude = 0.25f,
                 uint32_t decay_ms = 0);

  // ─────────────────────────────────────────────────────────────────────────
  // Volume Control
  // ─────────────────────────────────────────────────────────────────────────
  void set_volume(float volume);
  float get_volume() const { return this->volume_; }

  // ─────────────────────────────────────────────────────────────────────────
  // Statistics (for sensors)
  // ─────────────────────────────────────────────────────────────────────────
  uint32_t get_tx_packets() const { return this->tx_packets_; }
  uint32_t get_rx_packets() const { return this->rx_packets_; }

  // Recent peak amplitude of what the speaker path is actually playing, 0..1.
  // The audio task refreshes it per frame with a decay, so a waveform drawn
  // from it falls back toward zero through silence instead of freezing at the
  // last loud frame. Reads flat while stopped - nothing is updating it then.
  float get_peak_level() { return this->streaming_ ? this->peak_level_.load() : 0.0f; }

  // Number of entries in the recent-levels ring. One entry is pushed per audio
  // frame processed by the playback path, so the window this covers is
  // LEVEL_HISTORY_SIZE * (frame_size / sample_rate) - on the order of a second
  // or two at 16kHz. Power of two on purpose; see copy_levels().
  static constexpr size_t LEVEL_HISTORY_SIZE = 64;

  // Copies up to `max` of the most recent levels, OLDEST FIRST.
  // Returns the number actually written.
  //
  // Why this exists alongside get_peak_level(): the ESPHome main loop can be
  // blocked for hundreds of milliseconds at a time by unrelated synchronous
  // work, and anything sampling get_peak_level() on an interval simply loses
  // every frame that went by while it was stuck - the waveform then advances in
  // bursts. The audio task is never blocked, so it records the history here and
  // the UI reads the whole window whenever it next manages to run.
  //
  // Oldest-first is part of the contract: the caller draws left to right and
  // must not have to know where the write head currently is.
  //
  // Locking, deliberately absent: the audio FreeRTOS task writes and the main
  // loop reads, and the only synchronisation is the atomic write index. The
  // sample array is plain aligned floats because a 4-byte aligned load or store
  // cannot tear on Xtensa, so the worst a racing reader can see is a sample one
  // frame staler than it expected - invisible in a waveform. Taking a mutex in
  // the audio path would risk stalling playback to protect a drawing, which is
  // the wrong trade. Please do not "fix" this into a lock.
  size_t copy_levels(float *out, size_t max) {
    if (out == nullptr || max == 0)
      return 0;
    // Total pushes ever, not a wrapped index - the modulo below turns it into
    // one. LEVEL_HISTORY_SIZE divides 2^32, so this stays correct across the
    // eventual uint32_t rollover.
    const uint32_t total = this->level_write_index_.load(std::memory_order_acquire);
    size_t have = (total < (uint32_t) LEVEL_HISTORY_SIZE) ? (size_t) total : LEVEL_HISTORY_SIZE;
    if (have > max)
      have = max;
    const uint32_t start = total - (uint32_t) have;
    for (size_t i = 0; i < have; i++)
      out[i] = this->level_history_[(size_t) ((start + (uint32_t) i) % LEVEL_HISTORY_SIZE)];
    return have;
  }

  const char* get_audio_mode_text() const;
  I2SBusMode get_bus_mode() const { return this->bus_mode_; }
  AudioMode get_audio_mode() const { return this->audio_mode_; }

  // ─────────────────────────────────────────────────────────────────────────
  // Triggers
  // ─────────────────────────────────────────────────────────────────────────
  Trigger<> *get_on_start_trigger() { return &this->on_start_trigger_; }
  Trigger<> *get_on_stop_trigger() { return &this->on_stop_trigger_; }
  Trigger<std::string> *get_on_error_trigger() { return &this->on_error_trigger_; }

 protected:
  void deduce_modes_();
  bool init_i2s_single_bus_();
  bool init_i2s_dual_bus_();
  void deinit_i2s_();
  bool init_sockets_();
  void close_sockets_();
  void apply_software_volume_(int16_t *buffer, size_t samples);

  // Append one level to the history ring. Audio task only. The sample is
  // written before the index is published with release ordering, which is what
  // makes the acquire load in copy_levels() safe to pair with.
  void push_level_(float level) {
    const uint32_t idx = this->level_write_index_.load(std::memory_order_relaxed);
    this->level_history_[(size_t) (idx % LEVEL_HISTORY_SIZE)] = level;
    this->level_write_index_.store(idx + 1, std::memory_order_release);
  }

  // Flatten the history. Called wherever peak_level_ is zeroed, so a stopped
  // stream draws as a flat line instead of whatever was playing when it went
  // away. The write index is intentionally left alone: zeroed samples already
  // read as silence, and resetting it would make copy_levels() return a
  // shrinking window instead.
  void clear_levels_() {
    for (size_t i = 0; i < LEVEL_HISTORY_SIZE; i++)
      this->level_history_[i] = 0.0f;
  }

  // True when the speaker channel carries two slots per frame, so a mono
  // source has to be duplicated across both before it is written. The
  // single-bus (ES8311) path needs this: with no MCLK the codec derives its
  // clocks from BCLK and only accepts standard BCLK/rate ratios, and one
  // 16-bit slot per frame is not one of them.
  bool speaker_is_stereo_() const;

  static void audio_task(void *params);

  // Pin configuration - Single bus
  int i2s_lrclk_pin_{-1};
  int i2s_bclk_pin_{-1};
  int i2s_mclk_pin_{-1};
  int i2s_din_pin_{-1};
  int i2s_dout_pin_{-1};

  // Pin configuration - Dual bus
  int mic_lrclk_pin_{-1};
  int mic_bclk_pin_{-1};
  int mic_din_pin_{-1};
  int speaker_lrclk_pin_{-1};
  int speaker_bclk_pin_{-1};
  int speaker_dout_pin_{-1};

  // Audio config
  uint32_t sample_rate_{16000};
  int mic_bits_per_sample_{16};
  MicChannel mic_channel_{MIC_CHANNEL_LEFT};
  int mic_gain_{1};
  int speaker_enable_pin_{-1};
  float volume_{1.0f};

  // Network - runtime values (evaluated from templates)
  std::string remote_ip_;
  uint16_t remote_port_{0};
  uint16_t listen_port_{0};

  // Network - template functions
  std::function<std::string()> remote_ip_func_;
  std::function<uint16_t()> remote_port_func_;
  std::function<uint16_t()> listen_port_func_;

  // AEC
  esp_aec::EspAec *aec_{nullptr};
  bool aec_enabled_{true};  // Default enabled when AEC component is linked

  // Deduced modes
  I2SBusMode bus_mode_{I2S_BUS_SINGLE};
  AudioMode audio_mode_{AUDIO_MODE_FULL_DUPLEX};

  // Runtime state
  volatile bool streaming_{false};
  bool i2s_held_open_{false};
  i2s_chan_handle_t tx_handle_{nullptr};
  i2s_chan_handle_t rx_handle_{nullptr};
  std::unique_ptr<esphome::ring_buffer::RingBuffer> audio_ring_buffer_;
  int send_socket_{-1};
  int recv_socket_{-1};
  struct sockaddr_in remote_addr_;
  TaskHandle_t audio_task_handle_{nullptr};

  // Given exactly once by audio_task on every one of its exit paths, taken by
  // stop() to join it. A self-deleted task's handle is reaped by the idle task
  // almost immediately, so polling eTaskGetState() on it reads freed memory;
  // this is the only safe way to know the task is actually gone before the
  // state it was using is torn down.
  SemaphoreHandle_t task_done_{nullptr};

  // Statistics
  volatile uint32_t tx_packets_{0};
  volatile uint32_t rx_packets_{0};

  // Playback level, 0..1. Written by the audio task, read from the main loop.
  std::atomic<float> peak_level_{0.0f};

  // Recent history of that same level, one entry per processed frame. Plain
  // floats guarded only by the atomic index below - see copy_levels() for why
  // that is enough and why a lock would be worse. Unrelated to
  // audio_ring_buffer_ above, which carries PCM.
  float level_history_[LEVEL_HISTORY_SIZE]{};
  std::atomic<uint32_t> level_write_index_{0};

  // Triggers
  Trigger<> on_start_trigger_;
  Trigger<> on_stop_trigger_;
  Trigger<std::string> on_error_trigger_;
};

// Actions
template<typename... Ts>
class StartAction : public Action<Ts...>, public Parented<I2SAudioUDP> {
 public:
  void play(Ts... x) override { this->parent_->start(); }
};

template<typename... Ts>
class StopAction : public Action<Ts...>, public Parented<I2SAudioUDP> {
 public:
  void play(Ts... x) override { this->parent_->stop(); }
};

// Sensors
class I2SAudioUDPSensor : public sensor::Sensor, public PollingComponent {
 public:
  enum SensorType { TX_PACKETS, RX_PACKETS };

  void set_parent(I2SAudioUDP *parent) { this->parent_ = parent; }
  void set_sensor_type(SensorType type) { this->type_ = type; }
  void update() override {
    if (this->parent_ == nullptr) return;
    if (this->type_ == TX_PACKETS) {
      this->publish_state(this->parent_->get_tx_packets());
    } else {
      this->publish_state(this->parent_->get_rx_packets());
    }
  }

 protected:
  I2SAudioUDP *parent_{nullptr};
  SensorType type_{TX_PACKETS};
};

class I2SAudioUDPTextSensor : public text_sensor::TextSensor, public PollingComponent {
 public:
  void set_parent(I2SAudioUDP *parent) { this->parent_ = parent; }
  void update() override {
    if (this->parent_ != nullptr) {
      this->publish_state(this->parent_->get_audio_mode_text());
    }
  }

 protected:
  I2SAudioUDP *parent_{nullptr};
};

// Volume Number
class I2SAudioUDPVolume : public number::Number, public Component {
 public:
  void set_parent(I2SAudioUDP *parent) { this->parent_ = parent; }
  void setup() override { this->publish_state(this->parent_->get_volume() * 100); }

 protected:
  void control(float value) override {
    this->publish_state(value);
    if (this->parent_ != nullptr) {
      this->parent_->set_volume(value / 100.0f);
    }
  }

  I2SAudioUDP *parent_{nullptr};
};

// AEC Switch
class I2SAudioUDPAecSwitch : public switch_::Switch, public Component {
 public:
  void set_parent(I2SAudioUDP *parent) { this->parent_ = parent; }
  void setup() override {
    // Publish initial state (enabled by default if AEC is configured)
    if (this->parent_ != nullptr && this->parent_->has_aec()) {
      this->publish_state(this->parent_->is_aec_enabled());
    }
  }

 protected:
  void write_state(bool state) override {
    if (this->parent_ != nullptr) {
      this->parent_->set_aec_enabled(state);
      this->publish_state(state);
    }
  }

  I2SAudioUDP *parent_{nullptr};
};

}  // namespace i2s_audio_udp
}  // namespace esphome
