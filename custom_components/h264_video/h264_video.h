#pragma once

#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "esphome/core/helpers.h"

#include <atomic>
#include <cstdint>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// The decoder handle type is opaque but the struct tags live here, so the
// header is needed for the member declarations below rather than only in the
// translation unit that calls into it.
#include "esp_h264_dec_sw.h"

#ifdef USE_LVGL
// Included for lv_img_dsc_t and the colour-format constants only. Nothing in
// this component ever *calls* an lv_* function - see the note on the decode
// task in h264_video.cpp - but the frame handed to a YAML lambda has to be a
// descriptor LVGL already understands, or every consumer would have to build
// one by hand on the main loop.
#include <lvgl.h>
#endif

namespace esphome {
namespace h264_video {

// Geometry is configuration, not a constant, because two very different panels
// share this component: a 240x135 ST7789 fed a 240x144 stream, and an 800x480
// RGB panel fed a 480x288 one.
//
// Three separate rectangles have to be tracked and they are all different sizes:
//
//   source        what the decoder hands back. Macroblock-aligned, because the
//                 decoder applies no cropping window - 240x144 or 480x288.
//   visible       how many of those rows are real picture rather than encoder
//                 padding. 135 of the 240x144; all 288 of the 480x288.
//   output        the RGB565 buffer LVGL draws from. Equal to source x visible
//                 on the small panel, 800x480 on the large one, in which case
//                 the colour conversion scales on the way through.
//
// The defaults below reproduce the original hard-wired 240x144 -> 240x135
// behaviour exactly, so a config that sets none of these is unchanged.
static const uint16_t DEFAULT_SOURCE_WIDTH = 240;
static const uint16_t DEFAULT_SOURCE_HEIGHT = 144;
static const uint16_t DEFAULT_VISIBLE_HEIGHT = 135;
static const uint16_t DEFAULT_OUTPUT_WIDTH = 240;
static const uint16_t DEFAULT_OUTPUT_HEIGHT = 135;

class H264Video : public Component {
 public:
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

  void set_port(uint16_t port) { this->port_ = port; }
  void set_frame_timeout(uint32_t timeout_ms) { this->frame_timeout_ms_ = timeout_ms; }

  // Geometry. All five are validated in __init__.py - source_width and
  // source_height are multiples of 16 there because a source that is not
  // macroblock-aligned does not fail here, it silently produces sheared rows.
  void set_source_size(uint16_t w, uint16_t h) {
    this->source_width_ = w;
    this->source_height_ = h;
  }
  void set_visible_height(uint16_t h) { this->visible_height_ = h; }
  void set_output_size(uint16_t w, uint16_t h) {
    this->output_width_ = w;
    this->output_height_ = h;
  }

  uint16_t get_output_width() const { return this->output_width_; }
  uint16_t get_output_height() const { return this->output_height_; }

  // True when the converter has to resample rather than copy - i.e. whenever
  // the output rectangle differs from the visible part of the source.
  bool is_scaling() const {
    return this->output_width_ != this->source_width_ || this->output_height_ != this->visible_height_;
  }

  // ─────────────────────────────────────────────────────────────────────────
  // Lifecycle (main loop only)
  // ─────────────────────────────────────────────────────────────────────────
  void start();
  void stop();
  bool is_running() const { return this->running_.load(std::memory_order_acquire); }

  // ─────────────────────────────────────────────────────────────────────────
  // Frame handoff - main loop only, see the threading note in the .cpp
  // ─────────────────────────────────────────────────────────────────────────
#ifdef USE_LVGL
  // The currently published frame, or nullptr before the first decode. The
  // pointer alternates between two descriptors as buffers are swapped, which is
  // deliberate: LVGL keys its image cache off the source pointer, so an
  // alternating src is never mistaken for an unchanged one.
  const lv_img_dsc_t *get_frame();
#endif

  // Raw access for anything that is not LVGL. Same publication rules apply.
  const uint16_t *get_frame_buffer();

  // True exactly once per newly published frame. A caller that redraws only
  // when this returns true will not repaint an unchanged frame, and cannot miss
  // one either - the flag is cleared by the read, not by the writer.
  bool take_new_frame() { return this->new_frame_.exchange(false, std::memory_order_acq_rel); }

  // True when nothing has arrived within frame_timeout. A stopped component is
  // stale by definition - there is no live stream behind the last frame.
  bool is_stale() const;

  // ─────────────────────────────────────────────────────────────────────────
  // Diagnostics
  // ─────────────────────────────────────────────────────────────────────────
  uint32_t get_nals_received() const { return this->nals_received_.load(std::memory_order_relaxed); }
  uint32_t get_nals_dropped() const { return this->nals_dropped_.load(std::memory_order_relaxed); }
  uint32_t get_frames_decoded() const { return this->frames_decoded_.load(std::memory_order_relaxed); }
  uint32_t get_decode_errors() const { return this->decode_errors_.load(std::memory_order_relaxed); }
  uint32_t get_last_decode_ms() const { return this->last_decode_ms_.load(std::memory_order_relaxed); }

 protected:
  static void decode_task(void *param);
  void decode_task_body_();

  bool open_socket_();
  void close_socket_();
  bool alloc_buffers_();
  void free_buffers_();

  // Nearest-neighbour source-index tables, one entry per output column and per
  // output row. Built once when the buffers are, so the inner loop does a table
  // lookup instead of a multiply and a divide per pixel. Both stay nullptr when
  // no scaling is needed, and that null is what selects the fast copy path in
  // the converter.
  bool build_scale_maps_();
  void free_scale_maps_();
  bool open_decoder_();
  void close_decoder_();

  // Feeds one reassembled Annex-B NAL (start code already prepended) through
  // the decoder's consume loop and publishes any picture that falls out.
  void decode_nal_(uint32_t nal_len_with_start_code);

  // Handles one datagram: header validation, reassembly, and - on a complete
  // NAL - the decode. Returns the length including start code, or 0.
  uint32_t ingest_datagram_(const uint8_t *data, uint32_t len);

  void reset_reassembly_();
  void publish_frame_();

  uint16_t port_{12347};
  uint32_t frame_timeout_ms_{3000};

  uint16_t source_width_{DEFAULT_SOURCE_WIDTH};
  uint16_t source_height_{DEFAULT_SOURCE_HEIGHT};
  uint16_t visible_height_{DEFAULT_VISIBLE_HEIGHT};
  uint16_t output_width_{DEFAULT_OUTPUT_WIDTH};
  uint16_t output_height_{DEFAULT_OUTPUT_HEIGHT};

  // Size of one RGB565 framebuffer. Not a compile-time constant any more, so
  // every allocation, memset and descriptor field goes through this.
  uint32_t fb_bytes_() const { return (uint32_t) this->output_width_ * this->output_height_ * 2; }

  int socket_{-1};
  TaskHandle_t task_handle_{nullptr};

  // Created before the task exists and destroyed only by stop(), so there is
  // never a window where a running task has nothing to signal its exit on.
  SemaphoreHandle_t task_done_{nullptr};

  std::atomic<bool> running_{false};

  esp_h264_dec_handle_t decoder_{nullptr};
  esp_h264_dec_param_sw_handle_t decoder_param_{nullptr};
  bool decoder_opened_{false};
  bool logged_first_decode_{false};

  // Reassembly state. Exactly one NAL is ever in flight - see the .cpp for why
  // a fuller reorder buffer would buy nothing on this link.
  uint8_t *nal_buf_{nullptr};
  uint32_t nal_len_{0};
  uint16_t nal_seq_{0};
  uint8_t next_frag_{0};
  uint8_t frag_count_{0};
  bool nal_in_progress_{false};

  // False until an SPS has been seen on this run. The receiver joins a stream
  // that is already in flight, so the first NALs it gets belong to a GOP whose
  // parameter sets and IDR are long gone - see decode_nal_() for why feeding
  // those to the decoder produces a picture rather than an error.
  bool synced_{false};

  uint8_t *rx_buf_{nullptr};

  // Double-buffered output. The task only ever writes fb_[back_index_]; LVGL
  // only ever reads fb_[published_index_].
  uint16_t *fb_[2]{nullptr, nullptr};

  // Internal RAM, not PSRAM: these are read once per output pixel, which is the
  // one access pattern in the converter that is not a long forward sweep.
  uint16_t *x_map_{nullptr};
  uint16_t *y_map_{nullptr};
#ifdef USE_LVGL
  lv_img_dsc_t img_dsc_[2]{};
#endif
  uint8_t back_index_{0};
  std::atomic<int8_t> published_index_{-1};
  std::atomic<bool> new_frame_{false};

  std::atomic<uint32_t> last_frame_ms_{0};
  uint32_t started_ms_{0};

  std::atomic<uint32_t> nals_received_{0};
  std::atomic<uint32_t> nals_dropped_{0};
  std::atomic<uint32_t> frames_decoded_{0};
  std::atomic<uint32_t> decode_errors_{0};
  std::atomic<uint32_t> last_decode_ms_{0};
};

template<typename... Ts> class StartAction : public Action<Ts...>, public Parented<H264Video> {
 public:
  void play(Ts... x) override { this->parent_->start(); }
};

template<typename... Ts> class StopAction : public Action<Ts...>, public Parented<H264Video> {
 public:
  void play(Ts... x) override { this->parent_->stop(); }
};

}  // namespace h264_video
}  // namespace esphome
