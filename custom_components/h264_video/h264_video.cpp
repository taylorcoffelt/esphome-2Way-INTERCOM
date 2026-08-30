#include "h264_video.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

#include <cstring>
#include <errno.h>

#include <unistd.h>
#include <lwip/sockets.h>
#include <esp_heap_caps.h>

namespace esphome {
namespace h264_video {

static const char *TAG = "h264_video";

// Core 1 alongside the audio task, priority 5 against its 19. The audio task
// has hard I2S deadlines - a missed refill is an audible dropout - while a late
// video frame is invisible on a 10fps stream. Anything close to 19 here would
// let a 30ms decode push audio past its DMA refill window.
static const UBaseType_t TASK_PRIORITY = 5;
static const int TASK_CORE = 1;

// 8KB. Every buffer that matters - NAL reassembly, the two framebuffers, the
// decoder's own ~1MB of working set - is on the heap, so the task itself only
// needs room for the socket call and the converter's locals.
static const uint32_t TASK_STACK_SIZE = 8192;

// The decoder emits macroblock-aligned frames and applies no cropping, so a
// 240x135 stream would come back as 240x144 with nine rows of encoder padding
// that the display must not show. The sender encodes 240x144 outright for that
// reason; this is the height we expect back, but nothing below assumes it - the
// real dimensions are read from the decoder after every picture.
static const uint32_t EXPECTED_DECODE_HEIGHT = 144;

static const uint32_t FB_PIXELS = (uint32_t) FRAME_WIDTH * FRAME_HEIGHT;
static const uint32_t FB_BYTES = FB_PIXELS * 2;

// Wire header: 'H','2', uint16 LE sequence, uint8 fragment index, uint8
// fragment count, two reserved bytes.
static const uint32_t WIRE_HEADER_SIZE = 8;
static const uint32_t MAX_DATAGRAM = 1400;

// 255 fragments is what the one-byte count allows, but a 240x144 constrained
// baseline IDR is a few KB and a P-frame far less. 32KB is roughly an order of
// magnitude of headroom over anything this stream can produce while still being
// a rounding error in PSRAM, and it bounds the damage a lying fragment count
// can do.
static const uint32_t MAX_NAL_SIZE = 32768;

// Prepended to every reassembled NAL. The decoder is fed Annex-B and will not
// find a NAL without one - the wire format strips start codes to save four
// bytes per datagram, so they have to come back before the decoder sees them.
static const uint8_t START_CODE[4] = {0x00, 0x00, 0x00, 0x01};

// How long the socket blocks before the loop rechecks running_. Short enough
// that stop() joins promptly, long enough that a silent stream does not spin.
static const int RECV_TIMEOUT_MS = 100;

// ESPHome's LVGL build byte-swaps RGB565 when the panel wants big-endian
// halfwords. Doing it in the packer costs nothing (the value is already in a
// register) whereas a second pass over 64800 bytes of PSRAM would not be.
#if defined(USE_LVGL) && defined(LV_COLOR_16_SWAP) && LV_COLOR_16_SWAP
#define H264_VIDEO_RGB565_SWAP 1
#else
#define H264_VIDEO_RGB565_SWAP 0
#endif

static inline uint8_t clamp_u8(int32_t v) {
  if (v < 0)
    return 0;
  if (v > 255)
    return 255;
  return (uint8_t) v;
}

static inline uint16_t pack_rgb565(int32_t r, int32_t g, int32_t b) {
  uint16_t c = (uint16_t) (((clamp_u8(r) & 0xF8) << 8) | ((clamp_u8(g) & 0xFC) << 3) | (clamp_u8(b) >> 3));
#if H264_VIDEO_RGB565_SWAP
  c = (uint16_t) ((c >> 8) | (c << 8));
#endif
  return c;
}

// I420 -> RGB565.
//
// esp_h264 ships no such converter - its only colour helper is YUYV->I420 for
// the encoder path - so this is ours.
//
// Two properties drive the shape of the loop. First, the source planes live in
// PSRAM, where a cache miss costs far more than the arithmetic, so every plane
// is walked strictly forwards and each chroma pair is loaded once and reused
// for the 2x2 quad it covers rather than being re-read per pixel. Second, the
// BT.601 limited-range transform is done in 8.8 fixed point: the chroma terms
// are constant across the quad and hoisted out, leaving one multiply-add and a
// shift per component per pixel and no division anywhere in the inner loop.
static void i420_to_rgb565(const uint8_t *src, uint32_t src_w, uint32_t src_h, uint16_t *dst, uint32_t dst_w,
                           uint32_t dst_h) {
  const uint32_t w = (src_w < dst_w) ? src_w : dst_w;
  const uint32_t h = (src_h < dst_h) ? src_h : dst_h;
  if (w == 0 || h == 0)
    return;

  const uint32_t y_stride = src_w;
  const uint32_t c_stride = src_w >> 1;
  const uint8_t *plane_y = src;
  const uint8_t *plane_u = plane_y + (uint32_t) src_w * src_h;
  const uint8_t *plane_v = plane_u + ((uint32_t) src_w * src_h >> 2);

  for (uint32_t y = 0; y < h; y += 2) {
    const uint8_t *row_y0 = plane_y + (uint32_t) y * y_stride;
    // The visible height is 135 - odd - so the last pass has no second row.
    // Guarding here rather than trimming to an even height keeps that final
    // line of picture instead of leaving it as whatever the buffer held.
    const bool has_row1 = (y + 1) < h;
    const uint8_t *row_y1 = has_row1 ? (row_y0 + y_stride) : row_y0;
    const uint8_t *row_u = plane_u + (uint32_t) (y >> 1) * c_stride;
    const uint8_t *row_v = plane_v + (uint32_t) (y >> 1) * c_stride;

    uint16_t *out0 = dst + (uint32_t) y * dst_w;
    uint16_t *out1 = has_row1 ? (out0 + dst_w) : out0;

    for (uint32_t x = 0; x < w; x += 2) {
      const int32_t d = (int32_t) row_u[x >> 1] - 128;
      const int32_t e = (int32_t) row_v[x >> 1] - 128;

      // Chroma contributions, constant for all four pixels of the quad. The
      // +128 is the rounding term for the >> 8 that follows.
      const int32_t r_off = 409 * e + 128;
      const int32_t g_off = -100 * d - 208 * e + 128;
      const int32_t b_off = 516 * d + 128;

      int32_t c = 298 * ((int32_t) row_y0[x] - 16);
      out0[x] = pack_rgb565((c + r_off) >> 8, (c + g_off) >> 8, (c + b_off) >> 8);
      if (x + 1 < w) {
        c = 298 * ((int32_t) row_y0[x + 1] - 16);
        out0[x + 1] = pack_rgb565((c + r_off) >> 8, (c + g_off) >> 8, (c + b_off) >> 8);
      }

      if (has_row1) {
        c = 298 * ((int32_t) row_y1[x] - 16);
        out1[x] = pack_rgb565((c + r_off) >> 8, (c + g_off) >> 8, (c + b_off) >> 8);
        if (x + 1 < w) {
          c = 298 * ((int32_t) row_y1[x + 1] - 16);
          out1[x + 1] = pack_rgb565((c + r_off) >> 8, (c + g_off) >> 8, (c + b_off) >> 8);
        }
      }
    }
  }
}

void H264Video::setup() {
  // Nothing is allocated here on purpose. The decoder wants about a megabyte of
  // PSRAM and holds it for as long as it exists, which is a lot to keep tied up
  // for a device that may never open the video screen. start() pays for it,
  // stop() gives it back.
  ESP_LOGCONFIG(TAG, "H264 video component ready (idle)");
}

void H264Video::dump_config() {
  ESP_LOGCONFIG(TAG, "H264 Video:");
  ESP_LOGCONFIG(TAG, "  Listen Port: %u", (unsigned) this->port_);
  ESP_LOGCONFIG(TAG, "  Display: %ux%u RGB565 (decoded %ux%u, cropped)", (unsigned) FRAME_WIDTH,
                (unsigned) FRAME_HEIGHT, (unsigned) FRAME_WIDTH, (unsigned) EXPECTED_DECODE_HEIGHT);
  ESP_LOGCONFIG(TAG, "  Frame Timeout: %u ms", (unsigned) this->frame_timeout_ms_);
  ESP_LOGCONFIG(TAG, "  Task: core %d, priority %u, %u byte stack", TASK_CORE, (unsigned) TASK_PRIORITY,
                (unsigned) TASK_STACK_SIZE);
#if H264_VIDEO_RGB565_SWAP
  ESP_LOGCONFIG(TAG, "  Pixel Order: RGB565 byte-swapped (LV_COLOR_16_SWAP)");
#else
  ESP_LOGCONFIG(TAG, "  Pixel Order: RGB565 native");
#endif
}

// ───────────────────────────────────────────────────────────────────────────
// Resources
// ───────────────────────────────────────────────────────────────────────────

bool H264Video::open_socket_() {
  this->socket_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (this->socket_ < 0) {
    ESP_LOGE(TAG, "Failed to create socket: %d", errno);
    return false;
  }

  int reuse = 1;
  setsockopt(this->socket_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

  struct sockaddr_in addr;
  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = INADDR_ANY;
  addr.sin_port = htons(this->port_);
  if (bind(this->socket_, (struct sockaddr *) &addr, sizeof(addr)) < 0) {
    ESP_LOGE(TAG, "Failed to bind port %u: %s", (unsigned) this->port_, strerror(errno));
    close(this->socket_);
    this->socket_ = -1;
    return false;
  }

  // A blocking recv with a timeout, rather than non-blocking plus a delay: the
  // task then sleeps whenever the link is quiet instead of burning core 1 at
  // priority 5, and still wakes often enough to notice a stop().
  struct timeval tv;
  tv.tv_sec = RECV_TIMEOUT_MS / 1000;
  tv.tv_usec = (RECV_TIMEOUT_MS % 1000) * 1000;
  setsockopt(this->socket_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  // A burst of fragments belonging to one IDR must not be lost to a full
  // receive queue while the previous frame is still decoding - a dropped
  // fragment costs the whole NAL, and a dropped IDR costs every frame until the
  // next one.
  int rcvbuf = 32768;
  setsockopt(this->socket_, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

  ESP_LOGD(TAG, "Listening for H.264 on UDP port %u", (unsigned) this->port_);
  return true;
}

void H264Video::close_socket_() {
  if (this->socket_ >= 0) {
    close(this->socket_);
    this->socket_ = -1;
  }
}

bool H264Video::alloc_buffers_() {
  // PSRAM for everything. 64800 bytes per framebuffer would be a painful bite
  // out of internal RAM next to WiFi and the audio path, and neither buffer is
  // touched from an ISR.
  for (int i = 0; i < 2; i++) {
    this->fb_[i] = (uint16_t *) heap_caps_malloc(FB_BYTES, MALLOC_CAP_SPIRAM);
    if (this->fb_[i] == nullptr) {
      ESP_LOGE(TAG, "Failed to allocate %u byte framebuffer %d in PSRAM", (unsigned) FB_BYTES, i);
      return false;
    }
    // Black rather than whatever the allocator handed back: a consumer that
    // draws before the first decode should see a blank panel, not PSRAM noise.
    memset(this->fb_[i], 0, FB_BYTES);
  }

  this->nal_buf_ = (uint8_t *) heap_caps_malloc(MAX_NAL_SIZE + sizeof(START_CODE), MALLOC_CAP_SPIRAM);
  if (this->nal_buf_ == nullptr) {
    ESP_LOGE(TAG, "Failed to allocate NAL reassembly buffer");
    return false;
  }
  memcpy(this->nal_buf_, START_CODE, sizeof(START_CODE));

  this->rx_buf_ = (uint8_t *) heap_caps_malloc(MAX_DATAGRAM, MALLOC_CAP_INTERNAL);
  if (this->rx_buf_ == nullptr) {
    ESP_LOGE(TAG, "Failed to allocate receive buffer");
    return false;
  }

#ifdef USE_LVGL
  for (int i = 0; i < 2; i++) {
    lv_img_dsc_t &d = this->img_dsc_[i];
    memset(&d, 0, sizeof(d));
#if LVGL_VERSION_MAJOR >= 9
    d.header.magic = LV_IMAGE_HEADER_MAGIC;
    d.header.cf = LV_COLOR_FORMAT_RGB565;
    d.header.stride = FRAME_WIDTH * 2;
#else
    d.header.always_zero = 0;
    // TRUE_COLOR means "already in lv_color_t layout", which on this build is
    // RGB565 - so the converter's output is drawn straight out of PSRAM with no
    // per-frame decode step in LVGL.
    d.header.cf = LV_IMG_CF_TRUE_COLOR;
#endif
    d.header.w = FRAME_WIDTH;
    d.header.h = FRAME_HEIGHT;
    d.data_size = FB_BYTES;
    d.data = (const uint8_t *) this->fb_[i];
  }
#endif

  return true;
}

void H264Video::free_buffers_() {
  for (int i = 0; i < 2; i++) {
    if (this->fb_[i] != nullptr) {
      heap_caps_free(this->fb_[i]);
      this->fb_[i] = nullptr;
    }
  }
  if (this->nal_buf_ != nullptr) {
    heap_caps_free(this->nal_buf_);
    this->nal_buf_ = nullptr;
  }
  if (this->rx_buf_ != nullptr) {
    heap_caps_free(this->rx_buf_);
    this->rx_buf_ = nullptr;
  }
#ifdef USE_LVGL
  // The descriptors point at buffers that no longer exist. get_frame() is
  // already gated on published_index_, but zeroing these means a stale
  // descriptor cannot be dereferenced into freed PSRAM by anything else.
  memset(this->img_dsc_, 0, sizeof(this->img_dsc_));
#endif
}

bool H264Video::open_decoder_() {
  // I420 is not a preference: esp_h264_dec_sw_new rejects every other pic_type
  // outright, because tinyh264 has exactly one output format.
  esp_h264_dec_cfg_sw_t cfg = {};
  cfg.pic_type = ESP_H264_RAW_FMT_I420;

  esp_h264_err_t ret = esp_h264_dec_sw_new(&cfg, &this->decoder_);
  if (ret != ESP_H264_ERR_OK || this->decoder_ == nullptr) {
    ESP_LOGE(TAG, "esp_h264_dec_sw_new failed: %d", (int) ret);
    this->decoder_ = nullptr;
    return false;
  }

  // The decoder claims roughly a megabyte of PSRAM for its DPB and working
  // buffers. Logged at INFO rather than DEBUG because the real number is what
  // decides whether anything else on this device still fits.
  ESP_LOGI(TAG, "Decoder created; free heap %u B, free PSRAM %u B",
           (unsigned) heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
           (unsigned) heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

  ret = esp_h264_dec_open(this->decoder_);
  if (ret != ESP_H264_ERR_OK) {
    ESP_LOGE(TAG, "esp_h264_dec_open failed: %d", (int) ret);
    esp_h264_dec_del(this->decoder_);
    this->decoder_ = nullptr;
    return false;
  }
  this->decoder_opened_ = true;

  // The only way to learn the frame geometry. The decoder does not report it
  // through the out frame, and it is not known until an SPS has been parsed, so
  // this handle is fetched now and queried after each picture.
  if (esp_h264_dec_sw_get_param_hd(this->decoder_, &this->decoder_param_) != ESP_H264_ERR_OK) {
    ESP_LOGW(TAG, "Could not get decoder param handle; falling back to %ux%u", (unsigned) FRAME_WIDTH,
             (unsigned) EXPECTED_DECODE_HEIGHT);
    this->decoder_param_ = nullptr;
  }

  this->logged_first_decode_ = false;
  return true;
}

void H264Video::close_decoder_() {
  if (this->decoder_ != nullptr) {
    if (this->decoder_opened_)
      esp_h264_dec_close(this->decoder_);
    esp_h264_dec_del(this->decoder_);
    this->decoder_ = nullptr;
  }
  this->decoder_opened_ = false;
  this->decoder_param_ = nullptr;
}

// ───────────────────────────────────────────────────────────────────────────
// Reassembly
// ───────────────────────────────────────────────────────────────────────────

void H264Video::reset_reassembly_() {
  this->nal_in_progress_ = false;
  this->nal_len_ = 0;
  this->next_frag_ = 0;
  this->frag_count_ = 0;
}

uint32_t H264Video::ingest_datagram_(const uint8_t *data, uint32_t len) {
  // Every field below is read from the network, so each one is checked before
  // it is used to size a copy. A short or hostile datagram must cost a counter
  // increment, never a write past nal_buf_.
  if (len < WIRE_HEADER_SIZE)
    return 0;
  if (data[0] != 'H' || data[1] != '2')
    return 0;

  const uint16_t seq = (uint16_t) (data[2] | ((uint16_t) data[3] << 8));
  const uint8_t frag_idx = data[4];
  const uint8_t frag_count = data[5];
  const uint8_t *payload = data + WIRE_HEADER_SIZE;
  const uint32_t payload_len = len - WIRE_HEADER_SIZE;

  if (frag_count == 0 || frag_idx >= frag_count || payload_len == 0)
    return 0;

  // One NAL in flight, and fragments must arrive in order.
  //
  // This is not laziness. Fragments of one NAL are sent back to back down a
  // single path, so out-of-order delivery here is rare and almost always means
  // the datagram between them is already lost - in which case the NAL is dead
  // whatever we do. Holding a reorder window would add state to be wrong about
  // in exchange for salvaging a case that essentially does not occur, and this
  // is a live stream: the next IDR is a second away.
  if (this->nal_in_progress_ && (seq != this->nal_seq_ || frag_idx != this->next_frag_)) {
    this->nals_dropped_.fetch_add(1, std::memory_order_relaxed);
    this->reset_reassembly_();
  }

  if (!this->nal_in_progress_) {
    // Only a fragment 0 can begin a NAL. Joining mid-NAL would hand the decoder
    // a truncated slice, which is worse than no slice at all.
    if (frag_idx != 0)
      return 0;
    this->nal_in_progress_ = true;
    this->nal_seq_ = seq;
    this->frag_count_ = frag_count;
    this->nal_len_ = 0;
    this->next_frag_ = 0;
  }

  if (this->nal_len_ + payload_len > MAX_NAL_SIZE) {
    ESP_LOGW(TAG, "NAL seq %u exceeds %u byte cap; dropping", (unsigned) seq, (unsigned) MAX_NAL_SIZE);
    this->nals_dropped_.fetch_add(1, std::memory_order_relaxed);
    this->reset_reassembly_();
    return 0;
  }

  memcpy(this->nal_buf_ + sizeof(START_CODE) + this->nal_len_, payload, payload_len);
  this->nal_len_ += payload_len;
  this->next_frag_ = (uint8_t) (frag_idx + 1);

  if (this->next_frag_ < this->frag_count_)
    return 0;

  const uint32_t total = this->nal_len_ + (uint32_t) sizeof(START_CODE);
  this->reset_reassembly_();
  this->nals_received_.fetch_add(1, std::memory_order_relaxed);
  return total;
}

// ───────────────────────────────────────────────────────────────────────────
// Decode
// ───────────────────────────────────────────────────────────────────────────

void H264Video::decode_nal_(uint32_t nal_len_with_start_code) {
  if (this->decoder_ == nullptr)
    return;

  esp_h264_dec_in_frame_t in = {};
  in.raw_data.buffer = this->nal_buf_;
  in.raw_data.len = nal_len_with_start_code;

  esp_h264_dec_out_frame_t out = {};

  const uint32_t t0 = millis();
  bool produced = false;

  while (in.raw_data.len > 0) {
    in.consume = 0;
    const esp_h264_err_t ret = esp_h264_dec_process(this->decoder_, &in, &out);

    if (ret == ESP_H264_ERR_OK) {
      // There is no "need more data" status in this API. A NAL that carried no
      // picture - an SPS or PPS, which this stream repeats before every IDR -
      // returns OK with out_size 0, and that is the only way to tell.
      if (out.out_size > 0 && out.outbuf != nullptr) {
        // outbuf points straight into the decoder's DPB and is overwritten by
        // the next process call, so the conversion has to happen here, before
        // the loop goes round again - not queued for later.
        uint32_t src_w = FRAME_WIDTH;
        uint32_t src_h = EXPECTED_DECODE_HEIGHT;
        esp_h264_resolution_t res = {};
        if (this->decoder_param_ != nullptr &&
            esp_h264_dec_get_resolution(this->decoder_param_, &res) == ESP_H264_ERR_OK && res.width > 0 &&
            res.height > 0) {
          src_w = res.width;
          src_h = res.height;
        }

        // The reported geometry has to agree with the buffer we were handed, or
        // the plane offsets computed from it would read outside it. I420 is
        // 1.5 bytes per pixel.
        if ((uint32_t) (src_w * src_h + (src_w * src_h >> 1)) > out.out_size) {
          ESP_LOGW(TAG, "Decoder geometry %ux%u disagrees with %u byte output; skipping frame", (unsigned) src_w,
                   (unsigned) src_h, (unsigned) out.out_size);
          this->decode_errors_.fetch_add(1, std::memory_order_relaxed);
        } else {
          // Crop happens here and nowhere else: the decoder never applies the
          // SPS cropping window, so src_h is the macroblock-aligned 144 and we
          // simply stop after FRAME_HEIGHT rows.
          i420_to_rgb565(out.outbuf, src_w, src_h, this->fb_[this->back_index_], FRAME_WIDTH, FRAME_HEIGHT);
          this->publish_frame_();
          produced = true;
        }
        out.out_size = 0;
        out.outbuf = nullptr;
      }
    } else {
      this->decode_errors_.fetch_add(1, std::memory_order_relaxed);
      ESP_LOGD(TAG, "esp_h264_dec_process returned %d", (int) ret);
      // consume is written before the error return, so the stream can be
      // advanced past the bad bytes and kept running. Aborting would be wrong
      // for a lossy live link, where a corrupt slice is normal and the next IDR
      // recovers on its own.
    }

    // Guard against a zero-consume return, whatever the status: without this a
    // decoder that declines to advance would spin this task forever.
    if (in.consume == 0 || in.consume > in.raw_data.len)
      break;
    in.raw_data.buffer += in.consume;
    in.raw_data.len -= in.consume;
  }

  if (produced) {
    this->last_decode_ms_.store(millis() - t0, std::memory_order_relaxed);
    this->frames_decoded_.fetch_add(1, std::memory_order_relaxed);
    this->last_frame_ms_.store(millis(), std::memory_order_release);

    if (!this->logged_first_decode_) {
      this->logged_first_decode_ = true;
      // The steady-state footprint, taken once the DPB and reference frames are
      // actually populated - the number after esp_h264_dec_sw_new alone is
      // optimistic.
      ESP_LOGI(TAG, "First frame decoded in %u ms; free heap %u B, free PSRAM %u B",
               (unsigned) this->last_decode_ms_.load(std::memory_order_relaxed),
               (unsigned) heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
               (unsigned) heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    }
  }
}

void H264Video::publish_frame_() {
  // Publication is a single release store of the index the task just finished
  // writing. Until it lands, a reader keeps seeing the previous buffer whole;
  // after it lands, it sees the new one whole. There is no moment at which
  // anything reads a buffer that is being written, which is the entire reason
  // for the second buffer - LVGL tearing a frame mid-copy is the failure this
  // prevents.
  const int8_t just_written = (int8_t) this->back_index_;
  this->published_index_.store(just_written, std::memory_order_release);
  this->back_index_ ^= 1;
  this->new_frame_.store(true, std::memory_order_release);
}

// ───────────────────────────────────────────────────────────────────────────
// Task
// ───────────────────────────────────────────────────────────────────────────

void H264Video::decode_task(void *param) {
  H264Video *self = (H264Video *) param;
  self->decode_task_body_();
}

void H264Video::decode_task_body_() {
  ESP_LOGD(TAG, "Decode task started");

  while (this->running_.load(std::memory_order_acquire)) {
    struct sockaddr_in from;
    socklen_t from_len = sizeof(from);
    const int n = recvfrom(this->socket_, this->rx_buf_, MAX_DATAGRAM, 0, (struct sockaddr *) &from, &from_len);

    if (n <= 0) {
      // EAGAIN/EWOULDBLOCK is just the receive timeout expiring, which is how
      // this loop gets its chance to notice a stop().
      if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
        ESP_LOGW(TAG, "recvfrom failed: %d", errno);
      continue;
    }

    const uint32_t nal_len = this->ingest_datagram_(this->rx_buf_, (uint32_t) n);
    if (nal_len > 0)
      this->decode_nal_(nal_len);
  }

  ESP_LOGD(TAG, "Decode task stopping");

  // Nothing is freed here, and that is deliberate. The socket, the buffers and
  // the decoder are owned by stop(), which joins on task_done_ before touching
  // any of them. A task that tore down its own state could not know it still
  // owned it: if stop() ever gave up waiting and a later start() built fresh
  // buffers for a fresh task, this one waking up late would be freeing the new
  // task's framebuffers out from under it.
  //
  // The give is the last thing that happens on every path out of this function,
  // so a stop() returning from its take() knows this task is finished.
  if (this->task_done_ != nullptr)
    xSemaphoreGive(this->task_done_);
  vTaskDelete(nullptr);
}

// ───────────────────────────────────────────────────────────────────────────
// Lifecycle
// ───────────────────────────────────────────────────────────────────────────

void H264Video::start() {
  if (this->running_.load(std::memory_order_acquire)) {
    ESP_LOGW(TAG, "Already running");
    return;
  }

  ESP_LOGI(TAG, "Starting H.264 video receiver on port %u", (unsigned) this->port_);

  if (!this->alloc_buffers_()) {
    this->free_buffers_();
    return;
  }

  if (!this->open_decoder_()) {
    this->free_buffers_();
    return;
  }

  if (!this->open_socket_()) {
    this->close_decoder_();
    this->free_buffers_();
    return;
  }

  this->reset_reassembly_();
  this->back_index_ = 0;
  this->published_index_.store(-1, std::memory_order_release);
  this->new_frame_.store(false, std::memory_order_release);
  this->nals_received_.store(0, std::memory_order_relaxed);
  this->nals_dropped_.store(0, std::memory_order_relaxed);
  this->frames_decoded_.store(0, std::memory_order_relaxed);
  this->decode_errors_.store(0, std::memory_order_relaxed);
  this->last_decode_ms_.store(0, std::memory_order_relaxed);
  this->started_ms_ = millis();
  this->last_frame_ms_.store(0, std::memory_order_release);

  // Created before the task exists, so there is no window in which the task is
  // running and has nothing to signal its exit on.
  this->task_done_ = xSemaphoreCreateBinary();
  if (this->task_done_ == nullptr) {
    ESP_LOGE(TAG, "Failed to create task semaphore");
    this->close_socket_();
    this->close_decoder_();
    this->free_buffers_();
    return;
  }

  // running_ must be true before the task exists, or the task's loop condition
  // is false on its first evaluation and it exits immediately.
  this->running_.store(true, std::memory_order_release);

  const BaseType_t result = xTaskCreatePinnedToCore(decode_task, "h264_video", TASK_STACK_SIZE, this, TASK_PRIORITY,
                                                    &this->task_handle_, TASK_CORE);
  if (result != pdPASS) {
    ESP_LOGE(TAG, "Failed to create decode task");
    this->running_.store(false, std::memory_order_release);
    vSemaphoreDelete(this->task_done_);
    this->task_done_ = nullptr;
    this->task_handle_ = nullptr;
    this->close_socket_();
    this->close_decoder_();
    this->free_buffers_();
    return;
  }

  ESP_LOGI(TAG, "H.264 video receiver started");
}

void H264Video::stop() {
  if (!this->running_.load(std::memory_order_acquire)) {
    ESP_LOGW(TAG, "Not running");
    return;
  }

  ESP_LOGD(TAG, "Stopping H.264 video receiver");

  this->running_.store(false, std::memory_order_release);

  // Join on the semaphore, not on eTaskGetState().
  //
  // Polling the handle is a use-after-free: the task self-deletes with
  // vTaskDelete(NULL), the idle task reaps its TCB almost immediately, and this
  // runs on the main task while the decode task is pinned to core 1 - so the
  // poll usually misses the eDeleted window entirely and then reads a dangling
  // pointer into freed heap, turning the timeout into the normal exit path
  // rather than the exceptional one. The same bug was fixed in the audio
  // component; it is not repeated here.
  //
  // 2s is generous against a 100ms socket timeout plus one in-flight decode, so
  // a timeout here means something is genuinely wrong.
  if (this->task_handle_ != nullptr) {
    if (this->task_done_ != nullptr && xSemaphoreTake(this->task_done_, pdMS_TO_TICKS(2000)) != pdTRUE)
      ESP_LOGE(TAG, "Decode task did not exit within 2s; tearing down anyway");
    if (this->task_done_ != nullptr) {
      vSemaphoreDelete(this->task_done_);
      this->task_done_ = nullptr;
    }
    this->task_handle_ = nullptr;
  }

  // Retract the published frame before the memory behind it goes away, so a
  // lambda that runs between here and the free cannot be handed a descriptor
  // pointing into a buffer that is about to be released.
  this->published_index_.store(-1, std::memory_order_release);
  this->new_frame_.store(false, std::memory_order_release);

  this->close_socket_();
  this->close_decoder_();
  this->free_buffers_();

  ESP_LOGI(TAG, "H.264 video receiver stopped; free heap %u B, free PSRAM %u B",
           (unsigned) heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
           (unsigned) heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
}

// ───────────────────────────────────────────────────────────────────────────
// Frame handoff
// ───────────────────────────────────────────────────────────────────────────

#ifdef USE_LVGL
const lv_img_dsc_t *H264Video::get_frame() {
  const int8_t idx = this->published_index_.load(std::memory_order_acquire);
  if (idx < 0 || this->fb_[idx] == nullptr)
    return nullptr;
  return &this->img_dsc_[idx];
}
#endif

const uint16_t *H264Video::get_frame_buffer() {
  const int8_t idx = this->published_index_.load(std::memory_order_acquire);
  if (idx < 0)
    return nullptr;
  return this->fb_[idx];
}

bool H264Video::is_stale() const {
  if (!this->running_.load(std::memory_order_acquire))
    return true;
  const uint32_t last = this->last_frame_ms_.load(std::memory_order_acquire);
  // Before the first frame the timeout is measured from start(), so a receiver
  // that never hears anything reports stale rather than sitting at "fresh"
  // forever on a last_frame_ms_ of zero.
  const uint32_t since = (last == 0) ? this->started_ms_ : last;
  return (millis() - since) > this->frame_timeout_ms_;
}

}  // namespace h264_video
}  // namespace esphome
