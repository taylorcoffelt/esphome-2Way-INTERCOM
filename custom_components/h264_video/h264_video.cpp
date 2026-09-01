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
// reason, and source_height is what we expect back - but nothing below assumes
// it, the real dimensions are read from the decoder after every picture and the
// configured pair is only the fallback.
//
// The framebuffer size is output_width x output_height and therefore per
// instance rather than a file-scope constant; see H264Video::fb_bytes_().

// Wire header: 'H','2', uint16 LE sequence, uint8 fragment index, uint8
// fragment count, two reserved bytes.
static const uint32_t WIRE_HEADER_SIZE = 8;
static const uint32_t MAX_DATAGRAM = 1400;

// 255 fragments is what the one-byte count allows, but a constrained baseline
// IDR at the sizes this component is fed - 240x144, or 480x288 for the wall
// panel - is a few KB and a P-frame far less. 32KB keeps roughly an order of
// magnitude of headroom over anything either stream produces while still being
// a rounding error in PSRAM, and it bounds the damage a lying fragment count
// can do.
static const uint32_t MAX_NAL_SIZE = 32768;

// Prepended to every reassembled NAL. The decoder is fed Annex-B and will not
// find a NAL without one - the wire format strips start codes to save four
// bytes per datagram, so they have to come back before the decoder sees them.
static const uint8_t START_CODE[4] = {0x00, 0x00, 0x00, 0x01};

// The NAL header is the byte immediately after the start code, and its low five
// bits are the unit type: 1 non-IDR slice, 5 IDR slice, 7 SPS, 8 PPS. Only the
// SPS needs naming here - it is the one type that makes the stream decodable
// from that byte onwards, which is what the join gate in decode_nal_() waits
// for.
static const uint8_t NAL_TYPE_SPS = 7;

// How long the socket blocks before the loop rechecks running_. Short enough
// that stop() joins promptly, long enough that a silent stream does not spin.
static const int RECV_TIMEOUT_MS = 100;

static inline uint8_t clamp_u8(int32_t v) {
  if (v < 0)
    return 0;
  if (v > 255)
    return 255;
  return (uint8_t) v;
}

// RGB565 in native byte order. Nothing here pre-swaps, and that is the whole
// point of this comment.
//
// The trap is that LV_COLOR_16_SWAP reads like it describes the framebuffer and
// does not - under ESPHome it describes the *panel*. The lvgl component sets it
// from the display's byte_order, which is big_endian for this ST7789, so it is
// 1 in this build; then LvglComponent::draw_buffer_ hands that same flag to
// display->draw_pixels_at() as its bswap argument. The swap happens once, at
// flush, on the way out to the bus. LVGL 9 reads the macro in exactly one place
// of its own - a v8 compatibility shim in lv_refr.c gated on
// LV_DISPLAY_RENDER_MODE_DIRECT - and ESPHome renders PARTIAL or FULL, so that
// shim never runs either.
//
// Everything LVGL composites from must therefore be native little-endian
// RGB565, image descriptors included. ESPHome's own assets agree: the image
// component defaults RGB565 to LITTLE_ENDIAN and warns if you ask for
// big-endian, and those are the online_image sources that render correctly
// beside this one.
//
// Swapping here got swapped again at flush and put every pixel on the panel
// reversed. That is not a tint. 0xFFFF and 0x0000 are palindromes, so white and
// black come through untouched while everything between them lands in the
// greens and magentas - a permanent green-and-purple picture with motion still
// perfectly legible, because the luma structure is entirely intact and only the
// byte order is wrong. Nor does a converter unit test catch it: built outside
// the firmware there is no USE_LVGL, the swap compiled out, and the test
// measured the one configuration the device never builds.
static inline uint16_t pack_rgb565(int32_t r, int32_t g, int32_t b) {
  return (uint16_t) (((clamp_u8(r) & 0xF8) << 8) | ((clamp_u8(g) & 0xFC) << 3) | (clamp_u8(b) >> 3));
}

// Nearest-neighbour source index for output index i, sample-centre mapped:
// floor((i + 0.5) * src / dst), in integers as ((2i + 1) * src) / (2 * dst).
//
// Two properties are load-bearing. It returns exactly i when src == dst, so an
// unscaled axis is the identity and the two paths below agree on their shared
// case; and it is always strictly less than src, because (2i + 1) is at most
// (2*dst - 1), so a table built from it can never index off the end of a plane.
static inline uint16_t nn_src_index(uint32_t i, uint32_t src, uint32_t dst) {
  return (uint16_t) (((uint64_t) (2 * i + 1) * (uint64_t) src) / ((uint64_t) 2 * (uint64_t) dst));
}

// The three chroma terms of the BT.601 limited-range transform. They are
// constant across every pixel sharing a chroma sample, which is the whole
// reason the loops below are shaped around a quad. The +128 is the rounding
// term for the >> 8 that follows.
static inline void chroma_offsets(int32_t u, int32_t v, int32_t *r_off, int32_t *g_off, int32_t *b_off) {
  const int32_t d = u - 128;
  const int32_t e = v - 128;
  *r_off = 409 * e + 128;
  *g_off = -100 * d - 208 * e + 128;
  *b_off = 516 * d + 128;
}

// One pixel: the luma term, then the same shift-and-pack as ever.
static inline uint16_t yuv_to_rgb565(int32_t luma, int32_t r_off, int32_t g_off, int32_t b_off) {
  const int32_t c = 298 * (luma - 16);
  return pack_rgb565((c + r_off) >> 8, (c + g_off) >> 8, (c + b_off) >> 8);
}

// I420 -> RGB565, 1:1.
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
//
// This is the original converter unchanged, and it stays a separate function
// rather than a special case of the scaling one on purpose: the small panel
// runs this path on every frame and must not pay a table lookup, a second
// bounds test or a branch per pixel for a resample it never does.
static void i420_copy_rgb565(const uint8_t *src, uint32_t src_w, uint32_t src_h, uint16_t *dst, uint32_t dst_w,
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
      int32_t r_off, g_off, b_off;
      chroma_offsets(row_u[x >> 1], row_v[x >> 1], &r_off, &g_off, &b_off);

      out0[x] = yuv_to_rgb565(row_y0[x], r_off, g_off, b_off);
      if (x + 1 < w)
        out0[x + 1] = yuv_to_rgb565(row_y0[x + 1], r_off, g_off, b_off);

      if (has_row1) {
        out1[x] = yuv_to_rgb565(row_y1[x], r_off, g_off, b_off);
        if (x + 1 < w)
          out1[x + 1] = yuv_to_rgb565(row_y1[x + 1], r_off, g_off, b_off);
      }
    }
  }
}

// I420 -> RGB565 with a nearest-neighbour resample folded into the same pass.
//
// One traversal, not two. The alternative - convert at source size, then let
// LVGL scale the image on draw - reads and writes the 800x480 rectangle a
// second time every frame, on the main loop, in the middle of the redraw this
// component exists to feed.
//
// Nearest neighbour rather than bilinear is deliberate. This is a 10fps camera
// feed being enlarged, not text: bilinear would cost three more multiplies and
// three more plane reads per component per pixel to soften an image whose real
// limit is the encoder, not the resampler.
//
// x_map and y_map hold the source index for each output column and row, built
// once by build_scale_maps_(). No multiply or divide happens per pixel, and
// because both tables are non-decreasing the source planes are still read in
// forward order - which is what matters when they are in PSRAM.
//
// The quad structure survives scaling and is worth more here than at 1:1. When
// two adjacent output columns land on the same source column (the common case
// when enlarging) and the two output rows land on the same chroma row, all four
// pixels share one chroma pair and it is loaded once, exactly as before.
static void i420_scale_rgb565(const uint8_t *src, uint32_t src_w, uint32_t src_h, uint16_t *dst, uint32_t dst_w,
                              uint32_t dst_h, const uint16_t *x_map, const uint16_t *y_map) {
  if (dst_w == 0 || dst_h == 0 || src_w == 0 || src_h == 0)
    return;

  // The tables were built for the configured source geometry; this frame's
  // actual geometry comes from the decoder and is only checked against
  // out_size. If the decoder ever hands back something smaller than configured,
  // trim the output rather than read off the end of a plane. Both maps are
  // non-decreasing, so testing the last entry and walking back is exact, and it
  // is zero iterations whenever the geometry is what it should be.
  uint32_t cols = dst_w;
  while (cols > 0 && x_map[cols - 1] >= src_w)
    cols--;
  uint32_t rows = dst_h;
  while (rows > 0 && y_map[rows - 1] >= src_h)
    rows--;
  if (cols == 0 || rows == 0)
    return;

  const uint32_t y_stride = src_w;
  const uint32_t c_stride = src_w >> 1;
  const uint8_t *plane_y = src;
  const uint8_t *plane_u = plane_y + (uint32_t) src_w * src_h;
  const uint8_t *plane_v = plane_u + ((uint32_t) src_w * src_h >> 2);

  for (uint32_t dy = 0; dy < rows; dy += 2) {
    const uint32_t sy0 = y_map[dy];
    // Same odd-height guard as the 1:1 path, on the output side: an 800x480
    // target is even, but nothing here requires it to be.
    const bool has_row1 = (dy + 1) < rows;
    const uint32_t sy1 = has_row1 ? y_map[dy + 1] : sy0;

    const uint8_t *row_y0 = plane_y + (uint32_t) sy0 * y_stride;
    const uint8_t *row_y1 = plane_y + (uint32_t) sy1 * y_stride;

    const uint32_t cy0 = sy0 >> 1;
    const uint32_t cy1 = sy1 >> 1;
    const bool same_c_row = (cy0 == cy1);
    const uint8_t *row_u0 = plane_u + cy0 * c_stride;
    const uint8_t *row_v0 = plane_v + cy0 * c_stride;
    const uint8_t *row_u1 = same_c_row ? row_u0 : (plane_u + cy1 * c_stride);
    const uint8_t *row_v1 = same_c_row ? row_v0 : (plane_v + cy1 * c_stride);

    uint16_t *out0 = dst + (uint32_t) dy * dst_w;
    uint16_t *out1 = has_row1 ? (out0 + dst_w) : out0;

    for (uint32_t dx = 0; dx < cols; dx += 2) {
      const uint32_t sx0 = x_map[dx];
      const bool has_col1 = (dx + 1) < cols;
      const uint32_t sx1 = has_col1 ? x_map[dx + 1] : sx0;
      const uint32_t cx0 = sx0 >> 1;
      const uint32_t cx1 = sx1 >> 1;
      const bool same_c_col = (cx0 == cx1);

      // Top-left. When the quad is chroma-uniform - both output columns on one
      // source chroma column and both output rows on one chroma row - this is
      // the only chroma load the quad performs.
      int32_t r00, g00, b00;
      chroma_offsets(row_u0[cx0], row_v0[cx0], &r00, &g00, &b00);

      int32_t r01 = r00, g01 = g00, b01 = b00;
      if (has_col1 && !same_c_col)
        chroma_offsets(row_u0[cx1], row_v0[cx1], &r01, &g01, &b01);

      out0[dx] = yuv_to_rgb565(row_y0[sx0], r00, g00, b00);
      if (has_col1)
        out0[dx + 1] = yuv_to_rgb565(row_y0[sx1], r01, g01, b01);

      if (has_row1) {
        int32_t r10 = r00, g10 = g00, b10 = b00;
        int32_t r11 = r01, g11 = g01, b11 = b01;
        if (!same_c_row) {
          chroma_offsets(row_u1[cx0], row_v1[cx0], &r10, &g10, &b10);
          if (has_col1 && !same_c_col) {
            chroma_offsets(row_u1[cx1], row_v1[cx1], &r11, &g11, &b11);
          } else {
            r11 = r10;
            g11 = g10;
            b11 = b10;
          }
        }

        out1[dx] = yuv_to_rgb565(row_y1[sx0], r10, g10, b10);
        if (has_col1)
          out1[dx + 1] = yuv_to_rgb565(row_y1[sx1], r11, g11, b11);
      }
    }
  }
}

// The one entry point the decode path calls. A null map pair means the output
// rectangle equals the visible source rectangle, so no resampling is needed and
// the original 1:1 loop runs untouched.
static void i420_to_rgb565(const uint8_t *src, uint32_t src_w, uint32_t src_h, uint16_t *dst, uint32_t dst_w,
                           uint32_t dst_h, const uint16_t *x_map, const uint16_t *y_map) {
  if (x_map == nullptr || y_map == nullptr) {
    i420_copy_rgb565(src, src_w, src_h, dst, dst_w, dst_h);
  } else {
    i420_scale_rgb565(src, src_w, src_h, dst, dst_w, dst_h, x_map, y_map);
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
  ESP_LOGCONFIG(TAG, "  Source: %ux%u decoded (%u visible row(s), %u cropped)", (unsigned) this->source_width_,
                (unsigned) this->source_height_, (unsigned) this->visible_height_,
                (unsigned) (this->source_height_ - this->visible_height_));
  if (this->is_scaling()) {
    ESP_LOGCONFIG(TAG, "  Output: %ux%u RGB565 (scaling %ux%u -> %ux%u, nearest neighbour)",
                  (unsigned) this->output_width_, (unsigned) this->output_height_, (unsigned) this->source_width_,
                  (unsigned) this->visible_height_, (unsigned) this->output_width_, (unsigned) this->output_height_);
  } else {
    ESP_LOGCONFIG(TAG, "  Output: %ux%u RGB565 (1:1, no scaling)", (unsigned) this->output_width_,
                  (unsigned) this->output_height_);
  }
  ESP_LOGCONFIG(TAG, "  Frame Timeout: %u ms", (unsigned) this->frame_timeout_ms_);
  ESP_LOGCONFIG(TAG, "  Task: core %d, priority %u, %u byte stack", TASK_CORE, (unsigned) TASK_PRIORITY,
                (unsigned) TASK_STACK_SIZE);
  // Unconditional now - see the note above pack_rgb565. Any swap the panel wants
  // is ESPHome's to apply at flush, and applying it here too was the bug.
  ESP_LOGCONFIG(TAG, "  Pixel Order: RGB565 native little-endian");
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
  // PSRAM for everything. A framebuffer is output_width x output_height x 2 -
  // 64,800 bytes for the small panel and 768,000 for the 800x480 one - which
  // would be a painful bite out of internal RAM next to WiFi and the audio
  // path, and neither buffer is touched from an ISR. The scale tables are the
  // one exception; see build_scale_maps_().
  const uint32_t fb_bytes = this->fb_bytes_();
  for (int i = 0; i < 2; i++) {
    this->fb_[i] = (uint16_t *) heap_caps_malloc(fb_bytes, MALLOC_CAP_SPIRAM);
    if (this->fb_[i] == nullptr) {
      ESP_LOGE(TAG, "Failed to allocate %u byte framebuffer %d in PSRAM", (unsigned) fb_bytes, i);
      return false;
    }
    // Black rather than whatever the allocator handed back: a consumer that
    // draws before the first decode should see a blank panel, not PSRAM noise.
    memset(this->fb_[i], 0, fb_bytes);
  }

  if (!this->build_scale_maps_())
    return false;

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
    d.header.stride = this->output_width_ * 2;
#else
    d.header.always_zero = 0;
    // TRUE_COLOR means "already in lv_color_t layout", which on this build is
    // RGB565 - so the converter's output is drawn straight out of PSRAM with no
    // per-frame decode step in LVGL.
    d.header.cf = LV_IMG_CF_TRUE_COLOR;
#endif
    d.header.w = this->output_width_;
    d.header.h = this->output_height_;
    d.data_size = fb_bytes;
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
  this->free_scale_maps_();
#ifdef USE_LVGL
  // The descriptors point at buffers that no longer exist. get_frame() is
  // already gated on published_index_, but zeroing these means a stale
  // descriptor cannot be dereferenced into freed PSRAM by anything else.
  memset(this->img_dsc_, 0, sizeof(this->img_dsc_));
#endif
}

bool H264Video::build_scale_maps_() {
  this->free_scale_maps_();

  // No resample, no tables. The null pair is what selects the 1:1 loop in the
  // converter, so this is not merely an allocation saved.
  if (!this->is_scaling())
    return true;

  const uint32_t src_w = this->source_width_;
  const uint32_t src_h = this->visible_height_;
  const uint32_t dst_w = this->output_width_;
  const uint32_t dst_h = this->output_height_;

  // Internal RAM. Together they are a few KB - 2,560 bytes for 800x480 - and
  // they are the only randomly-indexed reads in the inner loop, so they are the
  // one thing here that should not be in PSRAM.
  this->x_map_ = (uint16_t *) heap_caps_malloc(dst_w * sizeof(uint16_t), MALLOC_CAP_INTERNAL);
  this->y_map_ = (uint16_t *) heap_caps_malloc(dst_h * sizeof(uint16_t), MALLOC_CAP_INTERNAL);
  if (this->x_map_ == nullptr || this->y_map_ == nullptr) {
    ESP_LOGE(TAG, "Failed to allocate %u byte scale tables", (unsigned) ((dst_w + dst_h) * sizeof(uint16_t)));
    this->free_scale_maps_();
    return false;
  }

  for (uint32_t i = 0; i < dst_w; i++)
    this->x_map_[i] = nn_src_index(i, src_w, dst_w);
  for (uint32_t i = 0; i < dst_h; i++)
    this->y_map_[i] = nn_src_index(i, src_h, dst_h);

  // Built from visible_height, never source_height: that is the entire crop.
  // The padding rows the encoder added to reach macroblock alignment have no
  // output row mapped to them and are never read.
  ESP_LOGD(TAG, "Scale tables built: %ux%u -> %ux%u (x %u..%u, y %u..%u)", (unsigned) src_w, (unsigned) src_h,
           (unsigned) dst_w, (unsigned) dst_h, (unsigned) this->x_map_[0], (unsigned) this->x_map_[dst_w - 1],
           (unsigned) this->y_map_[0], (unsigned) this->y_map_[dst_h - 1]);
  return true;
}

void H264Video::free_scale_maps_() {
  if (this->x_map_ != nullptr) {
    heap_caps_free(this->x_map_);
    this->x_map_ = nullptr;
  }
  if (this->y_map_ != nullptr) {
    heap_caps_free(this->y_map_);
    this->y_map_ = nullptr;
  }
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
    ESP_LOGW(TAG, "Could not get decoder param handle; falling back to %ux%u", (unsigned) this->source_width_,
             (unsigned) this->source_height_);
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

  // Nothing reaches the decoder until the stream has been synchronised to a
  // parameter set.
  //
  // The sender runs continuously and we join it wherever the socket happens to
  // open, so the first NALs to arrive are almost always P-slices from a GOP
  // whose SPS, PPS and IDR went past before this receiver existed. The trap is
  // that tinyh264 does not reject them: it decodes them against a reference
  // picture it never received, reports a picture ready, and hands back a frame
  // whose luma is noise and whose chroma sits at the extremes - which is the
  // green and magenta the display shows for the first second or two of every
  // stream. There is no error to test for downstream, because as far as the
  // decoder is concerned nothing went wrong, so the only place to stop it is
  // here, before it is ever given the slice.
  //
  // Waiting costs at most one keyframe interval: the sender runs with
  // repeat-headers=1, so an SPS and PPS precede every IDR, and starting the
  // feed at an SPS guarantees the decoder holds its parameter sets before it
  // sees its first slice. NALs skipped this way are counted as drops rather
  // than decode errors - landing mid-GOP is what joining a live stream looks
  // like, not a fault.
  if (!this->synced_) {
    if ((this->nal_buf_[sizeof(START_CODE)] & 0x1F) != NAL_TYPE_SPS) {
      this->nals_dropped_.fetch_add(1, std::memory_order_relaxed);
      return;
    }
    this->synced_ = true;
    ESP_LOGI(TAG, "Stream synchronised on SPS after %u NAL(s) skipped mid-GOP",
             (unsigned) this->nals_dropped_.load(std::memory_order_relaxed));
  }

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
        uint32_t src_w = this->source_width_;
        uint32_t src_h = this->source_height_;
        esp_h264_resolution_t res = {};
        if (this->decoder_param_ != nullptr &&
            esp_h264_dec_get_resolution(this->decoder_param_, &res) == ESP_H264_ERR_OK && res.width > 0 &&
            res.height > 0) {
          src_w = res.width;
          src_h = res.height;
        }

        // The reported geometry has to agree with the buffer we were handed, or
        // the plane offsets computed from it would read outside it. I420 is
        // 1.5 bytes per pixel. When the decoder will not say, src_w/src_h are
        // the configured source geometry, so this is still a real check and not
        // a tautology.
        if ((uint32_t) (src_w * src_h + (src_w * src_h >> 1)) > out.out_size) {
          ESP_LOGW(TAG, "Decoder geometry %ux%u disagrees with %u byte output; skipping frame", (unsigned) src_w,
                   (unsigned) src_h, (unsigned) out.out_size);
          this->decode_errors_.fetch_add(1, std::memory_order_relaxed);
        } else {
          // Crop happens here and nowhere else: the decoder never applies the
          // SPS cropping window, so src_h is the macroblock-aligned height and
          // only the visible rows are converted. At 1:1 that is the dst_h clip
          // inside the loop; when scaling, y_map_ was built over visible_height
          // and no output row maps to a padding one.
          i420_to_rgb565(out.outbuf, src_w, src_h, this->fb_[this->back_index_], this->output_width_,
                         this->output_height_, this->x_map_, this->y_map_);
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
  // Sync is per-run: a fresh socket is a fresh join, and the SPS that made the
  // last run decodable says nothing about where this one lands in the GOP.
  this->synced_ = false;
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
  this->synced_ = false;

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
