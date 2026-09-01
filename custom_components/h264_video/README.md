# H.264 Video Component

Receives H.264 Annex-B NAL units over UDP, decodes them with Espressif's
`esp_h264` software decoder (tinyh264), and publishes RGB565 frames for LVGL.

Built for the M5Stack StickS3 (ESP32-S3-PICO-1-N8R8, 8MB octal PSRAM, 240x135
ST7789V) running alongside the `i2s_audio_udp` component, and configurable for
larger panels - an 800x480 RGB wall panel fed a 480x288 stream is the other
supported shape.

## Features

- **UDP receive and fragment reassembly** on a configurable port (default 12347)
- **Software H.264 decode** on its own FreeRTOS task, core 1, priority 5
- **I420 to RGB565 conversion** in fixed-point integer math, 2x2 quad at a time
- **Optional nearest-neighbour scaling** folded into that same pass, so a
  480x288 stream reaches an 800x480 panel in one traversal rather than
  decode -> convert -> let LVGL scale
- **Configurable geometry** - source, crop and output are three separate
  rectangles (see below); the defaults are the original 240x144 -> 240x135
- **Double buffered** RGB565 output in PSRAM, published atomically
- **Thread-safe LVGL handoff** - the decode task never touches LVGL
- **Diagnostics** - NALs received/dropped, frames decoded, decode errors, decode time

## Wire protocol

A server fans out Annex-B NAL units over UDP. Each datagram is an 8-byte
little-endian header followed by a NAL fragment, and is at most 1400 bytes
total.

| Offset | Size | Field                                       |
|--------|------|---------------------------------------------|
| 0      | 2    | Magic, ASCII `'H'`, `'2'`                   |
| 2      | 2    | `uint16` NAL sequence number, LE, wraps     |
| 4      | 1    | Fragment index, 0-based                     |
| 5      | 1    | Fragment count                              |
| 6      | 2    | Reserved, zero                              |
| 8      | ...  | NAL payload, **without** its start code     |

A NAL larger than the payload cap is split into fragments that share one
sequence number. The four-byte start code is stripped on the wire and prepended
again by this component before the decoder sees the NAL - the decoder requires
Annex-B and will not find a NAL without one.

### Reassembly rules

The receiver holds exactly **one** in-progress NAL and requires fragments in
order. A datagram with a different sequence number than the one in progress, or
with an unexpected fragment index, discards the partial NAL and counts a drop.
Only a fragment 0 may begin a NAL.

This is intentional rather than a simplification for its own sake. Fragments of
one NAL are sent back to back down a single path, so out-of-order delivery here
almost always means the datagram between them is already lost, and the NAL is
dead either way. A reorder window would add state to be wrong about in exchange
for salvaging a case that essentially does not occur on this link - and the next
IDR is about a second away.

Reassembly is capped at 32KB per NAL, well above anything a constrained baseline
IDR at either supported size (240x144 or 480x288) produces, which also bounds
what a malformed fragment count can do.

## Three rectangles: source, visible, output

Geometry is three separate sizes and they are usually all different. Getting the
distinction wrong is the most common way to configure this component badly, so
it is worth being explicit:

| Rectangle   | Option(s)                        | What it is                                              |
|-------------|----------------------------------|---------------------------------------------------------|
| **source**  | `source_width`, `source_height`  | What the decoder hands back. The **encoded** frame size. |
| **visible** | `source_width`, `visible_height` | How much of that is picture rather than encoder padding. |
| **output**  | `output_width`, `output_height`  | The RGB565 buffer LVGL draws from.                       |

### Why source and visible differ: the macroblock-alignment constraint

**The decoder does not apply frame cropping.** It returns macroblock-aligned
frames and ignores the SPS cropping window entirely.

The StickS3 display is 240x135. 135 is not a multiple of 16, so a 240x135 stream
would come back as 240x144 with nine rows of encoder padding that must not reach
the panel. The stream is therefore encoded **240x144** outright
(`source_height: 144`) and only the top **135** rows are converted
(`visible_height: 135`).

`source_width` and `source_height` are **validated as multiples of 16** and the
config fails if they are not. That check is there because the failure is
otherwise silent: an unaligned source does not error, it decodes to the aligned
size and every plane offset computed from the unaligned number lands mid-row, so
the picture shears further to one side with every line and there is nothing in
the logs to trace it back to.

A stream whose height is already a multiple of 16 - 480x288, for instance -
needs no crop, and sets `visible_height` equal to `source_height`.

Nothing assumes the configured numbers come back. The real geometry is read from
the decoder (`esp_h264_dec_get_resolution`) after every picture and checked
against the reported output size; the configured pair is only the fallback for
when the decoder will not say.

### Why visible and output differ: scaling

Leave `output_width` x `output_height` equal to `source_width` x
`visible_height` and the converter does a straight colour conversion, exactly as
it always has. Set them larger (or smaller) and it resamples on the way through,
nearest neighbour, using per-axis source-index tables built once at `start()` -
no multiply or divide per pixel.

This is done in the converter rather than by handing LVGL a small image and
letting it scale on draw, because that reads and writes the full output
rectangle a second time every frame, on the main loop, in the middle of the
redraw this component exists to feed. One traversal, not two.

Nearest neighbour rather than bilinear is deliberate: this is a 10fps camera
feed being enlarged, not text. Bilinear would cost three more multiplies and
three more plane reads per component per pixel to soften an image whose real
limit is the encoder.

## Encoder flags on the server side

The stream must be constrained baseline, at a macroblock-aligned size, with
SPS/PPS repeated before every IDR so a receiver that joins late (or loses an
IDR) can recover without a signalling channel.

With `ffmpeg`:

```
ffmpeg -i <source> \
  -c:v libx264 \
  -profile:v baseline \
  -level 1.3 \
  -s 240x144 \
  -r 10 \
  -g 10 \
  -bf 0 \
  -refs 1 \
  -tune zerolatency \
  -x264-params "repeat-headers=1:annexb=1:sliced-threads=0:aud=0" \
  -pix_fmt yuv420p \
  -f h264 -
```

The parts that are not negotiable:

- `-profile:v baseline` and `-bf 0` - tinyh264 decodes baseline only, no B-frames.
- `-s 240x144` - see the alignment constraint above. Do not encode 240x135. It
  must match `source_width` x `source_height`, and both must be multiples of 16.
- `-pix_fmt yuv420p` - the decoder's only output format is I420.
- `repeat-headers=1` - SPS/PPS before every IDR.
- `annexb=1` - start codes, which the sender strips per-NAL and this component
  restores. Do not send AVCC/length-prefixed NALs.
- A short GOP (`-g 10`, one second at 10fps) so a lost IDR costs one second of
  video rather than the whole stream.

For the 800x480 wall panel the same command with `-level 2.1 -s 480x288` feeds
`source_width: 480`, `source_height: 288`. Encode 480x288 and let the component
enlarge to 800x480 rather than encoding 800x480: the decode cost is roughly the
pixel count, and a 480x288 P-frame is about a third of an 800x480 one.

Split each NAL across datagrams so that header + payload never exceeds 1400
bytes.

## Configuration

```yaml
external_components:
  - source: github://taylorcoffelt/esphome-2Way-INTERCOM
    components: [h264_video]

h264_video:
  id: video
  port: 12347        # optional, default 12347
  frame_timeout: 3s  # optional, default 3s
```

Every option is optional and the defaults reproduce the original hard-wired
240x144 -> 240x135 behaviour exactly, so an existing config needs no changes.

| Option           | Default | Meaning                                                                     |
|------------------|---------|-----------------------------------------------------------------------------|
| `port`           | 12347   | UDP port. One port is one stream - nothing in the wire protocol identifies it. |
| `frame_timeout`  | 3s      | How long the last published frame counts as live before `is_stale()`.        |
| `source_width`   | 240     | Encoded frame width. **Must be a multiple of 16.**                           |
| `source_height`  | 144     | Encoded frame height, macroblock-aligned. **Must be a multiple of 16.**      |
| `visible_height` | 135     | How many decoded rows are picture rather than padding. Must be <= `source_height`. |
| `output_width`   | 240     | Width of the RGB565 buffer handed to LVGL.                                   |
| `output_height`  | 135     | Height of that buffer. Scaling happens when output differs from source x visible. |

Both alignment rules are enforced at config time and fail the build with an
explanatory message, because neither failure is visible at runtime.

### Worked example: M5Stack StickS3, 240x135 ST7789

The stream is encoded 240x144 to satisfy macroblock alignment; the bottom nine
rows are padding and are cropped. Output matches the visible source, so no
scaling happens and the converter runs its original 1:1 loop.

```yaml
h264_video:
  id: video
  port: 12347
  source_width: 240     # encoded width
  source_height: 144    # encoded height, 16-aligned
  visible_height: 135   # the crop - the panel is 240x135
  output_width: 240
  output_height: 135
```

That is the default configuration, so in practice it can be written as just:

```yaml
h264_video:
  id: video
```

`dump_config()` reports:

```
  Source: 240x144 decoded (135 visible row(s), 9 cropped)
  Output: 240x135 RGB565 (1:1, no scaling)
```

### Worked example: 800x480 RGB wall panel

The stream is encoded 480x288 - both axes already 16-aligned, so nothing is
cropped - and the converter enlarges it to the panel's 800x480 while converting.

```yaml
h264_video:
  id: video
  port: 12347
  source_width: 480     # encoded width
  source_height: 288    # encoded height, 16-aligned
  visible_height: 288   # no crop needed, 288 is already aligned
  output_width: 800     # the panel
  output_height: 480
```

`dump_config()` reports:

```
  Source: 480x288 decoded (288 visible row(s), 0 cropped)
  Output: 800x480 RGB565 (scaling 480x288 -> 800x480, nearest neighbour)
```

Note the cost: two 800x480 RGB565 framebuffers are 1,536,000 bytes of PSRAM
against 129,600 for the small panel. See **Memory** below.

`esp_h264` is pulled in automatically as a managed IDF component
(`espressif/esp_h264`, `^1.3.8`) - there is nothing to add to
`idf_components.yaml`.

## Actions

```yaml
- h264_video.start:
    id: video

- h264_video.stop:
    id: video
```

The decoder claims roughly a megabyte of PSRAM and holds it for as long as it
exists, so it is allocated lazily on `start()` and released on `stop()`. Leave
the receiver stopped when the video screen is not on show.

## LVGL usage pattern

**LVGL is not thread-safe and this component never calls into it.** The decode
task only converts and publishes; every `lv_*` call below happens on the
ESPHome main loop, which is where LVGL is driven from.

Three methods make up the handoff:

- `get_frame()` - the currently published `lv_img_dsc_t *` (RGB565,
  `output_width` x `output_height`), or `nullptr` before the first decode
- `take_new_frame()` - true exactly once per newly published frame
- `is_stale()` - true when nothing has arrived within `frame_timeout`

`get_output_width()` and `get_output_height()` return the published geometry if a
lambda needs it; the `lv_img_dsc_t` header already carries it.

The returned descriptor pointer alternates between two buffers as frames are
published. That matters: LVGL keys its image cache off the source pointer, so an
alternating `src` is never mistaken for an unchanged one. Call
`lv_obj_invalidate()` after setting it - the pixel data behind a given
descriptor changes in place, and only the invalidate tells LVGL to redraw.

```yaml
image:
  # LVGL needs some src at codegen time; a 1x1 placeholder is enough, the
  # descriptor is swapped for a live frame at runtime.
  - file: mdi:video
    id: video_placeholder
    resize: 1x1

lvgl:
  pages:
    - id: video_page
      widgets:
        - image:
            id: video_img
            src: video_placeholder
            align: CENTER

interval:
  # 30ms, not 100ms: this only paints when a frame actually arrived, and polling
  # faster than the 10fps stream keeps the display latency below one frame time.
  - interval: 30ms
    then:
      - lambda: |-
          if (id(video).take_new_frame()) {
            const lv_img_dsc_t *frame = id(video).get_frame();
            if (frame != nullptr) {
              lv_img_set_src(id(video_img), frame);
              lv_obj_invalidate(id(video_img));
            }
          }
```

Staleness and diagnostics from the same main-loop context:

```yaml
- lambda: |-
    if (id(video).is_stale()) {
      ESP_LOGW("video", "no frame in %u ms", 3000);
    }
    ESP_LOGD("video", "nals %u (%u dropped), frames %u, errors %u, last decode %u ms",
             id(video).get_nals_received(),
             id(video).get_nals_dropped(),
             id(video).get_frames_decoded(),
             id(video).get_decode_errors(),
             id(video).get_last_decode_ms());
```

## Threading and priorities

| Task            | Core | Priority | Notes                                  |
|-----------------|------|----------|----------------------------------------|
| `i2s_audio_udp` | 1    | 19       | Hard I2S deadlines, must always win    |
| `h264_video`    | 1    | 5        | A late frame is invisible at 10fps     |

A decode is tens of milliseconds. Running it anywhere near the audio task's
priority would push audio past its DMA refill window and produce an audible
dropout; running it on the ESPHome main loop would trip loop-time warnings and
starve the LVGL redraw it exists to feed.

`stop()` joins the decode task on a binary semaphore that the task gives on
every exit path, with a 2s bound. It deliberately does **not** poll
`eTaskGetState()`: a self-deleted task's TCB is reaped almost immediately, so
that poll reads freed heap and its timeout becomes the normal path rather than
the exceptional one. The task frees nothing - `stop()` owns the socket, the
buffers and the decoder, and tears them down only after the join.

## Memory

Free internal heap and free PSRAM are logged at INFO after
`esp_h264_dec_sw_new` and again after the first successful decode. The second
number is the one that matters: the decoder's DPB and reference frames are not
fully populated until a picture has been through it.

Roughly, per running receiver:

| Allocation                | Where  | Size                                     |
|---------------------------|--------|------------------------------------------|
| Decoder (DPB + workspace) | PSRAM  | ~1 MB                                    |
| 2x RGB565 framebuffer     | PSRAM  | 2 x `output_width` x `output_height` x 2 |
| NAL reassembly buffer     | PSRAM  | 32 KB                                    |
| Scale tables              | IRAM   | (`output_width` + `output_height`) x 2 B, only when scaling |
| Datagram receive buffer   | IRAM   | 1,400 B                                  |
| Task stack                | IRAM   | 8 KB                                     |

The framebuffers are the term that moves with geometry:

| Output    | Per buffer | Both buffers |
|-----------|------------|--------------|
| 240x135   | 64,800 B   | 129,600 B    |
| 800x480   | 768,000 B  | 1,536,000 B  |

The scale tables are small - 2,560 bytes for an 800x480 output - and are the one
allocation deliberately kept in internal RAM rather than PSRAM: they are read
once per output pixel, which is the only access in the converter that is not a
long forward sweep.
