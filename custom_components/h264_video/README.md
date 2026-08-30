# H.264 Video Component

Receives H.264 Annex-B NAL units over UDP, decodes them with Espressif's
`esp_h264` software decoder (tinyh264), and publishes RGB565 frames for LVGL.

Built for the M5Stack StickS3 (ESP32-S3-PICO-1-N8R8, 8MB octal PSRAM, 240x135
ST7789V) running alongside the `i2s_audio_udp` component.

## Features

- **UDP receive and fragment reassembly** on a configurable port (default 12347)
- **Software H.264 decode** on its own FreeRTOS task, core 1, priority 5
- **I420 to RGB565 conversion** in fixed-point integer math, 2x2 quad at a time
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

Reassembly is capped at 32KB per NAL, well above anything a 240x144 constrained
baseline IDR produces, which also bounds what a malformed fragment count can do.

## The macroblock-alignment constraint

**The decoder does not apply frame cropping.** It returns macroblock-aligned
frames and ignores the SPS cropping window entirely.

The display is 240x135. 135 is not a multiple of 16, so a 240x135 stream would
come back as 240x144 with nine rows of encoder padding that must not reach the
panel. The stream is therefore encoded **240x144** outright, and this component
converts only the top **135** rows.

Nothing here assumes 135 rows come back. The real geometry is read from the
decoder (`esp_h264_dec_get_resolution`) after every picture, checked against the
reported output size, and the conversion is clipped to whichever of the source
and the display is smaller.

## Encoder flags on the server side

The stream must be constrained baseline, level 1.3, 240x144, with SPS/PPS
repeated before every IDR so a receiver that joins late (or loses an IDR) can
recover without a signalling channel.

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
- `-s 240x144` - see the alignment constraint above. Do not encode 240x135.
- `-pix_fmt yuv420p` - the decoder's only output format is I420.
- `repeat-headers=1` - SPS/PPS before every IDR.
- `annexb=1` - start codes, which the sender strips per-NAL and this component
  restores. Do not send AVCC/length-prefixed NALs.
- A short GOP (`-g 10`, one second at 10fps) so a lost IDR costs one second of
  video rather than the whole stream.

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

- `get_frame()` - the currently published `lv_img_dsc_t *` (RGB565, 240x135), or
  `nullptr` before the first decode
- `take_new_frame()` - true exactly once per newly published frame
- `is_stale()` - true when nothing has arrived within `frame_timeout`

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

| Allocation                | Where  | Size          |
|---------------------------|--------|---------------|
| Decoder (DPB + workspace) | PSRAM  | ~1 MB         |
| 2x RGB565 framebuffer     | PSRAM  | 2 x 64,800 B  |
| NAL reassembly buffer     | PSRAM  | 32 KB         |
| Datagram receive buffer   | IRAM   | 1,400 B       |
| Task stack                | IRAM   | 8 KB          |
