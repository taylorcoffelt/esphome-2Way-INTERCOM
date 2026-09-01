# I2S Audio UDP Component

Bidirectional audio streaming over UDP with I2S hardware support for ESP32.

## Features

- **Auto-deduced modes**: I2S bus mode (single/dual) and audio mode (TX/RX/Full Duplex) automatically determined from pin configuration
- **Templatable network config**: `remote_ip`, `remote_port`, `listen_port` evaluated at start() time
- **Flexible microphone support**: Configurable bit width (16/32), channel (left/right), and gain (1-16x)
- **Software volume control**: Built-in volume adjustment
- **AEC integration**: Optional link to `esp_aec` component for echo cancellation
- **Platform sensors**: TX/RX packet counters, audio mode text sensor, volume number
- **Monitor-only mode**: `monitor_only: true` drives a level/waveform display on a device with no audio hardware at all - see below

## Configuration

```yaml
i2s_audio_udp:
  id: audio_bridge

  # Single bus (ES8311 style)
  i2s_lrclk_pin: 8
  i2s_bclk_pin: 18
  i2s_mclk_pin: 16
  i2s_din_pin: 17
  i2s_dout_pin: 15

  # OR Dual bus (INMP441 + MAX98357A style)
  mic_din_pin: 4
  mic_bclk_pin: 5
  mic_lrclk_pin: 6
  speaker_dout_pin: 15
  speaker_bclk_pin: 16
  speaker_lrclk_pin: 17

  # Mic configuration (dual bus only)
  mic_bits_per_sample: 32  # 16 or 32
  mic_channel: left        # left or right
  mic_gain: 4              # 1-16

  # Optional speaker enable
  speaker_enable_pin: 48

  # Network - can use lambdas!
  remote_ip: "192.168.1.10"
  remote_port: 12345
  listen_port: 12346

  # Or with templates:
  remote_ip: !lambda 'return id(target_ip).state;'

  # Levels only, no audio hardware (see "Monitor-only mode" below)
  monitor_only: false

  # Optional AEC
  aec_id: my_aec

  # Triggers
  on_start:
    - logger.log: "Started"
  on_stop:
    - logger.log: "Stopped"
  on_error:
    - logger.log:
        format: "Error: %s"
        args: [error.c_str()]
```

## Monitor-only mode

```yaml
i2s_audio_udp:
  id: audio_levels
  monitor_only: true

  # No I2S pins. None are read, so none are required.
  remote_ip: "192.168.1.10"   # still required; see note below
  remote_port: 12345
  listen_port: 12346
```

For a device that wants the **waveform without the audio**: a wall panel that
should show what the intercom is carrying, but has no codec, no speaker, and -
with an RGB display already using most of its GPIOs - no pins to spare for
clocking an I2S bus into nothing.

With `monitor_only: true` the component:

- **never touches I2S.** No channel is allocated in `setup()` or `start()`, no
  pins are configured, no codec is assumed. The I2S pin options stop being
  required; any that are left in the YAML are ignored.
- **receives, measures and discards.** The audio task binds `listen_port` as
  usual, reads each datagram as 16-bit little-endian mono PCM, takes the peak of
  the chunk and applies the same decay the speaker path applies, then throws the
  samples away. Nothing is buffered, because nothing would ever play it.
- **behaves identically everywhere else.** `start()`, `stop()`, `is_streaming()`,
  `on_start` / `on_stop` / `on_error`, `get_peak_level()`, `copy_levels()` and
  the RX packet counter all work exactly as they do with hardware attached, so
  the same waveform display code runs on both kinds of device.

`play_tone()` and `hold_i2s_open()` become no-ops in this mode - there is no
output to make a sound on and no channel to hold open. Each logs one warning the
first time it is called and then stays quiet.

The mode is fixed at build time from the YAML; it cannot be toggled at runtime.

**Note:** `remote_ip` and `remote_port` are still required by the schema even
though a monitor-only device never sends. Any valid address will do - the
component only parses it - but the keys have to be present.

Reading the levels from a display:

```yaml
- lambda: |-
    float levels[64];
    size_t n = id(audio_levels).copy_levels(levels, 64);  // oldest first
    // ... draw n points ...
```

## Actions

```yaml
# Start streaming
- i2s_audio_udp.start:
    id: audio_bridge

# Stop streaming
- i2s_audio_udp.stop:
    id: audio_bridge
```

## Sensors

```yaml
sensor:
  - platform: i2s_audio_udp
    i2s_audio_udp_id: audio_bridge
    tx_packets:
      name: "TX Packets"
    rx_packets:
      name: "RX Packets"

text_sensor:
  - platform: i2s_audio_udp
    i2s_audio_udp_id: audio_bridge
    audio_mode:
      name: "Audio Mode"

number:
  - platform: i2s_audio_udp
    i2s_audio_udp_id: audio_bridge
    volume:
      name: "Volume"
```

## Lambda Access

```yaml
- lambda: |-
    if (id(audio_bridge).is_streaming()) {
      ESP_LOGI("test", "TX: %d, RX: %d",
               id(audio_bridge).get_tx_packets(),
               id(audio_bridge).get_rx_packets());
    }
```
