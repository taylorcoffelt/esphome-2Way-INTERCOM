"""
H.264 Video Component for ESPHome

Receives Annex-B NAL units over UDP, decodes them with Espressif's esp_h264
software decoder (tinyh264), and publishes RGB565 frames for LVGL to draw.

Decoding happens on its own FreeRTOS task. It is deliberately not a Component
loop() job: a single 240x144 P-frame costs tens of milliseconds on the S3, which
would trip ESPHome's loop-time warnings and starve the LVGL redraw it is meant
to be feeding.

Geometry is configurable because two very different panels use this component:
a 240x135 ST7789 fed a 240x144 stream 1:1, and an 800x480 RGB panel fed a
480x288 stream that the colour conversion enlarges on its way through. Every
option below defaults to the original hard-wired 240x144 -> 240x135 behaviour.
"""
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.components import esp32
from esphome.const import CONF_ID, CONF_PORT

CODEOWNERS = ["@n-IA-hane"]
DEPENDENCIES = ["esp32", "network"]

CONF_FRAME_TIMEOUT = "frame_timeout"
CONF_SOURCE_WIDTH = "source_width"
CONF_SOURCE_HEIGHT = "source_height"
CONF_VISIBLE_HEIGHT = "visible_height"
CONF_OUTPUT_WIDTH = "output_width"
CONF_OUTPUT_HEIGHT = "output_height"


def _macroblock_aligned(key):
    """Dimension of an encoded frame: positive and a multiple of 16.

    This is validated rather than merely documented because getting it wrong is
    silent. tinyh264 applies no cropping window and returns whole macroblocks,
    so a source height of, say, 135 does not fail to decode - it decodes to 144
    and every plane offset computed from 135 lands mid-row, which reaches the
    panel as a picture that shears further to one side with every line. There
    is no runtime symptom to trace back to the config, so the config is where
    it has to stop.
    """

    def validator(value):
        value = cv.int_range(min=16, max=4096)(value)
        if value % 16 != 0:
            aligned = (value + 15) // 16 * 16
            hint = (
                f" Encode at {aligned} and set visible_height to {value}."
                if key == CONF_SOURCE_HEIGHT
                else f" Encode at {aligned}."
            )
            raise cv.Invalid(
                f"{key} must be a multiple of 16 (got {value}). The H.264 decoder "
                f"returns macroblock-aligned frames and applies no cropping window, "
                f"so an unaligned source does not fail - it decodes to the aligned "
                f"size and every row after the first lands offset, which reaches the "
                f"panel as a sheared picture with nothing to trace it back to."
                + hint
            )
        return value

    return validator


def _validate_geometry(config):
    if config[CONF_VISIBLE_HEIGHT] > config[CONF_SOURCE_HEIGHT]:
        raise cv.Invalid(
            f"visible_height ({config[CONF_VISIBLE_HEIGHT]}) cannot exceed "
            f"source_height ({config[CONF_SOURCE_HEIGHT]}). visible_height is the "
            f"crop: how many of the decoded rows are picture rather than the "
            f"padding the encoder added to reach macroblock alignment.",
            path=[CONF_VISIBLE_HEIGHT],
        )
    return config


h264_video_ns = cg.esphome_ns.namespace("h264_video")
H264Video = h264_video_ns.class_("H264Video", cg.Component)

StartAction = h264_video_ns.class_("StartAction", automation.Action)
StopAction = h264_video_ns.class_("StopAction", automation.Action)

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(H264Video),

            # Matches the sender's default fan-out port. Nothing in the wire
            # protocol identifies the stream, so one port is one stream.
            cv.Optional(CONF_PORT, default=12347): cv.port,

            # How long the last published frame stays "live". A lossy UDP link
            # that has simply gone quiet looks exactly like a healthy one until
            # something notices the silence, so the UI needs this to tell the
            # difference between "paused on the last frame" and "stale".
            cv.Optional(CONF_FRAME_TIMEOUT, default="3s"): cv.positive_time_period_milliseconds,

            # The encoded frame size, which is what the decoder hands back.
            # Both axes must be macroblock-aligned - see _macroblock_aligned.
            cv.Optional(CONF_SOURCE_WIDTH, default=240): _macroblock_aligned(CONF_SOURCE_WIDTH),
            cv.Optional(CONF_SOURCE_HEIGHT, default=144): _macroblock_aligned(CONF_SOURCE_HEIGHT),

            # How many of those decoded rows are real picture. The default
            # stream is encoded 240x144 to satisfy the alignment above and only
            # its top 135 rows are shown; a stream whose height is already
            # aligned sets this equal to source_height.
            cv.Optional(CONF_VISIBLE_HEIGHT, default=135): cv.int_range(min=1, max=4096),

            # The RGB565 buffer handed to LVGL. Left equal to source_width x
            # visible_height this is a straight conversion; set larger (or
            # smaller) and the converter resamples in the same pass, which is
            # cheaper than letting LVGL scale the image again at draw time.
            cv.Optional(CONF_OUTPUT_WIDTH, default=240): cv.int_range(min=1, max=4096),
            cv.Optional(CONF_OUTPUT_HEIGHT, default=135): cv.int_range(min=1, max=4096),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    _validate_geometry,
    # NOT cv.only_with_esp_idf - that helper does not exist in ESPHome 2026.8.1
    # and the AttributeError fires at module import, i.e. before a single line
    # of the user's YAML is validated, so the failure looks like a broken config
    # rather than a broken component.
    cv.only_with_framework("esp-idf"),
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_port(config[CONF_PORT]))
    cg.add(var.set_frame_timeout(config[CONF_FRAME_TIMEOUT].total_milliseconds))
    cg.add(var.set_source_size(config[CONF_SOURCE_WIDTH], config[CONF_SOURCE_HEIGHT]))
    cg.add(var.set_visible_height(config[CONF_VISIBLE_HEIGHT]))
    cg.add(var.set_output_size(config[CONF_OUTPUT_WIDTH], config[CONF_OUTPUT_HEIGHT]))

    # The decoder is a managed IDF component, not vendored source. Pulling it in
    # from codegen rather than asking the user to hand-edit idf_components.yaml
    # keeps the dependency with the code that needs it, and the caret range lets
    # patch fixes land without a config change while pinning the ABI we compiled
    # against (esp_h264_dec_sw_new / the consume-loop process API).
    esp32.add_idf_component(name="espressif/esp_h264", ref="^1.3.8")


@automation.register_action(
    "h264_video.start",
    StartAction,
    cv.Schema({
        cv.GenerateID(): cv.use_id(H264Video),
    }),
)
async def start_action_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var


@automation.register_action(
    "h264_video.stop",
    StopAction,
    cv.Schema({
        cv.GenerateID(): cv.use_id(H264Video),
    }),
)
async def stop_action_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var
