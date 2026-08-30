"""
H.264 Video Component for ESPHome

Receives Annex-B NAL units over UDP, decodes them with Espressif's esp_h264
software decoder (tinyh264), and publishes RGB565 frames for LVGL to draw.

Decoding happens on its own FreeRTOS task. It is deliberately not a Component
loop() job: a single 240x144 P-frame costs tens of milliseconds on the S3, which
would trip ESPHome's loop-time warnings and starve the LVGL redraw it is meant
to be feeding.
"""
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.components import esp32
from esphome.const import CONF_ID, CONF_PORT

CODEOWNERS = ["@n-IA-hane"]
DEPENDENCIES = ["esp32", "network"]

CONF_FRAME_TIMEOUT = "frame_timeout"

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
        }
    ).extend(cv.COMPONENT_SCHEMA),
    cv.only_with_esp_idf,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_port(config[CONF_PORT]))
    cg.add(var.set_frame_timeout(config[CONF_FRAME_TIMEOUT].total_milliseconds))

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
