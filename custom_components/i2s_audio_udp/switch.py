"""
Switch platform for i2s_audio_udp component
Provides AEC (Echo Cancellation) on/off switch
"""
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch
from esphome.const import CONF_ID

from . import I2SAudioUDP, i2s_audio_udp_ns

CONF_I2S_AUDIO_UDP_ID = "i2s_audio_udp_id"
CONF_AEC = "aec"

I2SAudioUDPAecSwitch = i2s_audio_udp_ns.class_(
    "I2SAudioUDPAecSwitch", switch.Switch, cg.Component
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_I2S_AUDIO_UDP_ID): cv.use_id(I2SAudioUDP),
        cv.Optional(CONF_AEC): switch.switch_schema(
            I2SAudioUDPAecSwitch,
            icon="mdi:microphone-settings",
            entity_category="config",
        ),
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_I2S_AUDIO_UDP_ID])

    if CONF_AEC in config:
        conf = config[CONF_AEC]
        var = await switch.new_switch(conf)
        await cg.register_component(var, conf)
        cg.add(var.set_parent(parent))
