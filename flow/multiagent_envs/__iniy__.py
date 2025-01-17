"""Empty init file to ensure documentation for multi-agent envs is created."""

from flow.multiagent_envs.multiagent_env import MultiEnv
from flow.multiagent_envs.loop.wave_attenuation import \
    MultiWaveAttenuationPOEnv
from flow.multiagent_envs.loop.loop_accel import MultiAgentAccelEnv
from flow.multiagent_envs.traffic_light_grid import MultiTrafficLightGridPOEnv
from flow.multiagent_envs.highway import MultiAgentHighwayPOEnv
from flow.multiagent_envs.grid_avs_env import MultiGridAVsPOEnv

__all__ = [
    'MultiEnv',
    'MultiAgentAccelEnv',
    'MultiWaveAttenuationPOEnv',
    'MultiTrafficLightGridPOEnv',
    'MultiAgentHighwayPOEnv'
]
