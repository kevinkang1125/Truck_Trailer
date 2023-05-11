from gym.envs.registration import register
from pkg_resources import EntryPoint

register(
    id = 'TruckTrailer-v1',
    entry_point = 'truck_trailer.envs:TruckTrailerEnv',
    #max_episode_steps = 10000
)