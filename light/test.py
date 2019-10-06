from flow.simulation.ring import RingNetwork

name = "ring_example"

from flow.core.params import VehicleParams

vehicles = VehicleParams()

from flow.controllers.car_following_models import IDMController

from flow.controllers.routing_controllers import ContinuousRouter


vehicles.add("human",
             acceleration_controller=(IDMController, {}),
             routing_controller=(ContinuousRouter, {}),
             num_vehicles=22)

from flow.networks.ring import ADDITIONAL_NET_PARAMS

print(ADDITIONAL_NET_PARAMS)

from flow.core.params import NetParams

net_params = NetParams(additional_params=ADDITIONAL_NET_PARAMS)


from flow.core.params import InitialConfig

initial_config = InitialConfig(spacing="uniform", perturbation=1)


from flow.core.params import TrafficLightParams

traffic_lights = TrafficLightParams()

from flow.envs.ring.accel import AccelEnv

from flow.core.params import SumoParams

sumo_params = SumoParams(sim_step=0.1, render=True, emission_path='data')

from flow.envs.ring.accel import ADDITIONAL_ENV_PARAMS

print(ADDITIONAL_ENV_PARAMS)


from flow.core.params import EnvParams

env_params = EnvParams(additional_params=ADDITIONAL_ENV_PARAMS)

from flow.core.experiment import Experiment

network = RingNetwork(name="ring_example",
                      vehicles=vehicles,
                      net_params=net_params,
                      initial_config=initial_config,
                      traffic_lights=traffic_lights)

# create the environment object
env = AccelEnv(env_params, sumo_params, network)

# create the experiment object
exp = Experiment(env)

# run the experiment for a set number of rollouts / time steps
_ = exp.run(1, 3000, convert_to_csv=False)






