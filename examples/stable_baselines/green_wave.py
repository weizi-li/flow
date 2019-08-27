"""Grid/green wave example."""

import argparse
import json
import os

from stable_baselines.common.vec_env import DummyVecEnv, SubprocVecEnv
from stable_baselines import PPO2

from flow.core.params import SumoParams, EnvParams, InitialConfig, NetParams, \
    SumoCarFollowingParams, InFlows
from flow.core.params import VehicleParams
from flow.controllers import SimCarFollowingController, GridRouter
from flow.utils.registry import env_constructor
from flow.utils.rllib import FlowParamsEncoder, get_flow_params

# time horizon of a single rollout
HORIZON = 200


def run_model(num_cpus=1, rollout_size=50, num_steps=50, use_inflows=False):
    """Run the model for num_steps if provided. The total rollout length is rollout_size."""
    initial_config, net_params = setup_exps(use_inflows)
    # add the new parameters to flow_params
    flow_params['initial'] = initial_config
    flow_params['net'] = net_params

    if num_cpus == 1:
        constructor = env_constructor(params=flow_params, version=0)()
        env = DummyVecEnv([lambda: constructor])  # The algorithms require a vectorized environment to run
    else:
        env = SubprocVecEnv([env_constructor(params=flow_params, version=i) for i in range(num_cpus)])

    model = PPO2('MlpPolicy', env, verbose=1, n_steps=rollout_size)
    model.learn(total_timesteps=num_steps)
    return model


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--num_cpus', type=int, default=1, help='How many CPUs to use')
    parser.add_argument('--num_steps', type=int, default=5000, help='How many total steps to perform learning over')
    parser.add_argument('--rollout_size', type=int, default=1000, help='How many steps are in a training batch.')
    parser.add_argument('--result_name', type=str, default='green_wave', help='Name of saved model')
    parser.add_argument('--use_inflows', action='store_true')
    args = parser.parse_args()
    model = run_model(args.num_cpus, args.rollout_size, args.num_steps, args.use_inflows)
    # Save the model to a desired folder and then delete it to demonstrate loading
    if not os.path.exists(os.path.realpath(os.path.expanduser('~/baseline_results'))):
        os.makedirs(os.path.realpath(os.path.expanduser('~/baseline_results')))
    path = os.path.realpath(os.path.expanduser('~/baseline_results'))
    save_path = os.path.join(path, args.result_name)
    # dump the model
    model.save(save_path)
    # dump the flow params
    with open(os.path.join(path, args.result_name) + '.json', 'w') as outfile:
        json.dump(flow_params, outfile,  cls=FlowParamsEncoder, sort_keys=True, indent=4)
    del model
    del flow_params

    # Replay the result by loading the model
    print('Loading the trained model and testing it out!')
    model = PPO2.load(save_path)
    flow_params = get_flow_params(os.path.join(path, args.result_name) + '.json')
    flow_params['sim'].render = True
    env_constructor = env_constructor(params=flow_params, version=0)()
    env = DummyVecEnv([lambda: env_constructor])  # The algorithms require a vectorized environment to run
    obs = env.reset()
    reward = 0
    for i in range(flow_params['env'].horizon):
        action, _states = model.predict(obs)
        obs, rewards, dones, info = env.step(action)
        reward += rewards
    print('the final reward is {}'.format(reward))