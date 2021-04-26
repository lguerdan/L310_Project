"""Used as an example of ring experiment.

This example consists of 22 IDM cars on a ring creating shockwaves.
"""

from flow.controllers import IDMController, ContinuousRouter
from flow.controllers.car_following_models import IDMController, ConsensusController, BaselineController
from flow.core.params import SumoParams, EnvParams, InitialConfig, NetParams
from flow.core.params import VehicleParams, SumoCarFollowingParams
from flow.envs.ring.accel import AccelEnv, ADDITIONAL_ENV_PARAMS
from flow.networks.ring import RingNetwork, ADDITIONAL_NET_PARAMS


# Environment Hyperparameters
DIAMETER = 200

def get_ring_params(
    N_VEHICLES=20,
    MAX_ACC=2,
    MAX_DEC=2,
    MAX_SPEED=30,
    TARGET_VELOCITY=30,
    CONTROLLER='baseline',
    NAME='ring',
    c_headway=0.01,
    c_velocity=0.01,
    c_acceleration=0.001):

    ADDITIONAL_ENV_PARAMS.update({
        'max_accel': MAX_ACC,
        'max_decel': MAX_DEC,
        'max_speed': MAX_SPEED,
        'target_velocity': TARGET_VELOCITY, 
    })
    ADDITIONAL_NET_PARAMS.update({
        'length': 3.14159 * DIAMETER
    })

    vehicles = VehicleParams()

    if CONTROLLER=='baseline':
        acc_controller = (BaselineController, {
        'v0':TARGET_VELOCITY,
        'a': MAX_ACC,
        })
    elif CONTROLLER=='consensus':
        acc_controller = (ConsensusController, {
        'v0':TARGET_VELOCITY,
        'a': MAX_ACC,
        'c_acceleration': c_acceleration,
        'c_velocity': c_velocity,
        'c_headway': c_headway
    })

    vehicles.add(
        veh_id="idm",
        acceleration_controller=acc_controller,
        routing_controller=(ContinuousRouter, {}),
        car_following_params=SumoCarFollowingParams(
            speed_mode="aggressive",
            accel=MAX_ACC,
            decel=MAX_DEC,
            max_speed=MAX_SPEED,
        ),
        initial_speed=0,
        num_vehicles=N_VEHICLES)

    flow_params = dict(
        # name of the experiment
        exp_tag=NAME,

        # name of the flow environment the experiment is running on
        env_name=AccelEnv,

        # name of the network class the experiment is running on
        network=RingNetwork,

        # simulator that is used by the experiment
        simulator='traci',

        # sumo-related parameters (see flow.core.params.SumoParams)
        sim=SumoParams(
            render=True,
            sim_step=0.1,
        ),

        # environment related parameters (see flow.core.params.EnvParams)
        env=EnvParams(
            horizon=1500,
            additional_params=ADDITIONAL_ENV_PARAMS.copy(),
        ),

        # network-related parameters (see flow.core.params.NetParams and the
        # network's documentation or ADDITIONAL_NET_PARAMS component)
        net=NetParams(
            additional_params=ADDITIONAL_NET_PARAMS.copy(),
        ),

        # vehicles to be placed in the network at the start of a rollout (see
        # flow.core.params.VehicleParams)
        veh=vehicles,

        # parameters specifying the positioning of vehicles upon initialization/
        # reset (see flow.core.params.InitialConfig)
        initial=InitialConfig(
            bunching=20,
        ),
    )
    return flow_params
