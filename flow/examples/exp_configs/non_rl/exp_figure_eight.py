"""Example of a figure 8 network with human-driven vehicles.

Right-of-way dynamics near the intersection causes vehicles to queue up on
either side of the intersection, leading to a significant reduction in the
average speed of vehicles in the network.
"""
from flow.controllers import IDMController, StaticLaneChanger, ContinuousRouter
from flow.controllers.car_following_models import BaselineController, ConsensusFailureController, FigureEightController
from flow.core.params import SumoParams, EnvParams, NetParams
from flow.core.params import VehicleParams, SumoCarFollowingParams
from flow.envs.ring.accel import ADDITIONAL_ENV_PARAMS
from flow.networks.figure_eight import ADDITIONAL_NET_PARAMS
from flow.envs import AccelEnv
from flow.networks import FigureEightNetwork


DIAMETER = 200

def get_figeight_params(
    N_VEHICLES=20,
    MAX_ACC=2,
    MAX_DEC=2,
    MAX_SPEED=30,
    TARGET_VELOCITY=30,
    CONTROLLER='baseline',
    NAME='figure_eight',
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
        'radius_ring': DIAMETER/4,
    })

    vehicles = VehicleParams()
    vehicles.add(
        veh_id="idm",
        acceleration_controller=(FigureEightController, {
            'v0':TARGET_VELOCITY,
            'a': MAX_ACC,
            'c_acceleration': 0.00,
            'c_velocity': 0.0,
            'c_headway': 0.001
        }),
        lane_change_controller=(StaticLaneChanger, {}),
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
        network=FigureEightNetwork,

        # simulator that is used by the experiment
        simulator='traci',

        # sumo-related parameters (see flow.core.params.SumoParams)
        sim=SumoParams(
            render=True,
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
    )
    return flow_params
