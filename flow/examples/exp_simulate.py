"""Runner script for non-RL simulations in flow.

Usage
    python simulate.py EXP_CONFIG --no_render
"""
import argparse
import sys
import json
import os
from flow.core.experiment import Experiment

from flow.core.params import AimsunParams
from flow.utils.rllib import FlowParamsEncoder

from exp_configs.non_rl.exp_ring import get_ring_params
from exp_configs.non_rl.exp_figure_eight import get_figeight_params


def parse_args(args):
    """Parse training options user can specify in command line.

    Returns
    -------
    argparse.Namespace
        the output parser object
    """
    parser = argparse.ArgumentParser(
        description="Parse argument used when running a Flow simulation.",
        epilog="python simulate.py EXP_CONFIG --num_runs INT --no_render")

    # required input parameters
    parser.add_argument(
        'exp_config', type=str,
        help='Name of the experiment configuration file, as located in '
             'exp_configs/non_rl.')

    # optional input parameters
    parser.add_argument(
        '--num_runs', type=int, default=1,
        help='Number of simulations to run. Defaults to 1.')
    parser.add_argument(
        '--no_render',
        action='store_true',
        help='Specifies whether to run the simulation during runtime.')
    parser.add_argument(
        '--aimsun',
        action='store_true',
        help='Specifies whether to run the simulation using the simulator '
             'Aimsun. If not specified, the simulator used is SUMO.')
    parser.add_argument(
        '--gen_emission',
        action='store_true',
        help='Specifies whether to generate an emission file from the '
             'simulation.')

    return parser.parse_known_args(args)[0]



    # exp_name = 'acc_baseline'
    # for acc in [.5,1,1.5,2,2.5,3,3.5,4]:
    #     exp_kwargs = {'MAX_DEC': acc, 'MAX_ACC': acc, 'NAME' : f'{flags.exp_config}_{exp_name}_acc_{acc}_r2', 'N_VEHICLES':20, 'TARGET_VELOCITY': 30, 'MAX_SPEED': 30}

    # exp_name = 'velocity_baseline'
    # for vel in range(5,35,5):
    #     exp_kwargs = {'TARGET_VELOCITY': vel, 'MAX_SPEED': vel, 'NAME' : f'{flags.exp_config}_{exp_name}_vel_{vel}_r2', 'N_VEHICLES':20, 'MAX_DEC': 2, 'MAX_ACC': 2 }

    ############# Hyperparam experiment
    # exp_name = 'consensus_constants'
    # for ch in [0, .0001, .001, .01, .05]:
    #     for cv in [0, .0001, .001,  .01]:
    #         for ca in  [0, .0001, .001, .01]:

    #             exp_kwargs = {
    #                 'CONTROLLER': 'consensus',
    #                 'N_VEHICLES': 16,
    #                 'NAME' : f'{flags.exp_config}_{exp_name}_cars_16_ch_{ch}_cv_{cv}_ca_{ca}',
    #                 'TARGET_VELOCITY': 30,
    #                 'MAX_SPEED': 30,
    #                 'MAX_DEC': 2,
    #                 'MAX_ACC': 2,
    #                 'c_acceleration':ca,
    #                 'c_velocity': cv,
    #                 'c_headway':ch
    #             }
    ##################################

    ############# Controller Comparisons 
    # exp_name = 'concensus_basline_velocity'
    # for vel in range(10,41,4):
    #     for controller in ['consensus', 'baseline']:
    #         exp_kwargs = {
    #             'TARGET_VELOCITY': vel,
    #             'MAX_SPEED': vel,
    #             'NAME' : f'{flags.exp_config}_{exp_name}_vel_{vel}_controller_{controller}',
    #             'N_VEHICLES':16,
    #             'MAX_DEC': 2,
    #             'MAX_ACC': 2,
    #             'CONTROLLER': controller
    #         }
    # exp_name = 'concensus_basline_velocity'
    # for ncars in range(4,22,2):
    #     for controller in ['consensus', 'baseline']:
    #         exp_kwargs = {
    #             'TARGET_VELOCITY': 30,
    #             'MAX_SPEED': 30,
    #             'NAME' : f'{flags.exp_config}_{exp_name}_num_cars_{ncars}_controller_{controller}',
    #             'N_VEHICLES': ncars,
    #             'MAX_DEC': 2,
    #             'MAX_ACC': 2,
    #             'CONTROLLER': controller
    #         }
    # exp_name = 'concensus_basline_velocity'
        # for acc in [1, 2, 3, 4]:
        # for controller in ['consensus', 'baseline']:
        #     exp_kwargs = {
        #         'TARGET_VELOCITY': 30,
        #         'MAX_SPEED': 30,
        #         'NAME' : f'{flags.exp_config}_{exp_name}_acc_{acc}_controller_{controller}',
        #         'N_VEHICLES': 16,
        #         'MAX_DEC': acc,
        #         'MAX_ACC': acc,
        #         'CONTROLLER': controller
        #     }

            ############# Controller Comparisons 
    # exp_name = 'concensus_basline_velocity'
    # for ncars in range(4,22,2):
    #     for controller in ['consensus', 'baseline']:
    #         exp_kwargs = {
    #             'TARGET_VELOCITY': 30,
    #             'MAX_SPEED': 30,
    #             'NAME' : f'{flags.exp_config}_{exp_name}_num_cars_{ncars}_controller_{controller}',
    #             'N_VEHICLES': ncars,
    #             'MAX_DEC': 2,
    #             'MAX_ACC': 2,
    #             'CONTROLLER': controller
    #         }

if __name__ == "__main__":
    flags = parse_args(sys.argv[1:])

    ############# Controller Comparisons 
    exp_name = 'figure_eight_concensus'

    exp_kwargs = {
        'TARGET_VELOCITY': 15,
        'MAX_SPEED': 15,
        'NAME' : f'{flags.exp_config}_{exp_name}_FECT',
        'N_VEHICLES': 12,
        'MAX_DEC': 1,
        'MAX_ACC': 1,
        'CONTROLLER': 'consensus'
    }

    # Get the flow_params object.
    module = __import__("exp_configs.non_rl", fromlist=[flags.exp_config])
    flow_params = get_ring_params(**exp_kwargs) if flags.exp_config == 'exp_ring' else get_figeight_params(**exp_kwargs)

    # Get the custom callables for the runner.
    if hasattr(getattr(module, flags.exp_config), "custom_callables"):
        callables = getattr(module, flags.exp_config).custom_callables
    else:
        callables = None

    flow_params['sim'].render = not flags.no_render
    flow_params['simulator'] = 'aimsun' if flags.aimsun else 'traci'

    # If Aimsun is being called, replace SumoParams with AimsunParams.
    if flags.aimsun:
        sim_params = AimsunParams()
        sim_params.__dict__.update(flow_params['sim'].__dict__)
        flow_params['sim'] = sim_params

    # Specify an emission path if they are meant to be generated.
    if flags.gen_emission:
        flow_params['sim'].emission_path = "./data"

        # Create the flow_params object
        fp_ = flow_params['exp_tag']
        dir_ = flow_params['sim'].emission_path
        with open(os.path.join(dir_, "{}.json".format(fp_)), 'w') as outfile:
            json.dump(flow_params, outfile,
                    cls=FlowParamsEncoder, sort_keys=True, indent=4)

    # Create the experiment object.
    exp = Experiment(flow_params, callables)

    # Run for the specified number of rollouts.
    exp.run(flags.num_runs, convert_to_csv=flags.gen_emission)
