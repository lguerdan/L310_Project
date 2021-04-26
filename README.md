# L310 Project Repository

This repository contains code used to for the L310 Multi-Car Consensus Final Project.
- A video demonstrating the approach can be found [here](https://drive.google.com/file/d/1GWeQMgfHg4Q-loS0FVRbkIEhZfwMnKV_/view?usp=sharinghere)
- `utils.py` contains utility functions (not all were used for final analysis)
- `baseline_analysis.ipynb` contains experiment analysis code

The `flow` repository contains a fork of the Flow traffic simulator publically available [here](https://github.com/flow-project/flow).
- `flow/expamples/exp_simulate.py` contains the main driver function for loading the `exp_ring.py` and `exp_figure_eight.py` configuratinos.  Several of the experiment configurations are commented out in this file for reference.

- `flow/flow/controllers/car_following_models.py` contains implementations of the evaluated baseline and consensus controllers. The interface for these classes extends the `BaseController` and takes many of the same parameters as the `IDMController` (provided for reference).
