+++
title = "Lab 11: Grid Localization (Real-World)"
date = 2026-04-21
template = "page.html"
+++

Real implementation of grid localization! <!-- more -->

## Prelab

The provided files were copied into the *notebook* and root directories:

- lab11_sim.ipynb: Pre-built optimized Bayes filter implementation for the virtual robot
- lab11_real.ipynb: Skeleton code for Bayes filter implementation on the real robot
- localization_extras.py: Pre-built localization module

The Bluetooth python modules from previous labs were copied: base_ble.py, ble.py, connection.yaml, and cmd_types.py. 

## Simulated Localization

The simulated localization was tested with *lab11_sim.ipynb*, and the final odometry readings (red), ground truth (green), and beliefs (blue) are shown below:

![Simulated localization](/lab11/simulated.png)

## Real Localization

The real localization is implemented in *lab11_real.ipynb*. Prediction steps are not done due to the noisy sensor measurements. Therefore, the Bayes filter only runs the update steps.

In the *RealRobot* class, the method *perform_observation_loop* commands the robot to perform an on-axis 360 degree turn while collecting equidistant sensor readings, similar to [Lab 9](lab9.md).
