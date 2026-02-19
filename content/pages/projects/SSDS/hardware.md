+++
title = "Testbed Hardware"
date = 2026-02-18
template = "page.html"
+++

Testbed structure, mass balancers, propulsion systems, thrusters and reaction wheels. <!-- more -->

## Overall Structure

![](/SSDS/structure.png)

The testbed weighs around 150 kg and has a battery capacity for at least 2 hours of self-operation. It features attitude control about the 3 axes of rotation for a hemispherical boundary (+- 45 degrees). 
Its geometry consists of aluminum extrusions, with an inner and outer octagon structure. This shape allows a wider field of view for the star tracker, as well as placement for the propulsion systems (6 thrusters and 3 reaction wheels), since they can be placed on any axis. External payloads are also easily installed.

The lift system on the lower half features two columns with linear actuators (an external static tower and an internal dynamic tower). A bumper limits the range of motion to ensure the spacecraft does not contact the lift system and prevent over-rotation. The satellite must rest on the lift system when not running to preserve the air-bearing:

![](/SSDS/bearing.png)

An incompressible layer of air is formed when operated, allowing the testbed to rotate with low friction.

## Mass Balancers

<video src="/SSDS/balancing.mp4" loop controls width="400"></video>

The system consists of 3 mass balancing sliders designed to keep the CoM and CoR as close to each other.  

The mass balancing algorithm is performed by the MATLAB/Simulink software, sending position commands to the testbed's onboard Arduino. Stepper motors are used to carry out the precise position commands.

The full setup includes operating the mass balancing algorithm multiple times with different attitude changes to ensure balance in all three axes. Afterwards, the mass balancers stay in their positions throughout operation.

## Propulsion System

The propulsion system consists of 6 thrusters and 3 reaction wheels.

The thrusters are controlled independently through solenoids and manifolds that distribute air to each, rated for 160 psi.

[The reaction wheels](https://www.pmwdynamics.com/product/gpn16lr/#) are rated for 1.5 Nm, and have a max speed of 3000 RPM. They are placed about each axis. 


<img src="/SSDS/rw.png" alt="rw" width="400"/>