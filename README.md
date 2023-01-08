# SAMP

Sequential Assembly Movement Primitive

A Robot Learning from Demonstration Framework for Assembly

Haopeng Hu

2022.08.11

![robot](https://github.com/philthinker/SAMP/blob/main/panda.jpg)

## Greengrape

The MATLAB codes for learning the policies from the demonstration data.

- The codes are tested on MATLAB R2020b.
- Run "UNCORK.m" once you set "Greengrape" as your MATLAB workspace.

### Demos

- main_A.m: Learning the policies of Object A.
- main_B.m: Learning the policies of Object B.
- main_C.m: Learning the policies of Object C.

Runing these codes may take seconds.

## Grape

The C++ codes for controlling the robot.

- The "libfranka 0.8.0" library is needed.
- There is an demo file "Toys_Grape6.cpp".
- Build it with CMake.

## Data

Demo data.

- "main_C.mat": The demo data for the MATLAB demo.
- "main_C.csv": The demo data for the C++ demo.