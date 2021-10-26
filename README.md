# RExQuadruped


## Setting up on a Unitree Quadruped 
### Install Julia Cxxwrap 
Follow this instruction to install CxxWrap https://github.com/JuliaInterop/CxxWrap.jl
In particular, be careful not to skip this step to install libcxxwrap-julia https://github.com/JuliaInterop/libcxxwrap-julia

### Install Boost 
OS X: brew install boost 

### Install LCM (unitree sdk dependency )
LCM is actually not used here. but it is required for the unitree_sdk. It is usually installed on a UnitreeA1 quadruped. If it's not, follow this following instruction: https://lcm-proj.github.io/build_instructions.html

### Building the julia cxxwrap interface 
For some reason it will only work with the unitree_legged_sdk already installed on the robot. Currently the sdk in deps is not working. Chagne the path in  CMakelist.txt to make sure it's using the right one. 