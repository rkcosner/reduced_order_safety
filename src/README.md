# Reduced Order Controller and Safety Filter Stack 
This code was the reduced-order controller and safety filter stack for both the simulation and hardware experiments presented in the 2022 L4DC paper entitled "Safety-Aware Preferecne-Based Learning for Safety-Critical Control" authored by Ryan K. Cosner, Maegan Tucker, Andrew J. Taylor, Kejun Li, Tamas G. Molnar, Wyatt Ubellacker, Anil Alan, Gabor Orosz, Yisong Yue, and Aaron D. Ames. 

# Set Up 

This code is meant to run as a ROS package inside of a catkin workspace 

# Dependencies
To run this code in general you need Wyatt Ubellacker's *mpac_a1* Unitree A1 simulator 

In order to run this code to run in simulation you will need the following dependencies: 
    * 

In order for this code to run on hardware you will need the following dependencies: 
    * ROS *occupancy package* which is a modified version of Intel's SLAM package and can be found here: https://github.com/rkcosner/occupancy.git
    * ROS *realsesense2_camera* package


# Running 
To run the simulation you should run the code: 
    * *unitree_sim.launch*

The parameters to run this are the Tunable-Robustified Optimization Program (TR-OP) parameters (A, B, C, D) --> (a, b, phi, alpha). 