For Prithvi's Experiments: 

Connections: 
optitrack --> windows computer --> laptop --> unitree


Set Up: 
This laptop should be on the same wifi network as the windows lab computer, probably caltechsecure. Then make sure that the ip address in the optitrack.launch file is correct

Windows computer set up: 
In motive, set the 3 robots to have the rigid body names: 
    - duckie1
    - duckie2
    - Unitree
    make sure that it's publishing correcly. Follow [this](https://tuw-cpsg.github.io/tutorials/optitrack-and-ros/) tutorial if optitrack isn't working. 

Test: 
    If you run optitrack.launch, you should connect with the windows computer and you should receive which publish at >100Hz:
    - /vrpn_client_node/Unitree/pose
    - /vrpn_client_node/duckie1/pose
    - /vrpn_client_node/duckie2/pose


Command to run: 
roslaunch red_ord_unitree unitree_indoors.launch A:=1 B:=0.1 C:=0.1 D:=0.1


Command to view inputs: 
rostopic echo /cmd



