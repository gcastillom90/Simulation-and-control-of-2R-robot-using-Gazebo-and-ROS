# Simulation  of 2R robot using Gazebo and ROS

A 2R robot is simulated using Gazebo and ROS considering different values of height and mass for each robot's link and different initial of the joints. The dynamic behavior of the robot and its response to gravity is analized for each different setup of kinematic and dynamic conditions.

## Simulation setup and files

The robot model was created from scratch using the URDF format, which requires the knowledge of the kinematic and dynamic properties of the robot (mass, intertia matrix, links geometry, etc). The inertia matrix entries were chosen according the geometry of the link, which is a rectangle of dimensions width, length and height. For the 2R robot, we have a square base; therefore, the width and length is the same, and height is also known. Then, the values of the inertia matrix are determined by standard equations for this kind og geometries.

There are three ROS packages: bot_description, bot_gazebo, bot_control. The package bot_description contains the URDF file corresponding to the model of the robot and its respective launch file, while the bot_gazebo package contains the world files, which are related to the simulation environment in which you will spawn your robot, and its respective launch files.

Once the simulation is running, I used a subscriber node called joints1_2_subscriber to listen to the topics showed below and obtain the data of the joint angle at current time. 
*	/bot/joint1_position_controller/state
*	/bot/joint2_position_controller/state 

The python file created is called joint_subscriber.py and it is launched with the joint_data.launch file. The angles’ data is saved in a text file, which is read by the python script ROS_plots,py to plot the angle trajectories using the matplotlib library. The different set of values used in the simulations are described in [this](plots/Table1.png) table.

## Results

Once the appropriate values for the inertia matrix and center of mass are set, I ran the simulation for the set of values 1. The results obtained for are shown in [Fig. 1](plots/Fig1.png) It can be seen that the trajectory of the angles describe successfully the behavior of a pendulum. The 2R robot goes from the initial angle and falls describing and oscilating trajectory, which is damped due to friction, which is represented in the URDF file as the joint damping. Therefore, after some time (approximately 12 seconds) the 2R robot stops moving in the final position, which corresponds to -π radians for joint 1 and 0 radians for joint 2. 

In [Fig. 2](plots/Fig2.png), it can be seen that increasing the mass of the links leads to an increment in the moment of inertia of the bodies. Therefore, the links’ tendency to maintain their movement is higher than in the set of values 1, and they keep moving during more time as the plot shows.

Similarly that [Fig. 2](plots/Fig2.png), [Fig. 3](plots/Fig3.png) shows that increasing the length of the robot’s link increases the inertia properties of the links. Therefore, we can see that the links tend to keep their movement for a larger amount of time compared with the first set of values.

In [Fig. 4](plots/Fig4.png), we see the effect of changing the initial position of the joint 1 angle. However, it is important to denote that the plot shows an initial angle equal to 0. This is because when we change the initial angle in the URDF file, we are actually changing the rotation of the joint respect to its parent link. Therefore, the link’s position will rotate 90°, but the initial joint’s angle will still be 0°. However, the plot shows that the behavior of the system change completely respect to the set of values 1 as the links moves during more time and the trajectory described is more oscillatory than the previous ones. 

Finally, it has been verified from the results obtained above that changing the mass and length properties of the rigid body will affect directly the bodes’ inertia properties, and therefore it will affects the dynamics of those bodies. 
