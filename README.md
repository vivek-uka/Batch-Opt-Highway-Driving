# Batch_traj_opt

Repo for Batch Trajectory Optimization

Ours

Install [Eigen-Quadprog] ``sudo apt install libeigen-quadprog-dev`` and run ``sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen``

1. To compile the code ``g++ main.cpp optim.cpp -o run -O2``


ROS2 

in terimal 1  
colcon build  
source ./install/setup.bash  
ros2 run frenet_car frenet_node  

in terminal 2  
source ./install/setup.bash  
ros2 run highway_car highway_node2  
