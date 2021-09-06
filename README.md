# Repository associated with RAL-ICRA 2022 submission:  
"Multi-Modal Model Predictive Control through Batch Non-Holonomic Trajectory Optimization: Application to Highway Driving"

## Dependencies
* [eigen_quad_prog](https://github.com/jrl-umi3218/eigen-quadprog)
or  
```sudo apt install libeigen-quadprog-dev```
```sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen```  

Download [NGSIM dataset](https://drive.google.com/drive/folders/1cgsOWnc4JTeyNdBN6Fjef2-J5HqjnWyX?usp=sharing)   

ROS2   

in terimal 1  
colcon build  
source ./install/setup.bash  
ros2 run frenet_car frenet_node  

in terminal 2  
source ./install/setup.bash  
ros2 run highway_car highway_node2  
