# ROS2 Package - Repository associated with RAL-ICRA 2022 submission:  
"Multi-Modal Model Predictive Control through Batch Non-Holonomic Trajectory Optimization: Application to Highway Driving"

## Dependencies
* [eigen_quad_prog](https://github.com/jrl-umi3218/eigen-quadprog)   
```
sudo apt install libeigen-quadprog-dev  
sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen
```  
* [NGSIM dataset](https://drive.google.com/drive/folders/1cgsOWnc4JTeyNdBN6Fjef2-J5HqjnWyX?usp=sharing)  
  

## Installation
After installing the dependencies, build our package as follows:  
``` 
cd your_ws/src
git clone https://github.com/dv367/Batch-Opt-Highway-Driving  
cd your_ws/src/ros_ws/src  
colcon build  
source ./install/setup.bash  
``` 
#### Setting a high-level driving mission  
* In each approach folder, you will find ```config.yaml```, change ```setting``` to:
|Obstacles||Mission|Name|

#### In the first terminal:
* Running our proposed batch optimization  
```
ros2 run mpc_car_batch mpc_node  
```
* Running multi-threaded-acado 
```
ros2 run mpc_car_acado mpc_node  
```
* Running standard-mpc-acado   
```
ros2 run mpc_car_acado mpc_node_single
```
* Runinng frenet-frames
```  
ros2 run frenet_car frenet_node
```
#### In the second terminal:
```
source ./install/setup.bash  
ros2 run highway_car highway_node2    
```

