# ROS2 Package - Repository associated with RAL-ICRA 2022 submission:  
"Multi-Modal Model Predictive Control through Batch Non-Holonomic Trajectory Optimization: Application to Highway Driving"

[![Multi-Modal Model Predictive Control through Batch Non-Holonomic Trajectory Optimization: Application to Highway Driving](https://github.com/dv367/Batch-Opt-Highway-Driving/blob/master/ros_ws/stats/Screenshot.png)](http://www.youtube.com/watch?v=z2cDWWb_oS0&ab_channel=VivekAdajania)

## Dependencies
* [eigen_quad_prog](https://github.com/jrl-umi3218/eigen-quadprog)   
```
sudo apt install libeigen-quadprog-dev  
sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen
```  
* [NGSIM dataset](https://drive.google.com/drive/folders/1cgsOWnc4JTeyNdBN6Fjef2-J5HqjnWyX?usp=sharing)

    The dataset is for the I-80 freeway in the San Francisco Bay area , which can be downloaded from the above link. After downloading the data, place the downloaded file in `ros_ws/src/highway_car/highway/car`. The dataset has been taken from the [US Department of Transportaion website](https://data.transportation.gov/Automobiles/Next-Generation-Simulation-NGSIM-Vehicle-Trajector/8ect-6jqj). 
  

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
* In each approach folder, you will find ```config.yaml```, select ```setting``` :  
    `cruise_IDM`, `cruise_NGSIM`, `HSRL_IDM`, `HSRL_NGSIM`


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
ros2 run mpc_car_acado_single mpc_node_single
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

