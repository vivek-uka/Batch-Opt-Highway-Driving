## Structure  
The folder ```ros_ws/src``` contains the implementation of approaches: Standard MPC, Batch ACADO over parallel threads, Frenet Frame Planner in Python, Frenet Frame Planner in C++, and our proposed Multi-modal MPC. It also contains a highway driving simulator and custom ros2 messages used by the packages.  
* **mpc_car_acado_single**: implementation of standard MPC. The problem formulation can be viewed in the code generation file.  
* **mpc_car_acado**: implementation of batch ACADO or multi-threaded ACADO where each thread solves the optimization problem for different goals.  
* **frenet_cpp**: implementation of trajectory sampling based approach: Frenet Frame Planner in C++  
* **mpc_car_batch**: implementation of our proposed multi-modal MPC that is built on Eigen C++ library.
* **highway_car**: a highway driving simulator where obstacles are motivated by Intelligent Driver Model (IDM).
* **msgs_car**: custom ROS2 messages that consists of visualization data as well as control input data.
* **stats**: folder where the simulation data is saved
