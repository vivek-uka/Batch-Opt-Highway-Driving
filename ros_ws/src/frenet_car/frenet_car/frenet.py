import numpy as np
import math
# import frenet_car.frenet_optimal_trajectory
from frenet_car import frenet_optimal_trajectory
from time import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3, PoseArray, Pose
from msgs_car.msg import States, Controls

#%========================================%
#%Simulation details
#%========================================%

class MinimalSubscriber(Node):

	def __init__(self):
		super().__init__('minimal_subscriber')
		self.subscription = self.create_subscription(
			States,
			'ego_vehicle_obs',
			self.listener_callback,
			10
		)
		self.subscription
		self.publisher_ = self.create_publisher(Controls, 'ego_vehicle_cmds', 10)
		self.timer = self.create_timer(0.1, self.timer_callback)

		self.agent_r = 3.0/2
		self.obs_r = 3.0/2
		self.goal_r=1.0
		self.dt = 0.08

		self.agent_v = np.array([0,0], np.double)
		self.agent_p = np.array([-10,-2], np.double)
		self.obs_p = np.zeros([6, 2],  np.double)
		self.obs_v = np.zeros([6, 2], np.double)
		self.goal_p = np.array([1500,self.agent_p[1]],  np.double)

		self.wx = [0, self.goal_p[0]]
		self.wy = [0,0]

		self.c_speed = self.agent_v[0]  # current speed [m/s]
		self.c_d = self.agent_p[1]  # current lateral position [m]
		self.c_d_d = 0.0  # current lateral speed [m/s]
		self.c_d_dd = 0.0  # current latral acceleration [m/s]
		self.s0 = 0.0  # current course position

		self.MAX_SPEED = 24
		self.MAX_ACCEL = 4.0
		self.MAX_CURVATURE = 10.0
		self.MAX_ROAD_WIDTH = np.arange(-12+3.0/2.0,12.0-3.0/2.0)
		self.D_ROAD_W = 1.0
		self.DT = self.dt
		self.MAXT = {2.8, 3.6, 4.5}#{3,5,6}# planning time // for ngsim_0 add 1.5s
		self.MINT = 2.8#3
		self.TARGET_SPEED = 15
		self.D_T_S = 10.0 / 3.6		# target + dts
		self.N_S_SAMPLE = 4
		

		self.current_head = math.atan2((self.goal_p[1]-self.agent_p[1]), (self.goal_p[0]-self.agent_p[0]))
		self.Gotit = 0
		self.first = 1
		self.loop = 0
		# self.file = open("stats/New/Cruise/No-Time-Budget/car_frenet_data.txt", "w")
		# self.file = open("stats/New/HighSpeed_RightLane/No-Time-Budget/car_frenet_data.txt", "w")
		self.file = open("/home/vivek/On-Codes/Backup/Batch_traj_opt/ros_ws/stats/New/NGSIM/No-Time-Budget/car_frenet_data_0_ngsim.txt", "w")
		self.file2 = open("/home/vivek/On-Codes/Backup/Batch_traj_opt/ros_ws/stats/New/NGSIM/No-Time-Budget/car_frenet_data_0_ngsim_all_info.txt", "w")
		print("NODES ARE UP")

	def timer_callback(self):
		
		if(self.Gotit):
			msg = Controls()
			
			tx, ty, tyaw, tc, csp = frenet_optimal_trajectory.generate_target_course(self.wx, self.wy)
			
			t1 = time()
			path, fplist, index= frenet_optimal_trajectory.frenet_optimal_planning(csp, self.s0, self.c_speed, self.c_d, self.c_d_d, self.c_d_dd, 
					self.obs_p, self.obs_v, self.prev_vec, self.MAX_SPEED, self.MAX_ACCEL, self.MAX_CURVATURE, self.MAX_ROAD_WIDTH, 
					self.D_ROAD_W, self.dt, self.MAXT, self.MINT, self.TARGET_SPEED, self.D_T_S , self.N_S_SAMPLE, self.agent_r, self.obs_r)
			comp_time = time() - t1
			
			for i in range(len(fplist)):
				np.savetxt(self.file2, np.asarray([fplist[i].x, fplist[i].y]), delimiter='\n')
				for j in range(len(fplist[i].x)):
					pose = Pose()
					pose.position.x = fplist[i].x[j]
					pose.position.y = fplist[i].y[j]
					msg.batch.poses.append(pose)

			self.s0 = path.s[1]
			self.c_d = path.d[1]
			self.c_d_d = path.d_d[1]
			self.c_d_dd = path.d_dd[1]
			self.c_speed = path.s_d[1]

			v = (np.array([path.x[1],path.y[1]])-self.agent_p)/self.dt
			v_control = np.linalg.norm(v)
			w_control = (np.arctan2(path.y[1]-self.agent_p[1], path.x[1]-self.agent_p[0]) - self.current_head)/self.dt

			
			msg.v = v_control
			msg.w = w_control
			msg.index = index
			msg.goals = len(fplist)
			
			self.publisher_.publish(msg)
			self.Gotit = 0
			lin_acc = (msg.v - self.agent_v[0])/self.dt
			ang_acc = (msg.w - self.agent_v[1])/self.dt
			self.loop +=1
			np.savetxt(self.file,np.asarray([self.agent_p[0], self.agent_p[1], self.current_head, self.agent_v[0], self.agent_v[1], lin_acc, ang_acc, comp_time, self.loop, index]), delimiter='\n')
			

	def listener_callback(self,msg):
	
		
		self.prev_vec = self.agent_p
		self.obs_p[:,0] = msg.x[1:]
		self.obs_p[:,1] = msg.y[1:]
		self.obs_v[:,0] = msg.vx[1:]
		self.obs_v[:,1] = msg.vy[1:]

		# print(self.obs_p)
		
		self.agent_p[0] = msg.x[0]
		self.agent_p[1] = msg.y[0]
		self.current_head = msg.psi[0]
		# print("me", self.agent_p)
		self.agent_v[0] = np.sqrt(msg.vx[0] ** 2 + msg.vy[0] ** 2)
		self.agent_v[1] = msg.psidot
		
		if self.first:
			
			self.prev_vec = self.agent_p
			# self.s0 = self.agent_p[0]
			self.c_speed = self.agent_v[0]  # current speed [m/s]
			self.c_d = self.agent_p[1]  # current lateral position [m]
			self.wx = [self.agent_p[0], self.goal_p[0]]
			self.first = 0
			
		self.Gotit = 1

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    rclpy.shutdown()


if __name__ == '__main__':
    main()