
import numpy as np
import math
import draw
import random
from scipy.optimize import minimize
import frenet_optimal_trajectory
import cubic_spline_planner
# if available import pylab (from matlibplot)
try:
    import matplotlib.pylab as plt
except ImportError:
    pass


#%========================================%
#%Simulation details
#%========================================%
agent_r = 2.0
obs_r = agent_r
goal_r=1.0
dt = 0.1

Case = 0.
agent_p = np.array([0,0])
obs_p = np.array([[20,-2], [10,2]])
obs_v = np.array([[3,0], [3,0]])
goal_p = np.array([70,-2])

wx = [0, goal_p[0]]
wy = [0,0]
tx, ty, tyaw, tc, csp = frenet_optimal_trajectory.generate_target_course(wx, wy)
wx=np.array(wx)
wy=np.array(wy)  

c_speed = 7  # current speed [m/s]
c_d = agent_p[1]  # current lateral position [m]
c_d_d = 0.0  # current lateral speed [m/s]
c_d_dd = 0.0  # current latral acceleration [m/s]
s0 = 0.0  # current course position

MAX_SPEED = 10
MAX_ACCEL = 8.0
MAX_CURVATURE = 8.0
MAX_ROAD_WIDTH = 7.0
D_ROAD_W = 1.0
DT = dt
MAXT = 2.1
MINT = 2.0
TARGET_SPEED = 8
D_T_S = 5.0 / 3.6
N_S_SAMPLE = 1
agent_v=np.array([c_speed,0])
#%%
#%========================================%
#%Simulation details continued
#%========================================%

x_min = np.min(np.hstack((obs_p[:,0],agent_p[0],goal_p[0])))
x_max = np.max(np.hstack((obs_p[:,0],agent_p[0],goal_p[0])))
y_min = np.min(np.hstack((obs_p[:,1],agent_p[1],goal_p[1])))
y_max = np.max(np.hstack((obs_p[:,1],agent_p[1],goal_p[1])))
range = np.max(np.array([x_max-x_min, y_max-y_min]))
x_axis_min = (x_min+x_max)/2.-range/2.-2.
x_axis_max = (x_min+x_max)/2.+range/2.+2.
y_axis_min = (y_min+y_max)/2.-range/2.-2.
y_axis_max = (y_min+y_max)/2.+range/2.+2.

current_head = math.atan2((goal_p[1]-agent_p[1]), (goal_p[0]-agent_p[0]))
bot_path = np.array([agent_p])


save=1
def frenet_trajectory(agent_p,goal_p, obs_p, obs_v, agent_r, obs_r, goal_r, dt, csp, c_speed, c_d, c_d_d, c_d_dd, s0, current_head, bot_path, x_axis_min,x_axis_max,y_axis_min,y_axis_max, MAX_SPEED, MAX_ACCEL, MAX_CURVATURE, MAX_ROAD_WIDTH, D_ROAD_W, MAXT, MINT, TARGET_SPEED,D_T_S ,N_S_SAMPLE):
	counter=0 
	while np.linalg.norm((agent_p-goal_p)) > 0.2:
	    prev_vec=agent_p;
	    bot_path=np.append(bot_path,[agent_p],axis=0)
	    obs_p = obs_p+np.dot(obs_v, dt)
	    path = frenet_optimal_trajectory.frenet_optimal_planning(csp, s0, c_speed, c_d, c_d_d, c_d_dd, obs_p,obs_v,prev_vec, MAX_SPEED, MAX_ACCEL, MAX_CURVATURE, MAX_ROAD_WIDTH, D_ROAD_W, dt, MAXT, MINT, TARGET_SPEED,D_T_S ,N_S_SAMPLE,agent_r)
	    s0 = path.s[1]
	    c_d = path.d[1]
	    c_d_d = path.d_d[1]
	    c_d_dd = path.d_dd[1]
	    c_speed = path.s_d[1]
	    v = (np.array([path.x[0],path.y[0]])-agent_p)/dt
	    agent_v[0] = np.linalg.norm(v)
	    agent_v[1] = (np.arctan2(path.y[1]-agent_p[1], path.x[1]-agent_p[0]) - current_head)/dt
	    current_head = current_head+np.dot(agent_v[1], dt)
	    agent_p = [agent_p[0] + agent_v[0]*np.cos(current_head)*dt, agent_p[1] + agent_v[0]*np.sin(current_head)*dt]
	    draw.draw(obs_p, agent_p,v, goal_p, obs_r, agent_r, goal_r, bot_path,x_axis_min, x_axis_max, y_axis_min, y_axis_max,counter,dt,save,path)
	    counter = counter+1
	    
frenet_trajectory(agent_p,goal_p, obs_p, obs_v, agent_r, obs_r, goal_r, dt, csp, c_speed, c_d, c_d_d, c_d_dd, s0, current_head, bot_path, x_axis_min,x_axis_max,y_axis_min,y_axis_max, MAX_SPEED, MAX_ACCEL, MAX_CURVATURE, MAX_ROAD_WIDTH, D_ROAD_W, MAXT, MINT, TARGET_SPEED,D_T_S ,N_S_SAMPLE)	    
