
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
agent_r = 1.95/2
obs_r = 1.95/2
goal_r=1.0
dt = 0.08
upper_lim=140
lower_lim=-30
Case = 0.
agent_p = np.array([0,0])
obs_p = np.array([[20,-2], [10,2]])
obs_v = np.array([[3,0], [3,0]])
goal_p = np.array([70,-2])
detected_obs=9

Case = 1
agent_p = np.array([0,10])
total_obs = 36#+12
lane_y = [10, -6, -2, 2, 6, -10]
other_vehicles = np.zeros([total_obs, 5])       # x y vx vy dist
other_vehicles[:,1] = (np.hstack((lane_y,lane_y,lane_y,lane_y,lane_y,lane_y)))#,self.lane_y,self.lane_y)))
other_vehicles[:,0] = np.array([   70, 5, -10, 70, 65, -12, 
                                        77, -12, 15, -20, -25, -24,
                                        55, 67, 30, 40, 77, 32,
                                        91, 37, 69, 80, 35, 105,
                                        108, 74, 45, 60, -15, 96,
                                        120, 51, 5, 90, 42, 90])#,
                                        

other_vehicles[:,4] = np.sqrt((other_vehicles[:,0] - agent_p[0])**2 + (other_vehicles[:,1] - agent_p[1])**2) + (((-other_vehicles[:,0] + agent_p[0] - 2.5)/(abs(-other_vehicles[:,0] + agent_p[0] - 2.5)+0.0001)) + 1) * 10000
other_vehicles[:,2] = random.choices(range(8, 10), k=len(other_vehicles))
other_vehicles = other_vehicles[other_vehicles[:, 4].argsort()]
obs_p=other_vehicles[:,0:2]
obs_v=other_vehicles[:,2:4]
other_vehicles_desired = np.hstack((12*np.ones(3), 10*np.ones(3),11*np.ones(3),7*np.ones(3),11.5*np.ones(3),6.5*np.ones(3),8*np.ones(3),9*np.ones(3),10*np.ones(3),8.5*np.ones(3),9.5*np.ones(3),7.5*np.ones(3)))
goal_p = np.array([1500,agent_p[1]])


wx = [0, goal_p[0]]
wy = [0,0]

c_speed = 15 # current speed [m/s]
c_d = agent_p[1]  # current lateral position [m]
c_d_d = 0.0  # current lateral speed [m/s]
c_d_dd = 0.0  # current latral acceleration [m/s]
s0 = 0.0  # current course position

MAX_SPEED = 24
MAX_ACCEL = 4.0
MAX_CURVATURE = 8.0
MAX_ROAD_WIDTH = np.arange(-11.5,12)
D_ROAD_W = 1.0
DT = dt
MAXT = {2.8,3.6,4.5}
MINT = 2.7
TARGET_SPEED = 15
D_T_S = 10.0 / 3.6
N_S_SAMPLE = 2
agent_v=np.array([c_speed,0], np.double)
#%%
#%========================================%
#%Simulation details continued
#%========================================%

x_min = np.min(np.hstack((obs_p[:,0],agent_p[0],goal_p[0])))
x_max = np.max(np.hstack((obs_p[:,0],agent_p[0],goal_p[0])))
y_min = np.min(np.hstack((obs_p[:,1],agent_p[1],goal_p[1])))
y_max = np.max(np.hstack((obs_p[:,1],agent_p[1],goal_p[1])))
range1 = np.max(np.array([x_max-x_min, y_max-y_min]))
x_axis_min = (x_min+x_max)/2.-range1/2.-2.
x_axis_max = (x_min+x_max)/2.+range1/2.+2.
y_axis_min = (y_min+y_max)/2.-range1/2.-2.
y_axis_max = (y_min+y_max)/2.+range1/2.+2.

def IDM(other_vehicles, agent_p, agent_v):

    so = 1
    T = 1.5

    l = 5

    a = 4
    b = 3
    
    for i in range(len(other_vehicles)):
        nearest = -1
        x = other_vehicles[i][0]
        y = other_vehicles[i][1]
        inLane = other_vehicles[:,1] - y * np.ones(len(other_vehicles))
        index = np.where( inLane == 0)

        min = 10000
        for k in range(len(index[0])):
            if index[0][k] != i:
                if x < other_vehicles[index[0][k]][0]:
                    dist = other_vehicles[index[0][k]][0] - x
                    if dist < min:
                        min = dist
                        nearest = index[0][k]
         
        v_0 = other_vehicles[nearest][2]
        x_0 = other_vehicles[nearest][0]
        if (y - agent_p[1]) <= 1.8 and x < agent_p[0]:
            if nearest != -1:
                if agent_p[0] < other_vehicles[nearest][0]: 
                    nearest = 1
                    v_0 = agent_v[0]
                    x_0 = agent_p[0]
            else:
                v_0 = agent_v[0]
                x_0 = agent_p[0]
        v_1 = other_vehicles[i][2]
        x_1 = x

        v_r = other_vehicles_desired[i]
        delta_v = v_1 - v_0
        s_alpha = x_0 - x_1 - l

        if nearest != -1:
            s_star = so + v_1 * T + (v_1 * delta_v)/(2 * np.sqrt(a*b))
            decc = a * (1 - (v_1/v_r)**4 - (s_star/s_alpha)**2)
            if decc < -a:
                decc = -a
            other_vehicles[i][2] += decc * dt
        else:
            decc = a * (1 - (v_1/v_r)**4)
            if decc > a:
                decc = a
            other_vehicles[i][2] += decc * dt
    return other_vehicles       

current_head = math.atan2((goal_p[1]-agent_p[1]), (goal_p[0]-agent_p[0]))
def frenet_trajectory(agent_p, obs_p, obs_v, agent_r, obs_r, dt, wx, wy, c_speed, c_d, c_d_d, c_d_dd, s0, MAX_SPEED, MAX_ACCEL, MAX_CURVATURE, MAX_ROAD_WIDTH, D_ROAD_W, MAXT, MINT, TARGET_SPEED,D_T_S ,N_S_SAMPLE):
    tx, ty, tyaw, tc, csp = frenet_optimal_trajectory.generate_target_course(wx, wy)
    prev_vec=agent_p;
    path = frenet_optimal_trajectory.frenet_optimal_planning(csp, s0, c_speed, c_d, c_d_d, c_d_dd, obs_p,obs_v,prev_vec, MAX_SPEED, MAX_ACCEL, MAX_CURVATURE, MAX_ROAD_WIDTH, D_ROAD_W, dt, MAXT, MINT, TARGET_SPEED,D_T_S ,N_S_SAMPLE,agent_r,obs_r)
    s0 = path.s[1]
    c_d = path.d[1]
    c_d_d = path.d_d[1]
    c_d_dd = path.d_dd[1]
    c_speed = path.s_d[1]
    v = (np.array([path.x[1],path.y[1]])-agent_p)/dt
    agent_v[0] = np.linalg.norm(v)
    agent_v[1] = (np.arctan2(path.y[1]-agent_p[1], path.x[1]-agent_p[0]) - current_head)/dt
    return agent_v, s0, c_d, c_d_d, c_d_dd, c_speed, v
counter=0 
save=1
bot_path = np.array([agent_p])
while np.linalg.norm((agent_p-goal_p)) > 0.2:
	ob_p=other_vehicles[:detected_obs,0:2]
	ob_v=other_vehicles[:detected_obs,2:4]	    
	agent_v, s0, c_d, c_d_d, c_d_dd, c_speed, v=frenet_trajectory(agent_p, ob_p, ob_v, agent_r, obs_r, dt, wx, wy, c_speed, c_d, c_d_d, c_d_dd, s0, MAX_SPEED, MAX_ACCEL, MAX_CURVATURE, MAX_ROAD_WIDTH, D_ROAD_W, MAXT, MINT, TARGET_SPEED,D_T_S ,N_S_SAMPLE)
	current_head = current_head+np.dot(agent_v[1], dt)
	agent_p = [agent_p[0] + agent_v[0]*np.cos(current_head)*dt, agent_p[1] + agent_v[0]*np.sin(current_head)*dt]
	bot_path=np.append(bot_path,[agent_p],axis=0)
	other_vehicles[:,0] += other_vehicles[:,2] * dt    #x
	other_vehicles[:,1] += other_vehicles[:,3] * dt    #y
	other_vehicles[:,4] = np.sqrt((other_vehicles[:,0] - agent_p[0])**2 + (other_vehicles[:,1] - agent_p[1])**2) + (((-other_vehicles[:,0] + agent_p[0]-2.5)/(abs(-other_vehicles[:,0] + agent_p[0]-2.5)+0.0001)) + 1) * 10000
	other_vehicles = other_vehicles[other_vehicles[:, 4].argsort()]
	a=other_vehicles[:,2:4]
	other_vehicles=IDM(other_vehicles, agent_p, agent_v)
	obs_p=other_vehicles[:,0:2]
	obs_v=other_vehicles[:,2:4]
	lower_lim = -30 + agent_p[0]
	upper_lim = 140 + agent_p[0]
	#print(agent_v, np.sum(obs_v-a))
	draw.draw1(obs_p, obs_v, agent_p,v, goal_p, obs_r, agent_r, goal_r, bot_path,x_axis_min, x_axis_max, y_axis_min, y_axis_max,counter,dt,save,detected_obs)
	for i in range(len(other_vehicles)):
	    if other_vehicles[i][0] < lower_lim-6 and other_vehicles[i][1] != -10:
	        other_vehicles[i][0] = upper_lim + 5
	    if other_vehicles[i][0] > upper_lim+6 and other_vehicles[i][1] != -10:
	        other_vehicles[i][0] = lower_lim - 5
	counter=counter+1