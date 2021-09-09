
from math import radians
import random
import numpy as np
from time import time
import matplotlib.pyplot as plt
import yaml
from yaml.loader import SafeLoader

from matplotlib.patches import Ellipse, Rectangle
from shapely.geometry import Point
from shapely.affinity import scale, rotate


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from msgs_car.msg import States, Controls
# from highway_car import ngsim_data

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Controls,
            'ego_vehicle_cmds',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.plot_ellipse = 0


        self.publisher_ = self.create_publisher(States, 'ego_vehicle_obs', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        random.seed(0)
        self.a_ell = 5.6
        self.b_ell = 3.0
        self.a_rect = 4.0
        self.b_rect = 1.4

        self.cnt = 1
        self.loop = 0
        self.index = 0
        self.intersection = [False, False, False, False, False, False]
        self.upper = 140 #130 for cruisie
        self.lower_lim = -30
        self.upper_lim = self.upper
        self.pre_x = np.array([])
        self.pre_y = np.array([])
        self.Gotit = 1.0
        self.v_controls = np.array([])
        self.psi_constrols = np.array([])
        self.num_goal = 10

        with open('src/highway_car/config.yaml') as f:
            data = yaml.load(f, Loader=SafeLoader)
            setting = str(data["setting"])
        # cruise = 0, Right Lane = 1, High Speed RightLane = 2, NGSIM = 3
        if setting == "cruise_IDM":
            self.setting = 0 

        elif setting == "RL_IDM":
            self.setting = 1
        elif setting == "HSRL_IDM":
            self.setting = 2
        else:
            self.setting = 3

        if self.setting == 3:
            self.NGSIM = True
        else:
            self.NGSIM = False
        print (self.setting)
        self.num_obs = 6
        self.obs = np.zeros([self.num_obs+1, 4])

        # if self.NGSIM == False:
        #     self.obs[0] = [0, -10, 14.0, 0.0]
        # else:
        #     self.obs[0] =[-10, -2, 14.0, 0.0]

        self.obs[0] = [0, 10, 14.0, 0.0]
        self.other_vehicles = np.array([])

                
        if self.NGSIM == False:
            total_obs = 18
            self.lane_y = [-10, -6, -2, 2, 6, 10]
            self.other_vehicles = np.zeros([total_obs, 6])       # x y vx vy dist psi
            self.other_vehicles[:,1] = (np.hstack((self.lane_y,self.lane_y,self.lane_y)))
            self.other_vehicles[:,5] = np.zeros(total_obs)
            #cruise and hsrl
            if self.setting == 0 or self.setting == 2:
                self.other_vehicles[:,0] = np.array([-10, 25,   60, -20, 40, 35,
                                                    70, 85,   100, 10, 80, 170,
                                                    130, 110, 160, 140, 135, 95 
                ])
            else:
                self.other_vehicles[:,0] = np.array([-10, 25,   60, -20, 5, -35,
                                                    90, 85,   75, 10, 25, 50,
                                                    20, 40,   10,  100, 50, 95 
                ])
            self.other_vehicles[:,4] = np.sqrt((self.other_vehicles[:,0] - self.obs[0][0])**2 + (self.other_vehicles[:,1] - self.obs[0][1])**2) + (((-self.other_vehicles[:,0] + self.obs[0][0] - self.a_ell/2)/(abs(-self.other_vehicles[:,0] + self.obs[0][0] - self.a_ell/2)+0.0001)) + 1) * 10000
            self.other_vehicles = self.other_vehicles[self.other_vehicles[:, 4].argsort()]
            self.other_vehicles_desired = [10, 11, 8.0, 9.5, 8.5, 12, 
                                            7.5, 8.5, 7.0, 9.0, 8.5, 9.0,
                                            10.0, 9.5, 7.2, 9.5, 10, 10.5]
            self.other_vehicles[:,2] = np.roll(self.other_vehicles_desired, 6)
            for i in range(self.num_obs):
                self.obs[i+1] = self.other_vehicles[i,:4] 
        else:
            if setting == "cruise_NGSIM":
                file = "ngsim_data0.csv"
                self.time_shift = 250.0
                self.y_shift = 10.7
            else:
                file = "ngsim_data1.csv"
                self.time_shift = 511.12
                self.y_shift = 11.7
                self.obs[0] =[-10, 2, 14.0, 0.0]


            print("READING DATA........")
            self.ngsim_obs = np.genfromtxt('src/highway_car/highway_car/'+file,delimiter=',') # id x y psi length width vx vy  time
            mask = self.ngsim_obs[:,8] == np.round(self.time_shift, 2) # 2 for data0
            ngsim_obs = self.ngsim_obs[mask,:]
            
            self.other_vehicles = np.zeros([len(ngsim_obs), 8]) # x y vx vy dist psi length width
            self.other_vehicles[:,0] = ngsim_obs[:,2]
            self.other_vehicles[:,1] = ngsim_obs[:,1] - self.y_shift
            self.other_vehicles[:,2] = ngsim_obs[:,7]
            self.other_vehicles[:,3] = ngsim_obs[:,6]
            self.other_vehicles[:,5] = (np.pi/2 - ngsim_obs[:,3])
            self.other_vehicles[:,6] = ngsim_obs[:,4]
            self.other_vehicles[:,7] = ngsim_obs[:,5]
            # self.other_vehicles[:,4] = np.sqrt((self.other_vehicles[:,0] - self.obs[0][0])**2 + (self.other_vehicles[:,1] - self.obs[0][1])**2)
            self.other_vehicles[:,4] = np.sqrt((self.other_vehicles[:,0] - self.obs[0][0])**2 + (self.other_vehicles[:,1] - self.obs[0][1])**2) + (((-self.other_vehicles[:,0] + self.obs[0][0] - self.a_ell/2)/(abs(-self.other_vehicles[:,0] + self.obs[0][0] - self.a_ell/2)+0.0001)) + 1) * 10000
            self.other_vehicles = self.other_vehicles[self.other_vehicles[:, 4].argsort()]
            self.obs[1:] = self.other_vehicles[:len(self.obs)-1,:4]
            
        self.ours_x = []
        self.ours_y = []



        self.v = self.obs[0][2]
        self.w = 0.0 

        self.prev_psi = 0.0
        self.psi = 0.0
        self.dt = 0.08
    
        self.sim_time = np.array([])

        self.flag = 1
        self.fig = plt.figure(0)
        self.ax = self.fig.add_subplot(111, aspect='equal')
        mng = plt.get_current_fig_manager()
        # mng.full_screen_toggle()
        self.fig.set_size_inches(20, 10)
        
        print("STARTING SIMULATION")
    
    def create_ellipse(self, center, axes, inclination):
        p = Point(*center)
        c = p.buffer(1)
        ellipse = scale(c, *axes)
        ellipse = rotate(ellipse, inclination)
        return ellipse

    def checkCollision(self):
        
        obs_ellipse = [self.create_ellipse((self.other_vehicles[i][0], self.other_vehicles[i][1]), (self.a_ell/2, self.b_ell/2), 0) for i in range(self.num_obs)]
        ego_ellipse = self.create_ellipse((self.obs[0][0], self.obs[0][1]), (self.a_ell/2, self.b_ell/2), 0*self.psi*180.0/np.pi)

        self.intersection = [ego_ellipse.intersects(obs_ellipse[i]) for i in range(self.num_obs)]
        for i in range(self.num_obs):
            if self.intersection[i]:
                ptsx, ptsy = ego_ellipse.intersection(obs_ellipse[i]).exterior.coords.xy
                if len(ptsx) < 10:
                    self.intersection[i] = False
        self.intersection = any([inter == True for inter in self.intersection])


    def IDM(self):

        so = 1
        T = 1

        l = self.a_ell

        a = 4
        b = 3
        
        for i in range(len(self.other_vehicles)):
            nearest = -1
            x = self.other_vehicles[i][0]
            y = self.other_vehicles[i][1]
            inLane = self.other_vehicles[:,1] - y * np.ones(len(self.other_vehicles))
            index = np.where( inLane == 0)

            min = 10000
            for k in range(len(index[0])):
                if index[0][k] != i:
                    if x < self.other_vehicles[index[0][k]][0]:
                        dist = self.other_vehicles[index[0][k]][0] - x
                        if dist < min:
                            min = dist
                            nearest = index[0][k]
             
            v_0 = self.other_vehicles[nearest][2]
            x_0 = self.other_vehicles[nearest][0]
            if (y - self.obs[0][1]) <= self.b_ell and x < self.obs[0][0]:
                if nearest != -1:
                    if self.obs[0][0] < self.other_vehicles[nearest][0]: 
                        nearest = 1
                        v_0 = self.obs[0][2]
                        x_0 = self.obs[0][0]
                else:
                    v_0 = self.obs[0][2]
                    x_0 = self.obs[0][0]
            v_1 = self.other_vehicles[i][2]
            x_1 = x

            v_r = self.other_vehicles_desired[i]
            delta_v = v_1 - v_0
            s_alpha = x_0 - x_1 - l

            if nearest != -1:
                s_star = so + v_1 * T + (v_1 * delta_v)/(2 * np.sqrt(a*b))
                decc = a * (1 - (v_1/v_r)**4 - (s_star/s_alpha)**2)
                if decc < -a:
                    decc = -a
                self.other_vehicles[i][2] += decc * self.dt
            else:
                decc = a * (1 - (v_1/v_r)**4)
                if decc > a:
                    decc = a
                self.other_vehicles[i][2] += decc * self.dt

        


    def listener_callback(self, msg):
        
        self.Gotit = 1.0
        self.v = msg.v
        self.w = msg.w     
        
        xx  = 0
        cnt = 0
        self.steps = []
        self.pre_x = np.array([])
        self.pre_y = np.array([])
        for i in msg.batch.poses:
            self.pre_x = np.append(self.pre_x, i.position.x)
            self.pre_y = np.append(self.pre_y, i.position.y)
            if abs(i.position.x - self.pre_x[0]) < 0.001 and len(self.pre_x) > 1:
                self.steps.append(cnt)
                cnt = 0
            cnt+=1
        self.steps.append(cnt)
        self.num_goal = msg.goals
        self.index = msg.index
        self.v_controls = np.append(self.v_controls, self.v)
        
     
#
    def timer_callback(self):
        
        if (self.obs[0][0] < 600 and self.setting == 2) or (self.obs[0][1] >= -8.0-0.7 and self.setting == 1) or (self.obs[0][0] < 1000 and self.setting == 0) or (self.obs[0][0] < 400 and self.NGSIM == True):#self.obs[0][1] >= -8.0-0.7:#self.obs[0][0] < 1000:#self.obs[0][1] >= -8.0-0.7:#self.obs[0][0] < 1000 or self.obs[0][1] >= -10:
            
            if self.Gotit or self.flag:
                t1 = time()
                dt = self.dt
                
                self.ours_x.append(self.obs[0][0])
                self.ours_y.append(self.obs[0][1])

                self.loop += 1
                self.sim_time = np.append(self.sim_time, self.loop * dt)
                if self.flag == 0:
                    plt.clf()                        
                    self.ax = self.fig.add_subplot(111, aspect='equal')
                    prev = 0
                    for i in range(self.num_goal*(1-self.flag)):
                        if i == self.index:
                            plt.plot(self.pre_x[prev+2:prev+self.steps[i]], self.pre_y[prev+2:prev+self.steps[i]], alpha = 1, linewidth=3.5, color='red')#'orange')
                            plt.scatter(self.pre_x[prev+self.steps[i]-1], self.pre_y[prev+self.steps[i]-1], color='red')#'orange')
                        else:
                            plt.plot(self.pre_x[prev+2:prev+self.steps[i]], self.pre_y[prev+2:prev+self.steps[i]], alpha = 0.35)#, color='green')#'orange')
                        prev += self.steps[i]
                        
                        
                    diag = np.sqrt(self.a_rect ** 2 + self.b_rect ** 2)
                    ells = [Ellipse(xy=[self.other_vehicles[i][0], self.other_vehicles[i][1]], width=self.a_ell, height=self.b_ell, angle=0.0) for i in range(len(self.other_vehicles))]
                    rect = [Rectangle(xy=[self.other_vehicles[i][0] -  diag/2 * np.cos(self.other_vehicles[i][5] + np.arctan(self.b_rect/self.a_rect)), self.other_vehicles[i][1] - diag/2 * np.sin(self.other_vehicles[i][5] + np.arctan(self.b_rect/self.a_rect))], width=4.0, height=1.4, angle=self.other_vehicles[i][5]*180.0/np.pi) for i in range(len(self.other_vehicles))]
                    mm = 0
                    for e, r in zip(ells, rect):
                        self.ax.add_artist(e)
                        self.ax.add_artist(r)
                        e.set_clip_box(self.ax.bbox)
                        e.set_alpha(1)
                        r.set_alpha(0.5)
                        r.set_facecolor([1, 1, 1])
                        e.set_facecolor([0.0, 0.5, 1])
                        # if mm < 6:
                        #     e.set_facecolor([0.5, 0.5, 0.5])
                        if (self.other_vehicles[mm][0] < self.upper_lim - 2.5 and self.other_vehicles[mm][0] > self.lower_lim + 2.5) and (self.other_vehicles[mm][1] > -12 and self.other_vehicles[mm][1] < 12):
                            plt.text(self.other_vehicles[mm][0]-2, self.other_vehicles[mm][1]-0.3, '%s'%(round(self.other_vehicles[mm][2],2)), fontsize=10)
                        mm+=1
                    rob = [Ellipse(xy=[self.obs[0][0], self.obs[0][1]], width=self.a_ell, height=self.b_ell, angle=0*self.psi*180.0/np.pi)]
                    rect = [Rectangle(xy=[self.obs[0][0] - diag/2 * np.cos(self.psi + np.arctan(self.b_rect/self.a_rect)), self.obs[0][1] - diag/2 * np.sin(self.psi + np.arctan(self.b_rect/self.a_rect))], width=4.0, height=1.4, angle=self.psi*180.0/np.pi)]
                    for e, r in zip(rob, rect):
                        self.ax.add_artist(e)
                        self.ax.add_artist(r)
                        e.zorder = 10
                        r.zorder = 10
                        e.set_clip_box(self.ax.bbox)
                        e.set_alpha(1)
                        r.set_alpha(0.5)
                        r.set_facecolor([1, 1, 1])
                        e.set_facecolor([1, 0.5, 0.5])
                        # e.set_facecolor([1.0, 0.647, 0.0]) # orange
                        # e.set_facecolor([0, 100/255, 0]) #green
                        plt.text(self.obs[0][0]-2, self.obs[0][1]-0.3, '%s'%(round(self.obs[0][2],2)), fontsize=10, zorder = 20)
                    
                    plt.plot(self.ours_x, self.ours_y, color=[1, 0.5, 0.5])

                    plt.xlabel('Y in m')
                    plt.ylabel('X in m')
                    self.lower_lim = -30 + self.obs[0][0]
                    self.upper_lim = self.upper + self.obs[0][0]
                    
                    
                    if self.flag == 0:
                        plt.text(self.lower_lim+10, 14, 'Collision with obstacle= %s'%((self.intersection)), fontsize=10) 
                        plt.text(self.lower_lim+50, 14, 'Average speed= %s m/s'%(round(self.v_controls.mean(), 3)), fontsize=10)
                        plt.text(self.lower_lim+90, 14, 'Orientation= %s degrees'%(round(self.psi*180/np.pi, 3)), fontsize=10)
                        plt.text(self.lower_lim+130, 14, 'Number of Trajectories= %s'%(self.num_goal), fontsize=10)
                        
                    plt.text((self.lower_lim+self.upper_lim)/2 - 15, 20, 'Highway environment', fontsize=14)
                    plt.plot([self.lower_lim, self.upper_lim], [-4, -4], color='black',linestyle='--', alpha=0.2)
                    plt.plot([self.lower_lim, self.upper_lim], [0, 0], color='black',linestyle='--', alpha=0.2)
                    plt.plot([self.lower_lim, self.upper_lim], [4, 4], color='black',linestyle='--', alpha=0.2)
                    plt.plot([self.lower_lim, self.upper_lim], [-8, -8], color='black',linestyle='--', alpha=0.2)
                    plt.plot([self.lower_lim, self.upper_lim], [8, 8], color='black',linestyle='--', alpha=0.2)
                    plt.plot([self.lower_lim, self.upper_lim], [12, 12], color='black',linestyle='-', alpha=0.2)
                    plt.plot([self.lower_lim, self.upper_lim], [-12, -12], color='black',linestyle='-', alpha=0.2)
                    plt.xlim(self.lower_lim, self.upper_lim)
                    plt.ylim(-12, 12)
                    # plt.title('highway env')

                    plt.tight_layout()
                    # plt.draw()
                    plt.pause(0.000000000000000001)
                
                if self.Gotit:
                    
                    self.psi += self.w * dt
                    self.obs[0][2] = self.v * np.cos(self.psi)    #vx
                    self.obs[0][3] = self.v * np.sin(self.psi)    #vy
                    self.obs[0][0] += self.obs[0][2] * dt    #x
                    self.obs[0][1] += self.obs[0][3] * dt    #y

                    if self.NGSIM == False:
                        
                        self.IDM()

                        self.other_vehicles[:,0] += self.other_vehicles[:,2] * dt    #x
                        self.other_vehicles[:,1] += self.other_vehicles[:,3] * dt    #y
                        self.other_vehicles[:,4] = np.sqrt((self.other_vehicles[:,0] - self.obs[0][0])**2 + (self.other_vehicles[:,1] - self.obs[0][1])**2) + (((-self.other_vehicles[:,0] + self.obs[0][0]-self.a_ell/2)/(abs(-self.other_vehicles[:,0] + self.obs[0][0]-self.a_ell/2)+0.0001)) + 1) * 10000
                        self.other_vehicles = self.other_vehicles[self.other_vehicles[:, 4].argsort()]
                        self.obs[1:] = self.other_vehicles[:len(self.obs)-1,:4]
                    else:
                        mask = self.ngsim_obs[:,8] == np.round(self.sim_time[-1] + self.time_shift, 2)
                        ngsim_obs = self.ngsim_obs[mask,:]
                        
                        self.other_vehicles = np.zeros([len(ngsim_obs), 8]) # x y vx vy dist psi length width
                        self.other_vehicles[:,0] = ngsim_obs[:,2]
                        self.other_vehicles[:,1] = ngsim_obs[:,1] - self.y_shift
                        
                        self.other_vehicles[:,2] = ngsim_obs[:,7]
                        self.other_vehicles[:,3] = ngsim_obs[:,6]
                        self.other_vehicles[:,5] = (np.pi/2 - ngsim_obs[:,3])
                        self.other_vehicles[:,6] = ngsim_obs[:,4]
                        self.other_vehicles[:,7] = ngsim_obs[:,5]
                        # self.other_vehicles[:,4] = np.sqrt((self.other_vehicles[:,0] - self.obs[0][0])**2 + (self.other_vehicles[:,1] - self.obs[0][1])**2) 
                        self.other_vehicles[:,4] = np.sqrt((self.other_vehicles[:,0] - self.obs[0][0])**2 + (self.other_vehicles[:,1] - self.obs[0][1])**2) + (((-self.other_vehicles[:,0] + self.obs[0][0]-self.a_ell/2)/(abs(-self.other_vehicles[:,0] + self.obs[0][0]-self.a_ell/2)+0.0001)) + 1) * 10000
                        self.other_vehicles = self.other_vehicles[self.other_vehicles[:, 4].argsort()]
                        self.obs[1:] = self.other_vehicles[:len(self.obs)-1,:4]


                self.checkCollision()
                msg = States()
                msg.x = self.obs[:,0].T.tolist()
                msg.y = self.obs[:,1].T.tolist()
                msg.vx = self.obs[:,2].T.tolist()
                msg.vy = self.obs[:,3].T.tolist()
                msg.psi = (self.psi * np.ones(5)).tolist()
                msg.psidot = ((msg.psi[0] - self.prev_psi)/dt) 
                
                self.prev_psi = msg.psi[0]
                
                self.publisher_.publish(msg)

                self.Gotit = 0
                self.flag = 0
                
                if self.NGSIM == False:
                    for i in range(len(self.other_vehicles)):
                        if self.other_vehicles[i][0] < self.lower_lim-6:                    
                            self.other_vehicles[i][0] = self.upper_lim + 5 + self.other_vehicles[i][2]
                            self.other_vehicles[i][2] -= 2 * (i%2)
                            if (self.other_vehicles[i][1] == -10 and self.other_vehicles[i][1] == -6) or self.setting == 1 or self.setting == 0: # for hsrl
                                self.other_vehicles[i][0] += 15 * (i%3)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    rclpy.shutdown()


if __name__ == '__main__':
    main()



