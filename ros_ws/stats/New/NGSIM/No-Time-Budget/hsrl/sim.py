
from math import radians
import random
import numpy as np
from time import time
import matplotlib.pyplot as plt

from matplotlib.patches import Ellipse, Rectangle
from shapely.geometry import Point
from shapely.affinity import scale, rotate

class sim():

    def __init__(self):
        
        self.ours = np.loadtxt("mpc_car_batch_data_11_goals_0_ngsim.txt") # x y psi v w a j time loop
        self.acado = np.loadtxt("mpc_car_acado_data_11_goals_0_ngsim.txt") # 0 1 2  3 4 5 6 7     8
        self.acado_2 = np.loadtxt("mpc_car_acado_data_6_goals_0_ngsim.txt")
        self.abal = np.loadtxt("mpc_car_acado_data_1_goals_0_ngsim.txt")
        self.frenet = np.loadtxt("car_frenet_data_0_ngsim.txt")
        self.frenet = np.array([self.frenet]).reshape(int(self.frenet[-2]), 10)

        
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

        
        self.setting = 3 # cruise = 0, Right Lane = 1, High Speed RightLane = 2, NGSIM = 3

        if self.setting == 3:
            self.NGSIM = True
        else:
            self.NGSIM = False

        self.num_obs = 6


        self.other_vehicles = np.array([])

                
        file = "ngsim_data1.csv"
        if file == "ngsim_data0.csv":
            self.time_shift = 250.0
            self.y_shift = 10.7
        else:
            self.time_shift = 511.12
            self.y_shift = 11.7

        print("READING DATA........")
        self.ngsim_obs = np.genfromtxt('/home/vivek/On-Codes/Backup/Batch_traj_opt/ros_ws/src/highway_car/highway_car/'+file,delimiter=',') # id x y psi length width vx vy  time
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

        self.dt = 0.08
    
        self.sim_time = np.array([])

        self.fig = plt.figure(0)
        self.ax = self.fig.add_subplot(111, aspect='equal')
        mng = plt.get_current_fig_manager()
        mng.full_screen_toggle()
        self.fig.set_size_inches(20, 10)
        
        print("STARTING SIMULATION")
        


    def plot(self):
        while True:
            dt = self.dt
            
            if self.loop >= len(self.ours):
                ours_x = self.ours[-1][0]
                ours_y = self.ours[-1][1]
                ours_psi = self.ours[-1][2]
            else:
                ours_x = self.ours[self.loop][0]
                ours_y = self.ours[self.loop][1]
                ours_psi = self.ours[self.loop][2]
            if self.loop >= len(self.abal):
                abal_x = self.abal[-1][0]
                abal_y = self.abal[-1][1]
                abal_psi = self.abal[-1][2]
            else:
                abal_x = self.abal[self.loop][0]
                abal_y = self.abal[self.loop][1]
                abal_psi = self.abal[self.loop][2]
            if self.loop >= len(self.acado):
                acado_x = self.acado[-1][0]
                acado_y = self.acado[-1][1]
                acado_psi = self.acado[-1][2]
            else:
                acado_x = self.acado[self.loop][0]
                acado_y = self.acado[self.loop][1]
                acado_psi = self.acado[self.loop][2]
            if self.loop >= len(self.acado_2):
                acado2_x = self.acado_2[-1][0]
                acado2_y = self.acado_2[-1][1]
                acado2_psi = self.acado_2[-1][2]
            else:
                acado2_x = self.acado_2[self.loop][0]
                acado2_y = self.acado_2[self.loop][1]
                acado2_psi = self.acado_2[self.loop][2]
            if self.loop >= len(self.frenet):
                frenet_x = self.frenet[-1][0]
                frenet_y = self.frenet[-1][1]
                frenet_psi = self.frenet[-1][2]
            else:
                frenet_x = self.frenet[self.loop][0]
                frenet_y = self.frenet[self.loop][1]
                frenet_psi = self.frenet[self.loop][2]

                
            ego_vehicles_x = [ours_x, acado_x, acado2_x, abal_x, frenet_x]
            ego_vehicles_y = [ours_y, acado_y, acado2_y, abal_y, frenet_y]
            ego_vehicles_psi = [ours_psi, acado_psi, acado2_psi, abal_psi, frenet_psi]
            
            self.loop += 1
            self.sim_time = np.append(self.sim_time, self.loop * dt)
            
            plt.clf()                        
            self.ax = self.fig.add_subplot(111, aspect='equal')
            prev = 0
                    
                    
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
            
            

            rob = [Ellipse(xy=[ego_vehicles_x[i], ego_vehicles_y[i]], width=self.a_ell, height=self.b_ell, angle=0) for i in range(len(ego_vehicles_x))]
            rect = [Rectangle(xy=[ego_vehicles_x[i] - diag/2 * np.cos(ego_vehicles_psi[i] + np.arctan(self.b_rect/self.a_rect)), ego_vehicles_y[i] - diag/2 * np.sin(ego_vehicles_psi[i] + np.arctan(self.b_rect/self.a_rect))], width=4.0, height=1.4, angle=ego_vehicles_psi[i]*180.0/np.pi) for i in range(len(ego_vehicles_x))]
            mm = 0
            for e, r in zip(rob, rect):
                if mm!= 2:
                    self.ax.add_artist(e)
                    self.ax.add_artist(r)
                e.zorder = 10
                r.zorder = 10
                e.set_clip_box(self.ax.bbox)
                e.set_alpha(1)
                r.set_alpha(0.5)
                r.set_facecolor([1, 1, 1])
                if mm == 0:
                    e.set_facecolor([1, 0.5, 0.5])
                    plt.text(ego_vehicles_x[mm]-2, ego_vehicles_y[mm]-0.3, 'ours', fontsize=10, zorder = 20)
                    plt.plot(self.ours[:self.loop-1,0], self.ours[:self.loop-1,1], color=[1, 0.5, 0.5])
                # if mm == 1:
                #     e.set_facecolor([0.5, 1, 0.5])
                #     plt.text(ego_vehicles_x[mm]-2, ego_vehicles_y[mm]-0.3, 'ACADO_6', fontsize=10, zorder = 20)
                if mm == 1:
                    e.set_facecolor([0.5, 0.5, 1])
                    plt.text(ego_vehicles_x[mm]-2, ego_vehicles_y[mm]-0.3, 'ACADO', fontsize=10, zorder = 20)
                    plt.plot(self.acado[:self.loop-1,0], self.acado[:self.loop-1,1], color=[0.5, 0.5, 1])
                if mm == 3:
                    e.set_facecolor([255/255, 165/255, 0/255])
                    plt.text(ego_vehicles_x[mm]-2, ego_vehicles_y[mm]-0.3, 'SMPC', fontsize=10, zorder = 20)
                    plt.plot(self.abal[:self.loop-1,0], self.abal[:self.loop-1,1], color=[255/255, 165/255, 0/255])
                if mm == 4:
                    e.set_facecolor([34/255,139/255,34/255])
                    plt.text(ego_vehicles_x[mm]-2, ego_vehicles_y[mm]-0.3, 'Frenet', fontsize=10, zorder = 20)
                    plt.plot(self.frenet[:self.loop-1,0], self.frenet[:self.loop-1,1], color=[34/255,139/255,34/255])
                mm+=1
                
                

            plt.xlabel('Y in m')
            plt.ylabel('X in m')
            self.lower_lim = -30 + np.min(ego_vehicles_x)
            self.upper_lim = self.upper + np.min(ego_vehicles_x)
            
                
            
            
            plt.text((self.lower_lim+self.upper_lim)/2 - 15, 20, 'Highway environment', fontsize=14)
            plt.plot([self.lower_lim, self.upper_lim], [-4, -4], color='black',linestyle='--', alpha=0.2)
            plt.plot([self.lower_lim, self.upper_lim], [0, 0], color='black',linestyle='--', alpha=0.2)
            plt.plot([self.lower_lim, self.upper_lim], [4, 4], color='black',linestyle='--', alpha=0.2)
            plt.plot([self.lower_lim, self.upper_lim], [-8, -8], color='black',linestyle='--', alpha=0.2)
            plt.plot([self.lower_lim, self.upper_lim], [8, 8], color='black',linestyle='--', alpha=0.2)
            plt.plot([self.lower_lim, self.upper_lim], [14, 14], color='black',linestyle='-', alpha=0.2)
            plt.plot([self.lower_lim, self.upper_lim], [-13, -13], color='black',linestyle='-', alpha=0.2)
            plt.xlim(self.lower_lim, self.upper_lim)
            plt.ylim(-13, 13)
            # plt.title('highway env')

            plt.tight_layout()
            plt.draw()
            plt.pause(0.00001)
            
            
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
                   



if __name__ == '__main__':
    my_sim = sim()
    my_sim.plot()
    



