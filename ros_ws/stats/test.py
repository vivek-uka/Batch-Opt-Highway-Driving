from shapely.geometry import Point
from shapely.affinity import scale, rotate
from descartes import PolygonPatch
from matplotlib import pyplot as plt
import numpy as np
import random

def create_ellipse(center, axes, inclination):
    
    p = Point(*center)
    c = p.buffer(1)
    ellipse = scale(c, *axes)
    ellipse = rotate(ellipse, inclination)
    return ellipse

ell1 = create_ellipse((0, 2), (0.5, 1), 0)
ell2 = create_ellipse((0, 0.001), (0.5, 1), 0)


e1 = PolygonPatch(ell1, color='green')
e2 = PolygonPatch(ell2, color='red')
polys = [e1]#, e2]


if ell1.intersects(ell2):
    intersect = ell1.intersection(ell2)
    x, y = (intersect.exterior.coords.xy)
    # print(len(x))
    inter = PolygonPatch(intersect, color='blue')
    polys.append(inter)






plt.figure()
ax = plt.gca()
[ax.add_artist(item) for item in polys]
plt.xlim(-2, 2)
plt.ylim(-2, 2)
# plt.show()

random.seed(999)

print(random.uniform(0,10))

print(random.uniform(0,10))





































# from numpy import *  
# from scipy.io import savemat
# from scipy.io.matlab.mio import loadmat

# data = loadtxt("data_mpc_xy.txt") 
# data2 = loadtxt("data_mpc_obs.txt")
# data3 = loadtxt("data_mpc_lamda.txt")
# data4 = loadtxt("data_mpc_d.txt")


# num_goal = 11
# num_obs = 6

# x = data[0:num_goal]
# y = data[num_goal:2*num_goal]
# psi = data[2*num_goal:3*num_goal]
# xdot = data[3*num_goal:4*num_goal]
# ydot = data[4*num_goal:5*num_goal]
# psidot = data[5*num_goal: 6*num_goal]
# xddot = data[6*num_goal: 7*num_goal]
# yddot = data[7*num_goal: 8*num_goal]
# psiddot = data[8*num_goal: 9*num_goal]
# v = data[9*num_goal: 10*num_goal]
# d_a = data[11*num_goal: 12*num_goal]
# alpha_a = data[12*num_goal: 13*num_goal]


# x_obs = data2[:num_obs]
# y_obs = data2[num_obs:]


# lamda_x = data3[:num_goal]
# lamda_y = data3[num_goal:2*num_goal]
# lamda_psi = data3[2*num_goal:]

# d_obs = data4[:num_goal]
# alpha_obs = data4[num_goal:]

# savedict = {
#     'x':x,
#     'y':y,
#     'psi':psi,
#     'xdot':xdot,
#     'ydot':ydot,
#     'psidot':psidot,
#     'xddot':xddot,
#     'yddot':yddot,
#     'psiddot':psiddot,
#     'v':v,
#     'd_a':d_a,
#     'alpha_a':alpha_a,
#     'd_obs':alpha_obs,
#     'x_obs':x_obs,
#     'y_obs':y_obs,
#     'lamda_x':lamda_x,
#     'lamda_y':lamda_y,
#     'lamda_psi':lamda_psi}

# savemat('data.mat',savedict)

# # randm = loadmat('data.mat')
# # print(randm)

