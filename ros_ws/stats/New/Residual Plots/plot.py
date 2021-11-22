import numpy as np
import matplotlib.pyplot as plt


optim_obs = np.loadtxt("res_obs_optim.txt")
optim_bounds = np.loadtxt("res_bounds_optim.txt")
optim_nonhol = np.loadtxt("res_nonhol_optim.txt")

mpc_obs = np.loadtxt("res_obs_mpc.txt")
mpc_bounds = np.loadtxt("res_bounds_mpc.txt")
mpc_nonhol = np.loadtxt("res_nonhol_mpc.txt")


optim_obs_x = optim_obs[:,:600]
optim_obs_y = optim_obs[:,600:]
optim_bounds_x = optim_bounds[:,:100]
optim_bounds_y = optim_bounds[:,100:]
optim_nonhol_x = optim_nonhol[:,:100]
optim_nonhol_y = optim_nonhol[:,100:]

mpc_obs_x = mpc_obs[:,:600]
mpc_obs_y = mpc_obs[:,600:]
mpc_bounds_x = mpc_bounds[:,:100]
mpc_bounds_y = mpc_bounds[:,100:]
mpc_nonhol_x = mpc_nonhol[:,:100]
mpc_nonhol_y = mpc_nonhol[:,100:]


norm_optim_obs_x = np.zeros(len(optim_obs_x))
norm_optim_obs_y = np.zeros(len(optim_obs_y))
norm_optim_bounds_x = np.zeros(len(optim_bounds_x))
norm_optim_bounds_y = np.zeros(len(optim_bounds_y))
norm_optim_nonhol_x = np.zeros(len(optim_bounds_x))
norm_optim_nonhol_y = np.zeros(len(optim_bounds_y))

norm_mpc_obs_x = np.zeros(len(mpc_obs_x))
norm_mpc_obs_y = np.zeros(len(mpc_obs_y))
norm_mpc_bounds_x = np.zeros(len(mpc_bounds_x))
norm_mpc_bounds_y = np.zeros(len(mpc_bounds_y))
norm_mpc_nonhol_x = np.zeros(len(mpc_bounds_x))
norm_mpc_nonhol_y = np.zeros(len(mpc_bounds_y))

best = 0
for i in range(len(optim_obs_x)):
    norm_optim_obs_x[i] = np.linalg.norm(optim_obs_x[i])
    norm_optim_obs_y[i] = np.linalg.norm(optim_obs_y[i])

    norm_optim_bounds_x[i] = np.linalg.norm(optim_bounds_x[i])
    norm_optim_bounds_y[i] = np.linalg.norm(optim_bounds_y[i])

    norm_optim_nonhol_x[i] = np.linalg.norm(optim_nonhol_x[i])
    norm_optim_nonhol_y[i] = np.linalg.norm(optim_nonhol_y[i])



for i in range(len(mpc_obs_x)):

    norm_mpc_obs_x[i] = np.linalg.norm(mpc_obs_x[i])
    norm_mpc_obs_y[i] = np.linalg.norm(mpc_obs_y[i])

    norm_mpc_bounds_x[i] = np.linalg.norm(mpc_bounds_x[i])
    norm_mpc_bounds_y[i] = np.linalg.norm(mpc_bounds_y[i])

    norm_mpc_nonhol_x[i] = np.linalg.norm(mpc_nonhol_x[i])
    norm_mpc_nonhol_y[i] = np.linalg.norm(mpc_nonhol_y[i])

norm_optim_obs_x = norm_optim_obs_x.reshape(100, 11)
norm_optim_obs_y = norm_optim_obs_y.reshape(100, 11)

norm_optim_bounds_x = norm_optim_bounds_x.reshape(100, 11)
norm_optim_bounds_y = norm_optim_bounds_y.reshape(100, 11)

norm_optim_nonhol_x = norm_optim_nonhol_x.reshape(100, 11)
norm_optim_nonhol_y = norm_optim_nonhol_y.reshape(100, 11)

temp = norm_optim_bounds_x[99] + norm_optim_bounds_y[99] + norm_optim_nonhol_x[99] + norm_optim_nonhol_y[99] + norm_optim_obs_x[99] + norm_optim_obs_y[99]
best = np.argmin(temp)

#
norm_mpc_obs_x = norm_mpc_obs_x.reshape(int(len(mpc_obs_x)/11), 11)
norm_mpc_obs_y = norm_mpc_obs_y.reshape(int(len(mpc_obs_x)/11), 11)

norm_mpc_bounds_x = norm_mpc_bounds_x.reshape(int(len(mpc_obs_x)/11), 11)
norm_mpc_bounds_y = norm_mpc_bounds_y.reshape(int(len(mpc_obs_x)/11), 11)

norm_mpc_nonhol_x = norm_mpc_nonhol_x.reshape(int(len(mpc_obs_x)/11), 11)
norm_mpc_nonhol_y = norm_mpc_nonhol_y.reshape(int(len(mpc_obs_x)/11), 11)

least_obs_x = []
least_obs_y = []
least_bounds_x = []
least_bounds_y = []
least_nonhol_x = []
least_nonhol_y = []


for i in range(len(norm_mpc_bounds_x)):
    temp = norm_mpc_obs_x[i] + norm_mpc_obs_y[i] + norm_mpc_nonhol_x[i] + norm_mpc_nonhol_y[i] + norm_mpc_bounds_x[i] + norm_mpc_bounds_y[i]
    index = np.argmin(temp)

    least_obs_x.append(norm_mpc_obs_x[i][index])
    least_obs_y.append(norm_mpc_obs_y[i][index])

    least_bounds_x.append(norm_mpc_bounds_x[i][index])
    least_bounds_y.append(norm_mpc_bounds_y[i][index])

    least_nonhol_x.append(norm_mpc_nonhol_x[i][index])
    least_nonhol_y.append(norm_mpc_nonhol_y[i][index])


# plt.figure(2)
# plt.plot(norm_optim_obs_x[:,best])
# plt.plot(norm_optim_obs_y[:,best])
# plt.plot(norm_optim_bounds_x[:,best])
# plt.plot(norm_optim_bounds_y[:,best])
# plt.plot(norm_optim_nonhol_x[:,best])
# plt.plot(norm_optim_nonhol_y[:,best])
# plt.legend([r"$\mathbf{||f_c^x||}$", r"$\mathbf{||f_c^y||}$", r"$\mathbf{||f_a^x||}$", r"$\mathbf{||f_a^y||}$", r"$\mathbf{||f_{nh}^x||}$", r"$\mathbf{||f_{nh}^y||}$"], prop={'weight':'bold'})
# plt.xlabel('Optimizer iterations')
# plt.ylabel('Value')
# plt.title('Constraint residuals')
# plt.grid()

plt.figure(3)
plt.plot(np.sqrt(norm_optim_obs_x[:,best]**2 + norm_optim_obs_y[:,best]**2))
plt.plot(np.sqrt(norm_optim_bounds_x[:,best]**2 + norm_optim_bounds_y[:,best]**2))
plt.plot(np.sqrt(norm_optim_nonhol_x[:,best]**2 + norm_optim_nonhol_y[:,best]**2))
# plt.legend([r"$\mathbf{||f_c||}$", r"$\mathbf{||f_a||}$", r"$\mathbf{||f_{nh}||}$"], prop={'weight':'bold'})
plt.legend(["Collision avoidance", "Non-holonomic", "Acceleration"], fontsize=20)
plt.xlabel('Optimizer iterations', fontsize=20)
plt.ylabel('Value', fontsize=20)
plt.title('Constraint residuals', fontsize=20)
plt.grid()

plt.show()