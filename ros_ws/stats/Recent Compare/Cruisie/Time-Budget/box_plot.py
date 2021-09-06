
import matplotlib.pyplot as plt 
from numpy import *  
from matplotlib import rcParams
from matplotlib.cbook import boxplot_stats
import plotly.express as px
import plotly.graph_objects as go
import seaborn as sns
from matplotlib.ticker import FormatStrFormatter


ours = loadtxt("mpc_car_batch_data_11_goals.txt") # v w a j time loop
acado = loadtxt("mpc_car_acado_data_6_goals.txt")
acado_2 = loadtxt("mpc_car_acado_data_11_goals.txt")
# acado_3 = loadtxt("mpc_car_acado_data_3_goals.txt")
# acado_6 = loadtxt("mpc_car_acado_data_6_goals.txt")
# acado_12 = loadtxt("mpc_car_acado_data_12_goals.txt")

# plt.figure(figsize=(6.5,6.5))
# # plt.subplot(2,3,1)
# sns.set_theme(style="whitegrid")
# ax = sns.boxplot(data = [acado_3[:,4], acado_6[:,4], acado_12[:,4]], linewidth = 2)
# ax.set_xticklabels(["ACADO_3_goals", "ACADO 6 goals", "ACADO_12_goals"], fontsize=10, weight='bold')
# ax.set_ylabel(ylabel="seconds", weight='bold')
# ax.set_xlabel(xlabel="", weight='bold')
# ax.set_title(label="Laptop Computation Time")#, fontsize=20)

plt.figure(figsize=(16.5,16.5))
plt.subplot(2,2,1)
sns.set_theme(style="whitegrid")
ax = sns.boxplot(data = [ours[:,4], acado[:,4], acado_2[:,4]], linewidth = 2)
ax.set_xticklabels(["Ours_11", "ACADO_6", "ACADO_11"], fontsize=10, weight='bold')
ax.set_ylabel(ylabel="seconds", weight='bold')
ax.set_xlabel(xlabel="(a)", weight='bold')
ax.set_title(label="Laptop Computation Time ")#, fontsize=20)

# plt.figure(figsize=(6.5,6.5))
plt.subplot(2,2,2)
sns.set_theme(style="whitegrid")
ax = sns.boxplot(data = [abs(ours[:,0] - 15), abs(acado[:,0] - 15), abs(acado_2[:,0] - 15)], linewidth = 2)
ax.set_xticklabels(["Ours_11", "ACADO_6", "ACADO_11"], fontsize=10, weight='bold')
ax.set_ylabel(ylabel="m/s", weight='bold')
ax.set_xlabel(xlabel="(b)", weight='bold')
ax.set_title(label="Velocity residual")#, fontsize=20)

# plt.figure(figsize=(6.5,6.5))
plt.subplot(2,2,3)
sns.set_theme(style="whitegrid")
ax = sns.boxplot(data = [ours[:,2], acado[:,2],acado_2[:,2]], linewidth = 2)
ax.set_xticklabels(["Ours_11", "ACADO_6", "ACADO_11"], fontsize=10, weight='bold')
ax.set_ylabel(ylabel="m/s2", weight='bold')
ax.set_xlabel(xlabel="(c)", weight='bold')
ax.set_title(label="Linear Acceleration")#, fontsize=20)

plt.subplot(2,2,4)
sns.set_theme(style="whitegrid")
ax = sns.boxplot(data = [ours[:,3], acado[:,3], acado_2[:,3]], linewidth = 2)
ax.set_xticklabels(["Ours_11", "ACADO_6", "ACADO_11"], fontsize=10, weight='bold')
ax.set_ylabel(ylabel="rad/s2", weight='bold')
ax.set_xlabel(xlabel="(d)", weight='bold')
ax.set_title(label="Ang Acceleration")#, fontsize=20)

mng = plt.get_current_fig_manager()
mng.full_screen_toggle()
# plt.savefig("500m_run.png", dpi=100)
plt.savefig('Cruisie Ours(11goals) vs ACADO9(6goals) vs ACADO(11goals)',  bbox_inches='tight')
# psi_ours = [0.0]
# for i in range(len(ours[:,1])):
#     psi_ours.append(psi_ours[i] + ours[i,1]*(0.08))
# plt.plot((psi_ours))

# psi_acado = [0.0]
# for i in range(len(ours[:,1])):
#     psi_acado.append(psi_acado[i] + acado[i,1]*(0.08))
# plt.plot((psi_acado))
plt.show()