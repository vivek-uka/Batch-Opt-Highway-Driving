
import matplotlib.pyplot as plt 
from numpy import *  
from matplotlib import rcParams
from matplotlib.cbook import boxplot_stats
import plotly.express as px
import plotly.graph_objects as go
import seaborn as sns
from matplotlib.ticker import FormatStrFormatter


ours = loadtxt("mpc_car_batch_data_11_goals.txt") # x y psi v w a j time loop
acado = loadtxt("mpc_car_acado_data_6_goals.txt")
acado_2 = loadtxt("mpc_car_acado_data_11_goals.txt")

plt.figure(figsize=(16.5,16.5))
plt.subplot(2,3,1)
sns.set_theme(style="whitegrid")
ax = sns.boxplot(data = [ours[:,7], acado[:,7], acado_2[:,7]], linewidth = 2)
ax.set_xticklabels(["Ours_11", "ACADO_6", "ACADO_11"], fontsize=10, weight='bold')
ax.set_ylabel(ylabel="seconds", weight='bold')
ax.set_xlabel(xlabel="(a)", weight='bold')
ax.set_title(label="Laptop Computation Time")#, fontsize=20)

# plt.figure(figsize=(6.5,6.5))
plt.subplot(2,3,2)
sns.set_theme(style="whitegrid")
ax = sns.boxplot(data = [ours[:,3], acado[:,3], acado_2[:,3]], linewidth = 2)
ax.set_xticklabels(["Ours_11", "ACADO_6", "ACADO_11"], fontsize=10, weight='bold')
ax.set_ylabel(ylabel="m/s", weight='bold')
ax.set_xlabel(xlabel="(b)", weight='bold')
ax.set_title(label="Velocity")#, fontsize=20)

# plt.figure(figsize=(6.5,6.5))
plt.subplot(2,3,3)
sns.set_theme(style="whitegrid")
ax = sns.boxplot(data = [ours[:,5], acado[:,5], acado_2[:,5]], linewidth = 2)
ax.set_xticklabels(["Ours_11", "ACADO_6", "ACADO_11"], fontsize=10, weight='bold')
ax.set_ylabel(ylabel="m/s2", weight='bold')
ax.set_xlabel(xlabel="(c)", weight='bold')
ax.set_title(label="Linear Acceleration")#, fontsize=20)

plt.subplot(2,3,4)
sns.set_theme(style="whitegrid")
ax = sns.boxplot(data = [ours[:,6], acado[:,6], acado_2[:,6]], linewidth = 2)
ax.set_xticklabels(["Ours_11", "ACADO_6", "ACADO_11"], fontsize=10, weight='bold')
ax.set_ylabel(ylabel="rad/s2", weight='bold')
ax.set_xlabel(xlabel="(d)", weight='bold')
ax.set_title(label="Ang Acceleration")#, fontsize=20)

plt.subplot(2,3,5)
sns.set_theme(style="whitegrid")
ax = sns.boxplot(data = [where(ours[:,1] > -8, abs(ours[:,1] - (-8)), 0),where(acado[:,1] > -8, abs(acado[:,1] - (-8)), 0), where(acado_2[:,1] > -8, abs(acado_2[:,1] - (-8)), 0)], linewidth = 2)
ax.set_xticklabels(["Ours_11", "ACADO_6", "ACADO_11"], fontsize=10, weight='bold')
ax.set_ylabel(ylabel="rad/s2", weight='bold')
ax.set_xlabel(xlabel="(d)", weight='bold')
ax.set_title(label="Right Lane Residual")#, fontsize=20)

mng = plt.get_current_fig_manager()
mng.full_screen_toggle()
plt.savefig('HSRL Ours 11, ACADO 6, ACADO 11',  bbox_inches='tight')

plt.show()