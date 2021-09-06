
import matplotlib.pyplot as plt 
from numpy import *  
from matplotlib import rcParams
from matplotlib.cbook import boxplot_stats
import plotly.express as px
import plotly.graph_objects as go
import seaborn as sns
from matplotlib.ticker import FormatStrFormatter


ours = loadtxt("mpc_car_batch_data_11_goals.txt") # x y psi v w a j time loop index
acado = loadtxt("mpc_car_acado_data_11_goals.txt")
acado_2 = loadtxt("mpc_car_acado_data_6_goals.txt")
frenet = loadtxt("car_frenet_data.txt")
frenet = array([frenet]).reshape(int(frenet[-2]), 10)

plt.figure(figsize=(16.5,16.5))
plt.subplot(2,2,1)
sns.set_theme(style="whitegrid")
ax = sns.boxplot(data = [ours[:,7], acado[:,7], acado_2[:,7], frenet[:,7]], linewidth = 2 ,showfliers=False)
ax.set_xticklabels(["Ours", "ACADO_11_goals", "ACADO_6_goals", "Frenet"], fontsize=10, weight='bold')
ax.set_ylabel(ylabel="seconds", weight='bold')
ax.set_xlabel(xlabel="(a)", weight='bold')
ax.set_title(label="Laptop Computation Time")#, fontsize=20)

# plt.figure(figsize=(6.5,6.5))
plt.subplot(2,2,2)
sns.set_theme(style="whitegrid")
# ax = sns.boxplot(data = [ours[:,8]*0.08, acado[:,8]*0.08, acado_2[:,8]*0.08], linewidth = 2 ,showfliers=False)
ax = sns.barplot(x =["Ours", "ACADO_11_goals", "ACADO_6_goals", "Frenet"], y = [ours[-1][-2]*0.08, acado[-1][-2]*0.08, acado_2[-1][-2]*0.08, frenet[-1][-2]*0.08])
ax.set_xticklabels(["Ours", "ACADO_11_goals", "ACADO_6_goals", "Frenet"], fontsize=10, weight='bold')
ax.set_ylabel(ylabel="seconds", weight='bold')
ax.set_xlabel(xlabel="(b)", weight='bold')
ax.set_title(label="Time to merge")#, fontsize=20)

# plt.figure(figsize=(6.5,6.5))
plt.subplot(2,2,3)
sns.set_theme(style="whitegrid")
ax = sns.boxplot(data = [ours[:,5], acado[:,5], acado_2[:,5], frenet[:,5]], linewidth = 2 ,showfliers=False)
ax.set_xticklabels(["Ours", "ACADO_11_goals", "ACADO_6_goals","Frenet"], fontsize=10, weight='bold')
ax.set_ylabel(ylabel="m/s2", weight='bold')
ax.set_xlabel(xlabel="(c)", weight='bold')
ax.set_title(label="Linear Acceleration")#, fontsize=20)

plt.subplot(2,2,4)
sns.set_theme(style="whitegrid")
ax = sns.boxplot(data = [ours[:,6], acado[:,6], acado_2[:,6], frenet[:,6]], linewidth = 2 ,showfliers=False)
ax.set_xticklabels(["Ours", "ACADO_11_goals", "ACADO_6_goals", "Frenet"], fontsize=10, weight='bold')
ax.set_ylabel(ylabel="rad/s2", weight='bold')
ax.set_xlabel(xlabel="(d)", weight='bold')
ax.set_title(label="Ang Acceleration")#, fontsize=20)

# plt.subplot(2,3,5)
# sns.set_theme(style="whitegrid")
# ax = sns.boxplot(data = [where(ours[:,1] > -8, abs(ours[:,1] - (-8)), 0),where(acado[:,1] > -8, abs(acado[:,1] - (-8)), 0)], linewidth = 2)
# ax.set_xticklabels(["Ours", "ACADO"], fontsize=10, weight='bold')
# ax.set_ylabel(ylabel="rad/s2", weight='bold')
# ax.set_xlabel(xlabel="(d)", weight='bold')
# ax.set_title(label="Right Lane Residual")#, fontsize=20)

mng = plt.get_current_fig_manager()
mng.full_screen_toggle()
plt.savefig('RightLane Ours(11goals) vs ACADO(11goals) vs ACADO(6goals) vs Frenet',  bbox_inches='tight')
plt.show()