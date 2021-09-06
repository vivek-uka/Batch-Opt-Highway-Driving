
import matplotlib.pyplot as plt 
from numpy import *  
from matplotlib import rcParams
from matplotlib.cbook import boxplot_stats
import plotly.express as px
import plotly.graph_objects as go
import seaborn as sns
from matplotlib.ticker import FormatStrFormatter


ours = loadtxt("mpc_car_batch_data_11_goals.txt") # v w a j time loop
acado = loadtxt("mpc_car_acado_data_11_goals.txt")


plt.figure(figsize=(16.5,16.5))
plt.subplot(2,2,1)
sns.set_theme(style="whitegrid")
ax = sns.boxplot(data = [ours[:,4], acado[:,4]], linewidth = 2)
ax.set_xticklabels(["Ours", "ACADO"], fontsize=10, weight='bold')
ax.set_ylabel(ylabel="seconds", weight='bold')
ax.set_xlabel(xlabel="(a)", weight='bold')
ax.set_title(label="Laptop Computation Time (11 goals)")#, fontsize=20)

# plt.figure(figsize=(6.5,6.5))
plt.subplot(2,2,2)
sns.set_theme(style="whitegrid")
ax = sns.boxplot(data = [abs(ours[:,0] - 15), abs(acado[:,0] - 15)], linewidth = 2)
ax.set_xticklabels(["Ours", "ACADO"], fontsize=10, weight='bold')
ax.set_ylabel(ylabel="m/s", weight='bold')
ax.set_xlabel(xlabel="(b)", weight='bold')
ax.set_title(label="Velocity residual")#, fontsize=20)

# plt.figure(figsize=(6.5,6.5))
plt.subplot(2,2,3)
sns.set_theme(style="whitegrid")
ax = sns.boxplot(data = [ours[:,2], acado[:,2]], linewidth = 2)
ax.set_xticklabels(["Ours", "ACADO"], fontsize=10, weight='bold')
ax.set_ylabel(ylabel="m/s2", weight='bold')
ax.set_xlabel(xlabel="(c)", weight='bold')
ax.set_title(label="Linear Acceleration")#, fontsize=20)

plt.subplot(2,2,4)
sns.set_theme(style="whitegrid")
ax = sns.boxplot(data = [ours[:,3], acado[:,3]], linewidth = 2)
ax.set_xticklabels(["Ours", "ACADO"], fontsize=10, weight='bold')
ax.set_ylabel(ylabel="rad/s2", weight='bold')
ax.set_xlabel(xlabel="(d)", weight='bold')
ax.set_title(label="Ang Acceleration")#, fontsize=20)

mng = plt.get_current_fig_manager()
mng.full_screen_toggle()
plt.savefig('Cruisie Ours(11goals) vs ACADO(11goals)',  bbox_inches='tight')
plt.show()