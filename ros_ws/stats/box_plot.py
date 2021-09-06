
import matplotlib.pyplot as plt 
from numpy import *  
from matplotlib import rcParams
from matplotlib.cbook import boxplot_stats
import plotly.express as px
import plotly.graph_objects as go
import seaborn as sns
from matplotlib.ticker import FormatStrFormatter


ours = loadtxt("batch.txt") # x y psi v w a j time loop
                            # 0 1 2   3 4 5 6  7     8
frenet = loadtxt("frenet.txt")
frenet = array([frenet]).reshape(int(frenet[-1]), 9)


plt.figure(figsize=(16.5,16.5))
plt.subplot(2,2,1)
sns.set_theme(style="whitegrid")
ax = sns.boxplot(data = [ours[:,7], frenet[:,7]], linewidth = 2)
ax.set_xticklabels(["Ours", "Frenet"], fontsize=10, weight='bold')
ax.set_ylabel(ylabel="seconds", weight='bold')
ax.set_xlabel(xlabel="(a)", weight='bold')
ax.set_title(label="Laptop Computation Time)")#, fontsize=20)

# plt.figure(figsize=(6.5,6.5))
plt.subplot(2,2,2)
sns.set_theme(style="whitegrid")
ax = sns.boxplot(data = [abs(ours[:,3] - 15), abs(frenet[:,3] - 15)], linewidth = 2)
ax.set_xticklabels(["Ours", "Frenet"], fontsize=10, weight='bold')
ax.set_ylabel(ylabel="m/s", weight='bold')
ax.set_xlabel(xlabel="(b)", weight='bold')
ax.set_title(label="Velocity residual")#, fontsize=20)

# plt.figure(figsize=(6.5,6.5))
plt.subplot(2,2,3)
sns.set_theme(style="whitegrid")
ax = sns.boxplot(data = [ours[:,5], frenet[:,5]], linewidth = 2)
ax.set_xticklabels(["Ours", "ACADO"], fontsize=10, weight='bold')
ax.set_ylabel(ylabel="m/s2", weight='bold')
ax.set_xlabel(xlabel="(c)", weight='bold')
ax.set_title(label="Linear Acceleration")#, fontsize=20)

plt.subplot(2,2,4)
sns.set_theme(style="whitegrid")
ax = sns.boxplot(data = [ours[:,6], frenet[:,6]], linewidth = 2)
ax.set_xticklabels(["Ours", "Frenet"], fontsize=10, weight='bold')
ax.set_ylabel(ylabel="rad/s2", weight='bold')
ax.set_xlabel(xlabel="(d)", weight='bold')
ax.set_title(label="Ang Acceleration")#, fontsize=20)

mng = plt.get_current_fig_manager()
mng.full_screen_toggle()

plt.show()
