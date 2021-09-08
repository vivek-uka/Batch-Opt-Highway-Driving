import matplotlib.pyplot as plt 
from numpy import *  
from matplotlib import rcParams
from matplotlib.cbook import boxplot_stats
import numpy
import plotly.express as px
import plotly.graph_objects as go
import pandas as pd
import seaborn as sns
from matplotlib.ticker import FormatStrFormatter


plt.figure(figsize=(16.5,16.5))
ours_2 = loadtxt("Copy/mpc_car_batch_data_2_goals.txt") # x y psi v w a j time loop
ours_4 = loadtxt("Copy/mpc_car_batch_data_4_goals.txt") # x y psi v w a j time loop
ours_6 = loadtxt("Copy/mpc_car_batch_data_6_goals.txt") # x y psi v w a j time loop
ours_8 = loadtxt("Copy/mpc_car_batch_data_8_goals.txt") # x y psi v w a j time loop
ours_10 = loadtxt("Copy/mpc_car_batch_data_10_goals.txt") # x y psi v w a j time loop
ours_12 = loadtxt("Copy/mpc_car_batch_data_12_goals.txt") # x y psi v w a j time loop
ours_14 = loadtxt("Copy/mpc_car_batch_data_14_goals.txt") # x y psi v w a j time loop
ours_16 = loadtxt("Copy/mpc_car_batch_data_16_goals.txt") # x y psi v w a j time loop
ours_20 = loadtxt("Copy/mpc_car_batch_data_20_goals.txt") # x y psi v w a j time loop

acado_2 = loadtxt("Copy/mpc_car_acado_2_goals.txt") # x y psi v w a j time loop
acado_4 = loadtxt("Copy/mpc_car_acado_4_goals.txt") # x y psi v w a j time loop
acado_6 = loadtxt("Copy/mpc_car_acado_6_goals.txt") # x y psi v w a j time loop
acado_8 = loadtxt("Copy/mpc_car_acado_8_goals.txt") # x y psi v w a j time loop
acado_10 = loadtxt("Copy/mpc_car_acado_10_goals.txt") # x y psi v w a j time loop
acado_12 = loadtxt("Copy/mpc_car_acado_12_goals.txt") # x y psi v w a j time loop
acado_14 = loadtxt("Copy/mpc_car_acado_14_goals.txt") # x y psi v w a j time loop
acado_16 = loadtxt("Copy/mpc_car_acado_16_goals.txt") # x y psi v w a j time loop
acado_20 = loadtxt("Copy/mpc_car_acado_20_goals.txt") # x y psi v w a j time loop


seconds_data = numpy.vstack((ours_2[:,7].reshape(ours_2[:,7].size,1), acado_2[:,7].reshape(acado_2[:,7].size,1),
                            ours_4[:,7].reshape(ours_4[:,7].size,1), acado_4[:,7].reshape(acado_4[:,7].size,1),
                            ours_6[:,7].reshape(ours_6[:,7].size,1), acado_6[:,7].reshape(acado_6[:,7].size,1),
                            ours_8[:,7].reshape(ours_8[:,7].size,1), acado_8[:,7].reshape(acado_8[:,7].size,1),
                            ours_10[:,7].reshape(ours_10[:,7].size,1), acado_10[:,7].reshape(acado_10[:,7].size,1),
                            ours_12[:,7].reshape(ours_12[:,7].size,1), acado_12[:,7].reshape(acado_12[:,7].size,1),
                            ours_14[:,7].reshape(ours_14[:,7].size,1), acado_14[:,7].reshape(acado_14[:,7].size,1),
                            ours_16[:,7].reshape(ours_16[:,7].size,1), acado_16[:,7].reshape(acado_16[:,7].size,1),
                            ours_20[:,7].reshape(ours_20[:,7].size,1), acado_20[:,7].reshape(acado_20[:,7].size,1),

))

method_data_2 = ["2_goals"]*(ours_2[:,7].size + acado_2[:,7].size)
method_data_4 = ["4_goals"]*(ours_4[:,7].size + acado_4[:,7].size)
method_data_6 = ["6_goals"]*(ours_6[:,7].size + acado_6[:,7].size)
method_data_8 = ["8_goals"]*(ours_8[:,7].size + acado_8[:,7].size)
method_data_10 = ["10_goals"]*(ours_10[:,7].size + acado_10[:,7].size)
method_data_12 = ["12_goals"]*(ours_12[:,7].size + acado_12[:,7].size)
method_data_14 = ["14_goals"]*(ours_14[:,7].size + acado_14[:,7].size)
method_data_16 = ["16_goals"]*(ours_16[:,7].size + acado_16[:,7].size)
method_data_20 = ["20_goals"]*(ours_20[:,7].size + acado_20[:,7].size)

method_data = method_data_2+ method_data_4+ method_data_6+ method_data_8+ method_data_10+ method_data_12+ method_data_14+ method_data_16+ method_data_20

# scenario_c = ["Cruise with IDM"]*ours_2[:,7].size
# scenario_h = ["High Speed Right Lane with IDM"]*ours_2[:,7].size
# scenario_rl = ["Right Lane with IDM"]*abal_rl[:,7].size
# scenario = scenario_c+scenario_h+scenario_rl
# scenario_c = ["Cruise with IDM"]*ours_c[:,7].size
# scenario_h = ["High Speed Right Lane with IDM"]*ours_h[:,7].size
# scenario_rl = ["Right Lane with IDM"]*ours_rl[:,7].size
# scenario = scenario+scenario_c+scenario_h+scenario_rl
# scenario_c = ["Cruise with IDM"]*acado_c[:,7].size
# scenario_h = ["High Speed Right Lane with IDM"]*acado_h[:,7].size
# scenario_rl = ["Right Lane with IDM"]*acado_rl[:,7].size
# scenario = scenario+scenario_c+scenario_h+scenario_rl
# scenario_c = ["Cruise with IDM"]*acado_2_c[:,7].size
# scenario_h = ["High Speed Right Lane with IDM"]*acado_2_h[:,7].size
# scenario_rl = ["Right Lane with IDM"]*acado_2_rl[:,7].size
# scenario = scenario+scenario_c+scenario_h+scenario_rl
# scenario_c = ["Cruise with IDM"]*frenet_c[:,7].size
# scenario_h = ["High Speed Right Lane with IDM"]*frenet_h[:,7].size
# scenario_rl = ["Right Lane with IDM"]*frenet_rl[:,7].size
# scenario = scenario+scenario_c+scenario_h+scenario_rl
# print(len(scenario))
print(len(method_data))
print(shape(seconds_data))
sec = [i[0] for i in seconds_data]
#print(sec)


dict = {"method":method_data, "seconds":sec}#, "scenario":scenario}
#df = pd.DataFrame(dict) 
    
# saving the dataframe 
#df.to_csv('GFG.csv') 
sns.set_theme(style="whitegrid")
ax = sns.boxplot(x="method", y="seconds",
                 data=dict,showfliers=False)
ax.set_ylabel(ylabel="m/s2", fontsize=20)
plt.setp(ax.get_legend().get_texts(), fontsize='15') 
ax.set_xticklabels(["Standard-MPC", "Ours", "ACADO_11_goals", "ACADO_6_goals", "Frenet"], fontsize=15)
ax.set_title(label="Linear Acceleration", fontsize=20)
#ax.set_xlabel(xlabel="Method", weight='bold')
#plt.savefig('Linear Acceleration vs Methods',  dpi=100)#bbox_inches='tight')
plt.show()