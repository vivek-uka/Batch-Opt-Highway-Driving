
import numpy as np
import circ
# if available import pylab (from matlibplot)
try:
    import matplotlib.pylab as plt
except ImportError:
    pass

def draw(obs_p, robo_p,agent_v, goal_p, obs_r, robo_r, goal_r,bot_path,x_axis_min, x_axis_max, y_axis_min, y_axis_max,counter,dt,save):

    plt.ion()
    plt.show()
    plt.clf()
    for k in np.arange(obs_p.shape[0]):
        circ.circ(obs_p[k,0], obs_p[k,1], obs_r,'blue')
        
        
    #%% animation for robo1
    #%plot the agent
    circ.circ(robo_p[0], robo_p[1], robo_r, 'red')
    plt.plot([-2000000,2000000],[8,8],'black')
    plt.plot([-2000000,2000000],[-0,0],'black')
    plt.plot([-2000000,2000000],[-8,-8],'black')
    plt.plot([-2000000,2000000], [4,4], linestyle='--', color='black')
    plt.plot([-2000000,2000000], [-4,-4], linestyle='--', color='black')
    plt.plot([-2000000,2000000], [12,12], linestyle='--', color='black')
    plt.plot([-2000000,2000000], [-12,-12], linestyle='--', color='black')

    plt.arrow(robo_p[0], robo_p[1], 0.5*agent_v[0], 0.5*agent_v[1],length_includes_head=True, head_width=0.3, head_length=0.2)
    #%plot agent's path
    plt.plot(bot_path[:,0], bot_path[:,1])
    #%% plot the goal
    #circ.circ(goal_p[0], goal_p[1], goal_r,'black')
    plt.xlim(robo_p[0]-30,robo_p[0]+30) 
    plt.ylim(-30,30)
    time=counter*dt
    plt.title('Time: %f'%time)
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.draw()
    plt.pause(0.001)
    if(save==1):
     plt.gcf().savefig('run/{}.png'.format( str(int(counter)).zfill(4)), dpi=300)

    return []

def draw1(obs_p,obs_v, robo_p,agent_v, goal_p, obs_r, robo_r, goal_r,bot_path,x_axis_min, x_axis_max, y_axis_min, y_axis_max,counter,dt,save, detected_obs):

    plt.ion()
    plt.show()
    plt.clf()
    for k in np.arange(obs_p.shape[0]):
        if(k<=detected_obs):
            circ.circ(obs_p[k,0], obs_p[k,1], obs_r,'orange')
        else:
            circ.circ(obs_p[k,0], obs_p[k,1], obs_r,'blue')
        plt.arrow(obs_p[k,0], obs_p[k,1], 0.25*obs_v[k,0], 0.25*obs_v[k,1],length_includes_head=True, head_width=0.3, head_length=0.2)
    
        
    #%% animation for robo1
    #%plot the agent
    circ.circ(robo_p[0], robo_p[1], robo_r, 'red')
    plt.plot([-2000000,2000000],[8,8],'black')
    plt.plot([-2000000,2000000],[-0,0],'black')
    plt.plot([-2000000,2000000],[-8,-8],'black')
    plt.plot([-2000000,2000000], [4,4], linestyle='--', color='black')
    plt.plot([-2000000,2000000], [-4,-4], linestyle='--', color='black')
    plt.plot([-2000000,2000000], [12,12], linestyle='--', color='black')
    plt.plot([-2000000,2000000], [-12,-12], linestyle='--', color='black')

    plt.arrow(robo_p[0], robo_p[1], 0.25*agent_v[0], 0.25*agent_v[1],length_includes_head=True, head_width=0.3, head_length=0.2)
    #%plot agent's path
    plt.plot(bot_path[:,0], bot_path[:,1])
    #%% plot the goal
    #circ.circ(goal_p[0], goal_p[1], goal_r,'black')
    plt.xlim(robo_p[0]-30,robo_p[0]+30) 
    plt.ylim(-30,30)
    time=counter*dt
    plt.title('Time: %f'%time)
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.draw()
    plt.pause(0.001)
    if(save==1):
     plt.gcf().savefig('run/{}.png'.format( str(int(counter)).zfill(4)), dpi=300)

    return []

def main():
    print("Hi!")
if __name__== "__main__":
    main()
