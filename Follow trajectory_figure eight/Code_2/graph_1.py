import matplotlib.pyplot as plt
import matplotlib.patches as mplt #for custon legend


def plot(map_data_prev,map_data_curr):
    #first plot
    plt.subplot(2,2,1) 
    plt.title('Trajectory (planned vs actual)')
    plt.plot((map_data_prev[1],map_data_curr[1]), (map_data_prev[2],map_data_curr[2]),c='b') #trajectory actual
    plt.plot((map_data_prev[3],map_data_curr[3]), (map_data_prev[4],map_data_curr[4]),c='g') #trajectory planned
    traj_act=mplt.Patch(color='blue',linewidth=0.1,label='Actual') #custom legend
    traj_plan=mplt.Patch(color='green',linewidth=0.1,label='Planned') #custom legend
    plt.legend(handles=[traj_act,traj_plan], loc=3)
    plt.grid(True)
    
    #second plot
    plt.subplot(2,2,2) 
    plt.title('Angle error(before vs after)')
    # plt.plot((map_data_prev[0],map_data_curr[0]),(map_data_prev[5],map_data_curr[5]),c='r')  #distance to goal
    plt.plot((map_data_prev[0],map_data_curr[0]),(map_data_prev[6],map_data_curr[6]),c='red') # before angle error
    plt.plot((map_data_prev[0],map_data_curr[0]),(map_data_prev[7],map_data_curr[7]),c='green') # corrected angle error
    err_before=mplt.Patch(color='red',linewidth=0.1,label='err_before') #custom legend
    err_after=mplt.Patch(color='green',linewidth=0.1,label='err_after') #custom legend
    plt.legend(handles=[err_before,err_after], loc=3)
    plt.grid(True)

    #third plot
    plt.subplot(2,2,3) 
    plt.title('Linear velocity')
    plt.plot((map_data_prev[0],map_data_curr[0]),(map_data_prev[8],map_data_curr[8]),c='green') #velocity linear
    # velocity=mplt.Patch(color='green',linewidth=0.1,label='velocity') #custom legend
    # plt.legend(handles=[velocity], loc=3)
    plt.grid(True)

    #fourth plot
    plt.subplot(2,2,4) 
    plt.title('Angular velocity')
    plt.plot((map_data_prev[0],map_data_curr[0]),(map_data_prev[9],map_data_curr[9]),c='y') #angular velocity
    plt.grid(True)
    
    plt.pause(.00001) #time for plot to generate
    plt.show
    plt.savefig('/turtlebot3_ws/scripts/trajectory/trajectory.png')