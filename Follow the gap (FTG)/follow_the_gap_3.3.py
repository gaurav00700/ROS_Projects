#!/usr/bin/env python

import rospy
import threading
import yaml
import tf
import tf.transformations as tftr
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion ,quaternion_from_euler
from geometry_msgs.msg import Point, Pose2D, Twist, Vector3
from math import atan2
import tf.transformations as tftr
from numpy import matrix, arctan, arctan2, sqrt, pi, sin, cos, linspace, arange, radians
import numpy as np
import time
import matplotlib.pyplot as plt
import matplotlib.patches as mplt #for custon lagend
from gazebo_msgs.msg import ModelState #for setting initial state of model
from gazebo_msgs.srv import SetModelState #for setting initial state of model
from sensor_msgs.msg import LaserScan #for laserscan

class follow_gap:
    def __init__(self):
        self.lock = threading.Lock() #for lock thread
        self.rate=rospy.Rate(50) #sampelling rate
        self.stuck_count = False
        #odom values
        self.cur_pos=0.0 #position
        self.cur_orient=0.0 #orientation in quaternions
        self.cur_w=0.0 #angular velocity
        self.cur_rot=0 #rotation z in euler
        #scan callback
        self.ranges=[]
        self.curr_ranges=[]
        self.gap_range_dict={}
        self.intensities=[]
        self.fov=0 #field of view per quadrant
        self.fov_pc=0.5 #field of view percentage per quadrant
        #initialization parameters
        self.error_angle=0 # distance to goal 
        self.threshold=0.4  #set threshold value for distance to goal
        self.error_angle_pre = 0  #for control gain
        self.control_signal_prev = 0  #for control gain
        self.goal_num = 1 #number of goals
        #for data logging
        self.counter = 0
        self.trajectory=[]
        #for generating map
        self.map_data_curr=[]
        self.map_data_prev=[0.0]*20
        self.iter=0
        #Defining publisher and subscriber
        self.pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=2, latch=False)
        rospy.Subscriber("/odom", Odometry, self.odometry_callback)
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)

    def odometry_callback(self,msg):
        # read current position
        self.cur_pos = msg.pose.pose.position
        #read current orientation in quaternion form
        self.cur_orient = msg.pose.pose.orientation
        #orientation
        cur_pos = tftr.euler_from_quaternion((self.cur_orient.x, self.cur_orient.y, self.cur_orient.z, self.cur_orient.w))
        self.cur_rot = cur_pos[2]
        #angular velocity
        self.cur_w=msg.twist.twist.angular

    def scan_callback(self,msg):
        #ranges
        self.ranges=list(msg.ranges)
        # while len(self.ranges) <1:  #pass if ranges is zero
        #     pass
        """deal with 'inf' values """
        max_range=4.0
        #replace "inf" and "0.0" values with max_range
        for angle in range(len(self.ranges)):
            if self.ranges[angle] == 0.0:
                self.ranges[angle]=max_range
            else:
                self.ranges[angle]=min(self.ranges[angle], max_range) #lidar range is (0.12-3.5)
        
        #intensities
        self.intensities=list(msg.intensities)

    def gap_finder(self):
        self.lock.acquire
        #check minimum value of range theta and 5
        while len(self.ranges) <1:  #pass if ranges is zero
            pass
        
        #current ranges for front field of view
        self.fov=int((len(self.ranges)/4)*self.fov_pc) # field of view 
        self.curr_ranges=self.ranges[(len(self.ranges)-self.fov):(len(self.ranges)-1)] + self.ranges[0:(self.fov+1)]
        
        #calculate continious range values meeting requirements
        list_lim=9
        range_lim= 2.3
        range_diff_lim=0.15
        line_range_list=[]     
        self.gap_range_dict={}
        for index in range (len(self.curr_ranges)):
            range_diff=abs(self.curr_ranges[index]-self.curr_ranges[index-1]) #difference for controlling change in range values
            if self.curr_ranges[index] > range_lim and range_diff < range_diff_lim : #check for range limit and range diff limit
                line_range_list.append(self.curr_ranges[index]) #save ranges in a list
                if len(line_range_list) >= list_lim: #save range list in range dictionary
                    self.gap_range_dict.update({index+1-len(line_range_list) : line_range_list})
            else:
                line_range_list=[] #reset list
            # print(range_diff, "|", range_diff_lim)
        # print(self.gap_range_dict)
        
        #calculate length and angle of largest gap
        start_theta, range_theta, gap_len= [], [], [] 
        for key in self.gap_range_dict:  #key=angle
            a=self.gap_range_dict[key][0] #range length of starting
            b=self.gap_range_dict[key][len(self.gap_range_dict[key])-1] #range length of end theta
            theta=radians(len(self.gap_range_dict[key])) #angle of continious ranges
            # print(a,b, theta)
            length=sqrt(a**2 + b**2 - 2*a*b*cos(theta)) #lenght of continious ranges using cosine formula
            start_theta.append(key)
            range_theta.append(len(self.gap_range_dict[key]))
            gap_len.append(length)
        length_n_theta=[start_theta, range_theta, gap_len]  #add lists into one

        self.lock.release
        return length_n_theta

    def control(self, dt):
        self.lock.acquire()
        #control gains for angle error
        kp=1.5
        ki=0.2
        kd=0.5
        #get length and range angle of gap
        length_n_theta =self.gap_finder()

        #error angle if empty list of gap length 
        if len(length_n_theta[2]) > 0 :
            max_length=max(length_n_theta[2]) #maxinum gap length
            index_max=length_n_theta[2].index(max_length) #index of max length
            theta_max_len=length_n_theta[0][index_max] #starting range angle of max length
            range_max_len=length_n_theta[1][index_max] #range angle of max length
            #total value of angle
            total_theta=theta_max_len+range_max_len/2
            self.error_angle=total_theta-self.fov
        else: 
            self.error_angle=self.fov
        #normalizer error b/w -1 and +1
        self.error_angle=(self.error_angle-0)/(self.fov*2-0)

        #calcuale delta and summation of errors
        diff_error_angle=self.error_angle - self.error_angle_pre
        sum_error_angle=self.error_angle + self.error_angle_pre
        #update y_error_pre and control_signal_prev for next iteration
        self.error_angle_pre=self.error_angle
        #PID controller control signal
        linear_kp=kp*self.error_angle #control distance for P
        linear_ki=ki*sum_error_angle #control distance for I
        linear_kd=kd*diff_error_angle/dt #control distance for D
        control_signal=linear_kp+linear_ki+linear_kd #total control signal

        # publish linear and angular velocity
        velocity=Twist()        
        #linear velocity in case of crash/stuck
        if self.stuck_count ==True:
            velocity.linear.x = -0.5
            self.stuck_count = False
        elif len(length_n_theta[2])>0:
            velocity.linear.x = 0.6*abs(cos(control_signal))
        else:
            velocity.linear.x = 0.0
            self.stuck_count = True
        
        # velocity.linear.x = 0.6 * abs(cos(control_signal))
        velocity.angular.z= control_signal
        #finally publish velocities
        self.pub_cmd_vel.publish(velocity)

        #graph plotting and logging
        #dealing with ranges data
        x_range, y_range = [], []
        for i in range (len(self.curr_ranges)):
            x=self.curr_ranges[i]*cos(radians(i))
            y=self.curr_ranges[i]*sin(radians(i))
            # rotation matrix
            # x = (x * cos(self.cur_rot)) - (y * sin(self.cur_rot)) 
            # y = (x * sin(self.cur_rot)) + (y * cos(self.cur_rot))
            x_range.append(x)
            y_range.append(y)
        #dealing with gap ranges data 
        x_gap, y_gap = [] , []
        for key in self.gap_range_dict:
            for i in range (len(self.gap_range_dict[key])):
                x_gap_i=self.gap_range_dict[key][i]*cos(radians(key+i))
                y_gap_i=self.gap_range_dict[key][i]*sin(radians(key+i))
                # rotation matrix
                # x_gap_i = (x_gap_i * cos(self.cur_rot)) - (y_gap_i * sin(self.cur_rot)) 
                # y_gap_i = (x_gap_i * sin(self.cur_rot)) + (y_gap_i * cos(self.cur_rot))
                x_gap.append(x_gap_i)
                y_gap.append(y_gap_i)
        #maxinum gap length
        try:
            x_move=max(self.curr_ranges[int(total_theta)]*cos(radians(total_theta)),0) #angle for gap_dict should be integer
            y_move=max(self.curr_ranges[int(total_theta)]*sin(radians(total_theta)),0) #angle for gap_dict should be integer
        except:
            x_move=0
            y_move=0

        print('List of start, range and length:',length_n_theta[0], length_n_theta[1], np.around(length_n_theta[2], 2) )
        # print('Maximum length',round(max_length,2))
        # print('Angle for max length:',round(theta_max_len,2))
        # print('Range of angle:',round(range_max_len,2))
        print('Error Angle:',round(self.error_angle,2))
        print('Control signal:', round(control_signal,2))
        print('Linear and Angluar velocity:',round(velocity.linear.x,2) ,'|', round(velocity.angular.z,2))
        print('------------------------------------------------------------------')
        
        self.counter +=1
        self.map_data_curr=[self.counter, x_range, y_range, x_gap, y_gap ,x_move, y_move ,velocity.linear.x, velocity.angular.z ]
        self.plot(self.map_data_prev,self.map_data_curr) #seld data for plotting
        self.map_data_prev=self.map_data_curr #save data for next loop
        self.lock.release() #release lock

    def plot(self, map_data_pre, map_data_curr):
        # plt.clf()
        #first plot
        plt.subplot(1,2,1).clear() #clear map
        plt.title('Movement direction')
        plt.scatter(map_data_curr[1],map_data_curr[2],c='red', s= 2, label='front ranges')
        plt.scatter(map_data_curr[3],map_data_curr[4], c='green', s=3, label='selected gaps')
        plt.arrow(0,0,(map_data_curr[5]-0.2),(map_data_curr[6]-0.2), head_width=0.05,width = 0.01, shape='full', ec='blue')
        plt.xlim(0.0,4.25) 
        plt.ylim(0.0,4.25) 
        plt.legend(loc=3)
        plt.grid(True)
        #second plot
        plt.subplot(1,2,2) 
        plt.title('Linear & Angular vel')
        plt.plot((map_data_pre[0], map_data_curr[0]),(map_data_pre[7], map_data_curr[7]), c='blue') #linear velocity
        plt.plot((map_data_pre[0], map_data_curr[0]),(map_data_pre[8], map_data_curr[8]), c='green') #angular velocity
        # plt.text(0, 1.5, 'Text', style='italic',bbox={'facecolor': 'white', 'alpha': 0.1, 'pad': 1}) #showing text
        plt.ylim(-2.0,2.0) 
        l_v=mplt.Patch(color='blue',linewidth=0.1,label='linear vel') #custom legend
        w_v=mplt.Patch(color='green',linewidth=0.1,label='angular vel') #custom legend
        plt.legend(handles=[l_v,w_v], loc=3)
        plt.grid(True)
        plt.pause(0.0001) #time for plot to generate
        plt.show
        # plt.savefig('/turtlebot3_ws/scripts/parking/trajectory.png')

    def set_postion(self,position):
        state_msg = ModelState()
        state_msg.model_name = 'turtlebot3_burger'
        # initial_position=[1.0,0.0,0.0]  #x,y,theta_yaw_euler
        q=quaternion_from_euler(0, 0, position[2]) #convert euler to quaternion form
        state_msg.pose.position.x = position[0]
        state_msg.pose.position.y = position[1]
        state_msg.pose.position.z = 0
        #four quaternions (q1,q2,q3,q4)
        state_msg.pose.orientation.x = q[0]
        state_msg.pose.orientation.y = q[1]
        state_msg.pose.orientation.z = q[2]
        state_msg.pose.orientation.w = q[3]

        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        pos=set_state(state_msg)
        print ('Position set to', position ,'...!!!!!!')

    def clean_shutdown(self): #press ctrl+C
        velocity=Twist()
        velocity.linear.x=0 
        velocity.angular.z=0
        self.pub_cmd_vel.publish(velocity) 

    def main(self):  
        rospy.loginfo('Task started...Press "Ctrl + C" to terminate..!')
        #Set starting position (x, y , theta)
        initial_position=[-6.0,-7.0,0.0]   #x ,y, theta_yaw_euler
        self.set_postion(initial_position)
        time_prev = 0.00
        time_start = rospy.get_time()
        try:
            while not rospy.on_shutdown(self.clean_shutdown):
                net_t = rospy.get_time() - time_start
                dt = net_t - time_prev
                time_prev = net_t
                if dt > 0:
                    self.control(dt)
                self.rate.sleep()       
        except rospy.ROSInterruptException:
            pass
        rospy.loginfo("System is shutting down. Stopping Robot")

if __name__ == "__main__":
    rospy.init_node('follow_the_gap')
    task1 = follow_gap()
    task1.main()
