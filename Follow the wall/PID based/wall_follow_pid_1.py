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
from gazebo_msgs.msg import ModelState #for setting initial state of model
from gazebo_msgs.srv import SetModelState #for setting initial state of model
from sensor_msgs.msg import LaserScan #for laserscan

class trajectory:
    def __init__(self):
        self.lock = threading.Lock() #for lock thread
        self.rate=rospy.Rate(50) #sampelling rate
        self.dt = 0.0
        self.time_start = 0.0 #start time
        self.end = False
        #odom values
        self.cur_pos_x=0.0
        self.cur_pos_y=0.0
        self.cur_orient_x=0.0
        self.cur_orient_y=0.0
        self.cur_orient_z=0.0
        self.cur_orient_w=0.0
        self.cur_angular_z=0.0
        #scan callback
        self.ranges=[]
        self.intensities=[]
        range_a=0
        range_b=0
        range_front=0
        #initialization parameters
        self.flag = True
        self.y_error=0 # distance to goal 
        self.threshold=0.4  #set threshold value for distance to goal
        self.y_error_pre = 0  #for control gain
        self.control_signal_prev = 0  #for control gain
        self.goal_num = 1 #number of goals
        #for data logging
        self.log_counter = 0
        self.trajectory=[]
        #for generating map
        self.map_data_curr=[]
        self.map_data_prev=[0.0] * 20 #for vector_control
        # self.map_data_prev=[0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] #for pid_control
        self.iter=0
        #trajectory
        # distance=2*pi
        # velocity=0.3
        self.T=2*pi
        range_a_x,range_a_y=1.5,1.5
        #Defining publisher and subscriber
        self.pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=2, latch=False)
        self.sub_odom = rospy.Subscriber("/odom", Odometry, self.odometry_callback)
        self.sub_scan = rospy.Subscriber("/scan", LaserScan, self.scan_callback)


    def odometry_callback(self,msg):
        # read current position
        self.cur_pos_x = msg.pose.pose.position.x
        self.cur_pos_y = msg.pose.pose.position.y
        #read current orientation in quaternion form
        self.cur_orient_x = msg.pose.pose.orientation.x 
        self.cur_orient_y = msg.pose.pose.orientation.y
        self.cur_orient_z = msg.pose.pose.orientation.z
        self.cur_orient_w = msg.pose.pose.orientation.w
        #read velocity
        self.cur_angular_z= msg.twist.twist.angular.z

    def scan_callback(self,msg):
        self.ranges=list(msg.ranges)
        # while len(self.ranges) <1:  #pass if ranges is zero
        #     pass
        """deal with 'inf' values """
        max_range=4.0
        for angle in range(len(self.ranges)):
            if self.ranges[angle] == 0.0:
                self.ranges[angle]=max_range
            else:
                self.ranges[angle]=min(self.ranges[angle], max_range) #lidar range is (0.12-3.5)
        self.intensities=list(msg.intensities)

    def parking_point(self):
        self.lock.acquire
        curr_intensities=self.intensities
        curr_ranges=self.ranges
        # print(curr_ranges)
        #calculate continious intensity meeting threshold value
        threshold= -0.1
        curr_list_len=0
        angle=0
        list_len=10
        line_intensities=[]
        line_intensities_dict={}
        while angle<len(curr_intensities):
            if curr_intensities[angle] > threshold :
                curr_list_len +=1
                line_intensities.append(curr_intensities[angle])
                if curr_list_len == list_len:
                    line_intensities_dict.update({angle-9:line_intensities})
                    line_intensities=[]
                    curr_list_len =0
                angle +=1
            else:
                line_intensities=[]
                angle +=1
                curr_list_len=0
        # print(line_intensities_dict)
        # #get value of theta ray for required lenght of reflectiive tape
        required_len=0.5
        theta_end=int(1/360*len(curr_ranges))
        length_theta=[]
        for key in line_intensities_dict:  #key=angle
            length=0
            count=0
            print(len(line_intensities_dict[key]))
            #calculate lenght of line using cosine length formula for triangle
            while length < required_len and count<=len(line_intensities_dict[key]):
                count +=1
                length=sqrt(curr_ranges[key]**2+curr_ranges[key+theta_end]**2-2*curr_ranges[key]*curr_ranges[key+theta_end]*cos(radians(key+theta_end)))
                theta_end +=int(1/360*len(curr_ranges)) #increase theta by unit angle
            length_theta.append([theta_end,length])
        
        # print(line_intensities_dict={})
        print(length_theta)
        self.lock.release
        return length_theta

    def control(self):
        #control gain
        kp = 2.0
        ki = 0.1
        kd = 0.5

        self.lock.acquire()
        #quaternion to euler conversion
        cur_rpy = tftr.euler_from_quaternion((self.cur_orient_x, self.cur_orient_y, self.cur_orient_z, self.cur_orient_w))  
        self.cur_rot_z = cur_rpy[2]

        #check minimum value of range theta and 5
        while len(self.ranges) <1:  #pass if ranges is zero
            pass
        range_b=self.ranges[280] #range at 270deg
        range_a=self.ranges[310] #range at 270+30deg
        range_front=self.ranges[0]

        #get length and range angle of reflective tape
        # length_theta =self.parking_point()
        # print(length_theta)

        # calculate alpha
        alpha=arctan((range_a*cos(radians(30)-range_b))/(range_a*sin(radians(30))))

        #calculate projected lateral offset (CD)
        proj_dist=self.dt*0
        lateral_dist=range_b*cos(alpha) + proj_dist*sin(alpha)

        #error
        self.y_error=self.threshold - lateral_dist
        #calcuale delta and summation of errors
        diff_y_error=self.y_error - self.y_error_pre
        sum_y_error=self.y_error + self.y_error_pre

        #update y_error_pre and control_signal_prev for next iteration
        self.y_error_pre=self.y_error 

        #PID controller control signal
        linear_kp=kp*self.y_error #control distance for P
        linear_ki=ki*sum_y_error #control distance for I
        linear_kd=kd*diff_y_error/self.dt #control distance for D
        control_signal=linear_kp+linear_ki+linear_kd #total control signal

        # angle error correction for -pi, +pi, + and -
        if control_signal > pi:  
            control_signal =(2*pi)- control_signal
        elif control_signal < -pi:  
            control_signal =-control_signal - (2*pi) 
        else:            
            control_signal =control_signal  #to get higher w due to high error 

        #publish linear and angular velocity
        velocity=Twist()
        if range_front < self.threshold*2.5 and range_a <= 5:
            print('--1--')
            velocity.linear.x = 0.05
            velocity.angular.z= 0.3
        elif range_a >= 5 and range_b <= 5 :
            print('--2--')
            velocity.linear.x = 0.2
            velocity.angular.z= -0.3    
        elif range_a <5 and range_b <5:
            print('--3--')
            velocity.linear.x = 0.4 * abs(cos(control_signal))
            velocity.angular.z= min(control_signal, 3 ) if velocity.angular.z >0 else max(control_signal, -3)
        else:
            print('--4--')
            velocity.linear.x = 0
            velocity.angular.z= 0.3   
        
        #finally publish velocities
        self.pub_cmd_vel.publish(velocity)

        # logging once every 100 times (Gazebo runs at 1000Hz; we save it at 10Hz)
        self.log_counter += 1
        if self.log_counter == 5:
            self.log_counter = 0
            #plot the graph
            self.map_data_curr=[self.iter, self.cur_pos_x, self.cur_pos_y, self.y_error, velocity.linear.x, velocity.angular.z]
            self.plot(self.map_data_prev,self.map_data_curr)
            self.map_data_prev=self.map_data_curr
            self.iter += 1

        #printing values
        # print('Goal co-ordinates:', [round(elem,1) for elem in pos_des])
        print("Range front , a & b :",round(range_front,2) , '|', round(range_a,2) , '|', round(range_b,2))
        print("y_error and threshold:", round(self.y_error,2) , '|', round(self.threshold,2))
        print("Alpha:", round(alpha,3))
        # print("Control Signal:", round(control_signal,2))
        print('Linear velocity:', round(velocity.linear.x, 2))
        print('Angular velocity:', round(velocity.angular.z,2))
        print('-------------------------------------------------------')

        self.lock.release() #release lock

    def plot(self, map_data_prev,map_data_curr):
        plt.subplot(2,2,1) #first plot
        plt.title('Trajectory')
        plt.plot((map_data_prev[1],map_data_curr[1]), (map_data_prev[2],map_data_curr[2]),c='b')
        plt.grid(True)
        plt.subplot(2,2,2) #second plot
        plt.title('Trajectory error')
        plt.plot((map_data_prev[0],map_data_curr[0]),(map_data_prev[3],map_data_curr[3]),c='b') # before angle error
        plt.plot((map_data_prev[0],map_data_curr[0]),(0.0, 0.0),c='r') # corrected angle error
        plt.grid(True)
        plt.subplot(2,2,3) #third plot
        plt.title('Linear velocity')
        plt.plot((map_data_prev[0],map_data_curr[0]),(map_data_prev[4],map_data_curr[4]),c='g') #velocity linear
        plt.grid(True)
        plt.subplot(2,2,4) #fourth plot
        plt.title('Angular velocity')
        plt.plot((map_data_prev[0],map_data_curr[0]),(map_data_prev[5],map_data_curr[5]),c='y') #angular velocity
        plt.pause(.01) #time for plot to generate
        plt.grid(True)
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
        initial_position=[0.0,0.0,0.0]   #x ,y, theta_yaw_euler
        # self.set_postion(initial_position) 
    
        time_prev = 0.0
        self.time_start = rospy.get_time()
        try:
            while not rospy.on_shutdown(self.clean_shutdown):
                net_t = rospy.get_time() - self.time_start
                self.dt = net_t - time_prev
                time_prev = net_t
                # print('delta t:',self.time_start, net_t, self.dt)
                self.control()
                self.rate.sleep()       
        except rospy.ROSInterruptException:
            pass
        rospy.loginfo("System is shutting down. Stopping robot")

if __name__ == "__main__":
    rospy.init_node('trajectory_node')
    task1 = trajectory()
    task1.main()
