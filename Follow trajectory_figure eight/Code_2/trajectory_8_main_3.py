#! /usr/bin/env python3

import rospy
import threading
import yaml
import tf
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Point, Twist
from math import atan2
import tf.transformations as tftr
from numpy import matrix, arctan2, sqrt, pi, sin, cos, linspace, arange
import numpy as np
import time
import matplotlib.pyplot as plt
from gazebo_msgs.msg import ModelState #for setting initial state of model
from gazebo_msgs.srv import SetModelState #for setting initial state of model
# import controller_2 as control
from controller_3 import *


# global dist2goal , dist2goal_prev
# dist2goal, dist2goal_prev=1.0, 0.0

class trajectory(control):

    def __init__(self):
        rospy.init_node('trajectory_node')
        self.lock = threading.Lock() #for lock thread
        self.rate=rospy.Rate(18) #sampelling rate
        #trajectory
        distance=2*pi
        velocity=0.1
        self.T=(distance/velocity)
        self.A_x,self.A_y=1.5,1.5
        #Defining publisher and subscriber
        self.pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1, latch=False)
    
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
        print ('Position set to', position,'...!!!!!!')

    def clean_shutdown(self): #press ctrl+C
        velocity=Twist()
        velocity.linear.x=0 
        velocity.angular.z=0
        self.pub_cmd_vel.publish(velocity)

    def main(self):  
        rospy.loginfo('Task started...Press "Ctrl + C" to terminate..!')
        #Set starting position (x, y , theta)
        initial_position=[0.0,0.0,0.0]   #x ,y, theta_yaw_euler
        self.set_postion(initial_position)
        
        #for initialization values
        init_data={'cur_pos_x': 0.0, 'cur_pos_y': 0.0, 'cur_orient_x': 0.0, 'cur_orient_y': 0.0, 'cur_orient_z': 0.0, 'cur_orient_w': 0.0, 
        'dist2goal': 1, 'dist2goal_prev': 1, 'error_angle_prev': 0.0, 'log_counter': 0, 'trajectory': [], 'map_data_curr': [], 'iter': 0, 
        'map_data_prev': [0]*20}
        #Trajectory related
        time_prev = 0.0
        time_start = rospy.get_time()
        loop_count=1
        t_list = arange(0, self.T*1.1, self.T*0.001) #start , T, t
        try:
            while not rospy.on_shutdown(self.clean_shutdown) and loop_count<=len(t_list):
                for t in t_list:
                    net_t = rospy.get_time() - time_start
                    dt = net_t - time_prev
                    time_prev = net_t
                    # calculate trajectory point and theta
                    x = self.A_x*sin(2*pi/self.T*t)
                    y = self.A_y*sin(4*pi/self.T*t)
                    theta = arctan2(y,x)
                    pos_des=[x,y,theta]
                    # send points to controller 
                    controller=control(init_data)
                    init_data=controller.pid_control(pos_des,dt)
                    # controller.derivative_control(t) #derivative control
                    
                    loop_count +=1     
                    self.rate.sleep()
                
                #saving log file into csv
                np.savetxt('/turtlebot3_ws/scripts/trajectory/trajectory_log.csv',np.array(init_data['trajectory']),delimiter=',', fmt='%f', 
                header="itr_n,pos_cur_x,pos_cur_y,pos_des_x,pos_des_y,dist2goal,theta_err_before,theta_err_after,vel_x,ang_vel_z", comments='')
                rospy.loginfo("Trajectory_log saved at /turtlebot3_ws/scripts/trajectory")
        
        except rospy.ROSInterruptException:
            pass
        rospy.loginfo("System is shutting down. Stopping robot")

if __name__ == "__main__":
    # rospy.init_node('trajectory_node')
    task1 = trajectory()
    task1.main()