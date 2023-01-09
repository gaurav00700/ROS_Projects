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
from geometry_msgs.msg import Pose2D, Twist, Vector3
import graph_1 as graph

class control:
    def __init__(self,data):
        # rospy.init_node('contoller')
        self.lock = threading.Lock() #for lock thread
        #odom values
        self.cur_pos_x=data['cur_pos_x']
        self.cur_pos_y=data['cur_pos_y']
        self.cur_orient_x=data['cur_orient_x']
        self.cur_orient_y=data['cur_orient_y']
        self.cur_orient_z=data['cur_orient_z']
        self.cur_orient_w=data['cur_orient_w']
        #initialization parameters
        self.dist2goal=data['dist2goal']
        self.dist2goal_limit=0.02
        self.dist2goal_prev = data['dist2goal_prev']  #for control gain
        self.error_angle_prev = data['error_angle_prev']  #for control gain
        #for data logging
        self.log_counter = data['log_counter']
        self.trajectory=data['trajectory']
        #for generating map
        self.map_data_curr=data['map_data_curr']
        self.iter= data['iter']
        self.map_data_prev= data['map_data_prev'] #for derivative_control
        # self.map_data_prev=[0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] #for pid_control
        #trajectory
        self.T=2*pi #time period
        self.A_x,self.A_y=1.5,1.5 #amplitude
        self.omega_x, self.omega_y = 2 * pi / self.T , 2 * (2 * pi / self.T)
        #Defining publisher and subscriber
        self.pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1, latch=False)
        self.sub_odom = rospy.Subscriber("/odom", Odometry, self.odometry_callback)

    def odometry_callback(self,msg):
        self.lock.acquire()

        # read current position
        self.cur_pos_x = msg.pose.pose.position.x
        self.cur_pos_y = msg.pose.pose.position.y
        #read current orientation in quaternion form
        self.cur_orient_x = msg.pose.pose.orientation.x 
        self.cur_orient_y = msg.pose.pose.orientation.y
        self.cur_orient_z = msg.pose.pose.orientation.z
        self.cur_orient_w = msg.pose.pose.orientation.w
        
        self.lock.release()

    def pid_control(self, pos_des, dt):
        self.lock.acquire()
        #control gain for distance
        # kp_distance, ki_distance, kd_distance = 1 , 0.3 , 0.2 #for manual control
        kp_distance, ki_distance, kd_distance = 0.8 , 0.3 , 0.1 #for cosine function based 
        #control gain for angle
        # kp_angle, ki_angle, kd_angle = 2.2 , 0.4 , 0.5 #for manual control
        kp_angle, ki_angle, kd_angle = 1.5 , 0.2 , 0.6 #for cosine function based 

        # quaternion to euler conversion
        cur_rpy = tftr.euler_from_quaternion((self.cur_orient_x, self.cur_orient_y, self.cur_orient_z, self.cur_orient_w))  
        cur_rot_z = cur_rpy[2]

        #Calculation for distance to goal
        delta_x=pos_des[0]-self.cur_pos_x #delta x to goal 
        delta_y=pos_des[1]-self.cur_pos_y #delta y to goal
        dist2goal=sqrt(delta_x**2+delta_y**2) #distance to goal
        if dist2goal > self.dist2goal_limit:
            #angle error calculations
            angle2goal=arctan2(delta_y, delta_x) #angle to goal
            error_angle = cur_rot_z -angle2goal #error angle wrt angle to goal
            before_error_angle=error_angle  #for printing
            #angle error correction for -pi, +pi, + and -
            if error_angle > pi:  
                error_angle =(2*pi)- error_angle
            elif error_angle < -pi:  
                error_angle =-error_angle - (2*pi) 
            else:            
                error_angle =-error_angle  #to get higher w due to high error 

            #calcuale delta errors of distance and angle
            diff_dist2goal=dist2goal-self.dist2goal_prev
            diff_error_angle=error_angle-self.error_angle_prev

            #calculate sum of distance and angle
            sum_dist2goal=dist2goal+self.dist2goal_prev
            sum_error_angle=error_angle+self.error_angle_prev

            #PID controller
            # for distance
            linear_kp=kp_distance*dist2goal #control distance for P
            linear_ki=ki_distance*sum_dist2goal #control distance for I
            linear_kd=kd_distance*diff_dist2goal/dt #control distance for D
            control_signal_distance=linear_kp+linear_ki+linear_kd #total control signal
            #for angle
            angular_kp=kp_angle*error_angle  #control angle for P
            angular_ki=ki_angle*sum_error_angle #control angle for I
            angular_kd=kd_angle*diff_error_angle/dt  #control angle for D
            control_signal_angle=angular_kp+angular_ki+angular_kd

            #publish linear and angular velocity
            velocity=Twist() #calling velocity message to publish

            #linear velocity limits
            #cosine function based
            if control_signal_distance > 0.0:
                velocity.linear.x = min(control_signal_distance*abs(cos(error_angle)), 0.3)
            else:
                velocity.linear.x = max(control_signal_distance*abs(cos(error_angle)), -0.3)
            
            #manual control
            # if error_angle < pi/25 and error_angle > -pi/25: #+-0.13
            #     velocity.linear.x = min(control_signal_distance, 0.45)
            # elif error_angle < pi/15 and error_angle > -pi/15: #+-0.21
            #     velocity.linear.x = min(control_signal_distance, 0.35)
            # elif error_angle < pi/10 and error_angle > -pi/10: #+-0.31
            #     velocity.linear.x = min(control_signal_distance, 0.20)
            # else: # > +- 0.31
            #     velocity.linear.x = min(control_signal_distance, 0.15)

            #angular velocity limits
            if control_signal_angle > 0:
                velocity.angular.z=min(control_signal_angle, 4.0)
            else:
                velocity.angular.z=max(control_signal_angle,-4.0)
            
            #finally publishing velocities 
            self.pub_cmd_vel.publish(velocity)

            #update dist2goal_prev and error_angle_prev for next iteration
            self.dist2goal_prev=dist2goal 
            self.error_angle_prev=error_angle 

            # logging once every 100 times (Gazebo runs at 1000Hz; we save it at 10Hz)
            self.log_counter += 1
            if self.log_counter == 5 or dist2goal <= self.dist2goal_limit:
                self.log_counter = 0
                self.iter += 1
                self.trajectory.append([self.iter,self.cur_pos_x,pos_des[0],pos_des[1], self.cur_pos_y, dist2goal, before_error_angle, error_angle, velocity.linear.x, velocity.angular.z])

                #plot the graph
                self.map_data_curr=[self.iter, self.cur_pos_x, self.cur_pos_y, pos_des[0],pos_des[1], dist2goal, before_error_angle, error_angle, velocity.linear.x, velocity.angular.z]
                graph.plot(self.map_data_prev,self.map_data_curr)
                self.map_data_prev=self.map_data_curr

            #printing values
            print('Goal co-ordinates:', [round(elem,1) for elem in pos_des])
            print("Distance to goal and threshold:", round(dist2goal,2) , '|', round(self.dist2goal_limit,2))
            print("Current angle error:", round(error_angle,2))
            print('Linear velocity:', round(velocity.linear.x, 2))
            print('Angular velocity:', round(velocity.angular.z,2))
            print('-------------------------------------------------------')
        else:
            pass
        #send data for next loop from self.__dict__
        init_data=self.__dict__  
        self.lock.release() #release lock
        return init_data
