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
from geometry_msgs.msg import Pose2D, Twist, Vector3
from gazebo_msgs.msg import ModelState  #for setting initial state of model
from gazebo_msgs.srv import SetModelState #for setting initial state of model

#control gain for distance
kp_distance = 1
ki_distance = 0.3
kd_distance = 0.2

#control gain for angle
kp_angle = 2.2
ki_angle = 0.4
kd_angle = 0.2

class control:
    # global dist2goal, dist2goal_prev
    def __init__(self,data):
        # rospy.init_node('contoller')
        self.lock = threading.Lock() #for lock thread
        # self.rate=rospy.Rate(60) #sampelling rate
        #Time values
        # self.time_prev=0.0
        # self.dt = 0.0
        # self.time_start = 0.0 #start time
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
        # self.lock.acquire()
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
            velocity.linear.x = control_signal_distance
            # velocity.linear.x = min(control_signal_distance, 0.15) #velocity min of control signal distance or 0.1        
            if error_angle > pi/4 or error_angle < -pi/4: #linear velocity limit for high angle error
                velocity.linear.x = min(control_signal_distance, 0.15)
            elif error_angle < pi/25 and error_angle > -pi/25:
                velocity.linear.x = min(control_signal_distance, 0.45)
            else:
                velocity.linear.x = min(control_signal_distance, 0.30)

            #angular velocity limits
            velocity.angular.z=control_signal_angle #net control gain for angular velocity
            # if velocity.angular.z > 0:
            #     velocity.angular.z=min(velocity.angular.z,5)
            # else:
            #     velocity.angular.z=max(velocity.angular.z,-5)
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
                self.plot(self.map_data_prev,self.map_data_curr)
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
        # self.lock.release() #release lock
        return init_data

    def derivative_control(self,t):
            self.lock.acquire()
            # calculate trajectory point and orientation
            x = self.A_x*sin(self.omega_x*t)
            y = self.A_y*sin(self.omega_y*t)
            theta = arctan2(y,x)
            #quaternion to euler conversion
            cur_rpy = tftr.euler_from_quaternion((self.cur_orient_x, self.cur_orient_y, self.cur_orient_z, self.cur_orient_w))  
            self.cur_rot_z = cur_rpy[2]
            #Calculation for distance to goal
            delta_x=x-self.cur_pos_x #delta x to goal 
            delta_y=y-self.cur_pos_y #delta y to goal
            self.dist2goal=sqrt(delta_x**2+delta_y**2) #distance to goal
            angle2goal=arctan2(delta_y, delta_x) #angle to goal
            error_angle = self.cur_rot_z -angle2goal #error angle wrt angle to goal

            # calculate linear velocity
            v_x = self.A_x*cos(self.omega_x*t)*(self.omega_x) #xdot
            v_y = self.A_y*cos(self.omega_y*t)*(self.omega_y) #ydot
            v = sqrt(v_x**2+v_y**2)

            # calculate linear acceleration
            a_x = -self.A_x*sin(self.omega_x*t)*((self.omega_x)**2) #xdotdot
            a_y = -self.A_y*sin(self.omega_y*t)*((self.omega_y)**2) #ydotdot
            
            # calculate angular velocity
            # w = (a_y*v_x - a_x*v_y) / (v_x**2 + v_y**2)
            w = (v_x*y-v_y*x)/(x**2+y**2)

            # publish control data
            velocity=Twist() #calling velocity message to publish
            velocity.linear.x = v_x
            velocity.linear.y = v_y
            velocity.angular.z = w
            self.pub_cmd_vel.publish(velocity)
            # self.pub_cmd_vel.publish(Vector3(v_x, v_y, 0.), Vector3(0., 0., w))
            
            #Map related data
            self.map_data_curr=[self.iter, x, y, theta, self.cur_pos_x, self.cur_pos_y, v_x, v_y, v, w]
            # self.plot(self.map_data_prev,self.map_data_curr)
            plt.subplot(1,3,1)
            plt.title('Trajectory (planned vs actual)')
            plt.plot((self.map_data_prev[1],self.map_data_curr[1]),(self.map_data_prev[2],self.map_data_curr[2]),c='g')
            plt.plot((self.map_data_prev[4],self.map_data_curr[4]),(self.map_data_prev[5],self.map_data_curr[5]),c='b')
            plt.subplot(1,3,2)
            plt.title('Trajectory theta')
            plt.plot((self.map_data_prev[0],self.map_data_curr[0]),(self.map_data_prev[3],self.map_data_curr[3]),c='y')
            plt.subplot(1,3,3)
            plt.title('Velocity (V_x vs V_y vs V vs W)')
            plt.plot((self.map_data_prev[0],self.map_data_curr[0]),(self.map_data_prev[6],self.map_data_curr[6]),c='y') #v_x
            plt.plot((self.map_data_prev[0],self.map_data_curr[0]),(self.map_data_prev[7],self.map_data_curr[7]),c='g') #v_y
            plt.plot((self.map_data_prev[0],self.map_data_curr[0]),(self.map_data_prev[8],self.map_data_curr[8]),c='b') #v
            plt.plot((self.map_data_prev[0],self.map_data_curr[0]),(self.map_data_prev[9],self.map_data_curr[9]),c='r') #w
            plt.pause(.0001)
            plt.savefig('/turtlebot3_ws/scripts/trajectory/trajectory.png')
            plt.show
            
            #printing
            print('time is:',t)
            print('Linear valocity_x is:',v_x)
            print('Linear valocity_y is:',v_y)
            print('Linear valocity is:',v)
            print('Angular valocity is:',w)
            print('map data previous',self.map_data_prev)
            print('map data current', self.map_data_curr)
            print('---------------------------------------------------------')
            self.map_data_prev=self.map_data_curr
            self.iter += 1

            self.rate.sleep()
            # correct angle for destination
            # goal_position=[self.cur_pos_x,self.cur_pos_y,theta] #get current goal position
            # self.set_postion(goal_position) #set goal goal_angle
            dist2goal=1 #reset dist2goal  
            self.lock.release()

    def plot(self, map_data_prev,map_data_curr):
        plt.subplot(2,2,1) #first plot
        plt.title('Trajectory (planned vs actual)')
        plt.plot((map_data_prev[1],map_data_curr[1]), (map_data_prev[2],map_data_curr[2]),c='b') #trajectory actual
        plt.plot((map_data_prev[3],map_data_curr[3]), (map_data_prev[4],map_data_curr[4]),c='g') #trajectory planned
        plt.grid(True)
        plt.subplot(2,2,2) #second plot
        plt.title('Angle error(before vs after correction)')
        # plt.plot((map_data_prev[0],map_data_curr[0]),(map_data_prev[5],map_data_curr[5]),c='r')  #distance to goal
        plt.plot((map_data_prev[0],map_data_curr[0]),(map_data_prev[6],map_data_curr[6]),c='r') # before angle error
        plt.plot((map_data_prev[0],map_data_curr[0]),(map_data_prev[7],map_data_curr[7]),c='g') # corrected angle error
        plt.grid(True)
        plt.subplot(2,2,3) #third plot
        plt.title('Linear velocity')
        plt.plot((map_data_prev[0],map_data_curr[0]),(map_data_prev[8],map_data_curr[8]),c='g') #velocity linear
        plt.grid(True)
        plt.subplot(2,2,4) #fourth plot
        plt.title('Angular velocity')
        plt.plot((map_data_prev[0],map_data_curr[0]),(map_data_prev[9],map_data_curr[9]),c='y') #angular velocity
        plt.pause(.01) #time for plot to generate
        plt.grid(True)
        plt.show
        plt.savefig('/turtlebot3_ws/scripts/trajectory/trajectory.png')
    
    def save(self,data):
        np.savetxt('/turtlebot3_ws/scripts/trajectory/trajectory_log.csv',np.array(data),delimiter=',', fmt='%f', header="itr_n,pos_cur_x,pos_cur_y,pos_des_x,pos_des_y,dist2goal,theta_err_before,theta_err_after,vel_x,ang_vel_z", comments='')
        print('Trajectory_log saved at /turtlebot3_ws/scripts/trajectory')
# controller=control()
# #objects
# control_pid=controller.pid_control #pid object
# control_deri=controller.derivative_control #derivative object