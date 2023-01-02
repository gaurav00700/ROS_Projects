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
from numpy import matrix, arctan2, sqrt, pi, sin, cos
import numpy as np
import time
import matplotlib.pyplot as plt
from gazebo_msgs.msg import ModelState #for setting initial state of model
from gazebo_msgs.srv import SetModelState #for setting initial state of model

#control gain for distance
kp_distance = 1
ki_distance = 0.01
kd_distance = 0.4

#control gain for angle
kp_angle = 1
ki_angle = 0.01
kd_angle = 0.3

class point2point:
    def __init__(self):
        self.lock = threading.Lock() #for lock thread
        self.RATE = rospy.get_param('/rate', 50)
        self.dt = 0.0
        self.time_start = 0.0 #start time
        self.end = False

        #initialization parameters
        self.flag = True
        self.pose_des=[]
        self.dist2goal=1 # distance to goal 
        self.dist2goal_limit=0.01  #set threshold value for distance to goal
        self.dist2goal_prev = 0  #for control gain
        self.error_angle_prev = 0  #for control gain
        self.goal_num = 1 #number of goals
        #for data logging
        self.log_counter = 0
        self.trajectory=[]
        #for generating map
        self.map_data_curr=[]
        self.map_data_prev=[0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0, 0.0]
        self.iter=0

        #Defining publisher and subscriber
        self.pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1, latch=False)
        self.sub_odom = rospy.Subscriber("/odom", Odometry, self.odometry_callback)
            
    def odometry_callback(self, msg):
        self.lock.acquire()

        # read current robot state
        cur_position = msg.pose.pose.position
        cur_orientation = msg.pose.pose.orientation
        cur_rpy = tftr.euler_from_quaternion((cur_orientation.x, cur_orientation.y, cur_orientation.z, cur_orientation.w))  # roll pitch yaw
        cur_rot_z = cur_rpy[2]

        #Calcultion for distance to goal
        delta_x=self.pose_des[0]-cur_position.x #delta x to goal 
        delta_y=self.pose_des[1]-cur_position.y #delta y to goal
        self.dist2goal=sqrt(delta_x**2+delta_y**2) #distance to goal
        angle2goal=arctan2(delta_y, delta_x) #angle to goal
        error_angle = cur_rot_z -angle2goal #arror angle wrt angle to goal

        before_error_angle=error_angle  #for printing

        #angle error correction for -pi, +pi, + and -
        if error_angle > pi:  
            error_angle =-error_angle+(2*pi)
        elif error_angle < -pi:  
            error_angle =-(2*pi)+error_angle 
        else:
            error_angle =-error_angle

        #calcuale delta errors of distance and angle
        diff_dist2goal=self.dist2goal-self.dist2goal_prev
        diff_error_angle=error_angle-self.error_angle_prev

        #calculate sum of distance and angle
        sum_dist2goal=self.dist2goal+self.dist2goal_prev
        sum_error_angle=error_angle+self.error_angle_prev

        #PID controller
        # for distance
        linear_kp=kp_distance*self.dist2goal #control distance for P
        linear_ki=ki_distance*sum_dist2goal #control distance for I
        linear_kd=kd_distance*diff_dist2goal/self.dt #contol distance for D
        control_signal_distance=linear_kp+linear_ki+linear_kd #total control signal
        #for angle
        angular_kp=kp_angle*error_angle  #control angle for P
        angular_ki=ki_angle*sum_error_angle #control angle for I
        angular_kd=kd_angle*diff_error_angle/self.dt  #control angle for D
        control_signal_angle=angular_kp+angular_ki+angular_kd

        #publish linear and angular velocity
        velocity=Twist() #calling velocity message to publish

        #linear velocity limits
        velocity.linear.x = min(control_signal_distance, 0.2) #velocity min of control signal distance or 0.1        
        if error_angle > pi/4 or error_angle < -pi/4: #linear velocity limit for high angle error
            velocity.linear.x = 0
        else:
            pass

        #angular velocity limits
        velocity.angular.z=control_signal_angle #net control gain for angular velocity                
        if velocity.angular.z > 0:
            velocity.angular.z = min(velocity.angular.z, 2.0)
        else:
            velocity.angular.z = max(velocity.angular.z, -2.0)
        
        # set angle to goal_angle and velocities to zero at goal
        if self.dist2goal < self.dist2goal_limit:  
            velocity.linear.x=0 
            velocity.angular.z=0
            goal_position=[cur_position.x,cur_position.y,self.pose_des[2]] #get current goal position
            self.set_postion(goal_position) #set goal goal_angle
        else:
            pass
        
        #finally publishing velocities 
        self.pub_cmd_vel.publish(velocity)

        #update dist2goal_prev and error_angle_prev for next iteration
        self.dist2goal_prev=self.dist2goal 
        self.error_angle_prev=error_angle 

        # logging once every 100 times (Gazebo runs at 1000Hz; we save it at 10Hz)
        self.log_counter += 1
        if self.log_counter == 10 or self.dist2goal <= self.dist2goal_limit:
            self.log_counter = 0
            self.iter += 1
            self.trajectory.append([self.iter,cur_position.x, cur_position.y, self.dist2goal, before_error_angle, error_angle, velocity.linear.x, velocity.angular.z])

            #plot the graph
            self.map_data_curr=[self.iter, cur_position.x, cur_position.y, self.dist2goal, before_error_angle, error_angle, velocity.linear.x, velocity.angular.z]
            self.plot(self.map_data_prev,self.map_data_curr)
            self.map_data_prev=self.map_data_curr

        time.sleep(0.01)
        self.lock.release()

        #printing values
        print('Destination co-ordinates:', [round(elem,1) for elem in self.pose_des])
        print("Distance to goal", round(self.dist2goal,2))
        # print("Delta X and Y:",delta_x, delta_y)
        # print('Previous error angle:', error_angle)
        print("Current angle error:", round(error_angle,2))
        print('Linear velocity is', round(velocity.linear.x, 2))
        print('Angular velocity is', round(velocity.angular.z,2))
        
        print('-------------------------------------------------------')
    
    def plot(self, map_data_prev,map_data_curr):
        plt.subplot(2,2,1)
        plt.title('Trajectory')
        plt.plot((map_data_prev[1],map_data_curr[1]), (map_data_prev[2],map_data_curr[2]),c='b') #navigation
        if self.dist2goal <= self.dist2goal_limit:
            plt.scatter(map_data_curr[1],map_data_curr[2], c='r', s=50) #mark current position
        plt.subplot(2,2,2)
        plt.title('Distance error')
        plt.plot((map_data_prev[0],map_data_curr[0]),(map_data_prev[3],map_data_curr[3]),c='r')  #distance to goal
        plt.subplot(2,2,3)
        plt.title('Angle error(before vs after)')
        plt.plot((map_data_prev[0],map_data_curr[0]),(map_data_prev[4],map_data_curr[4]),c='r') # before angle error
        plt.plot((map_data_prev[0],map_data_curr[0]),(map_data_prev[5],map_data_curr[5]),c='g') # corrected angle error
        plt.subplot(2,2,4)
        plt.title('Linear vs angular velocity')
        plt.plot((map_data_prev[0],map_data_curr[0]),(map_data_prev[6],map_data_curr[6]),c='g') #velocity linear
        plt.plot((map_data_prev[0],map_data_curr[0]),(map_data_prev[7],map_data_curr[7]),c='y') #angular velocity
        plt.pause(.01) #time for plot to generate
        plt.show
        if self.dist2goal <= self.dist2goal_limit:
            plt.savefig('/turtlebot3_ws/scripts/trajectory.png')

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

    def spin(self):  
        rospy.loginfo('Task started...!')

        #Set starting position (x, y , theta)
        initial_position=[0.0,0.0,0.0]   #x ,y, theta_yaw_euler
        self.set_postion(initial_position) 
        
        time_prev = 0.0
        self.time_start = rospy.get_time()

        #set goals
        # goal_list=[[2.0,2.0,pi]] #diagonal straight line 
        goal_list=[[1.0,0.0,0.0],[-1.0,-1.0,0.0],[-1.0,1.0,0.0],[1.0,0.0,0.0]] #triangle
        # goal_list=[[1.0,0.0,0.0],[-1.0,-1.0,0.0],[-1.0,1.0,0.0],[1.0,0.0,0.0],[2.0,0.0,0.0],[-2.0,-2.0,0.0],[-2.0,2.0,0.0],[2.0,0.0,0.0]] # double triangle
        # goal_list=[[0.0,1.0,0.0],[1.0,0.0,0.0],[1.0,1.0,0.0],[2.0,0.0,0.0],[2.0,1.0,0.0],[1.0,0.0,0.0],[1.0,1.0,0.0],[0.0,0.0,0.0]]
        for goal in goal_list:
            while self.dist2goal > self.dist2goal_limit:
                t = rospy.get_time() - self.time_start
                self.dt = t - time_prev
                time_prev = t
                self.pose_des = goal
                time.sleep(0.01)

            print(f'Reached to goal: {self.pose_des}')
            self.dist2goal=1 #reset distance for next point            
        #saving log file into csv
        np.savetxt('/turtlebot3_ws/scripts/trajectory_log.csv',np.array(self.trajectory),delimiter=',', fmt='%f', header="itr_n,pos_x,pos_y,dist2goal,theta_err_before,theta_err_after,vel_x,ang_vel_z", comments='')

        print('Trajectory_log saved at /turtlebot3_ws/scripts')
        print('!!!!.....TASK COMPLETED   ^_^  .....!!!!')

if __name__ == "__main__":
    rospy.init_node('navigation_node')
    task1 = point2point()
    task1.spin()
