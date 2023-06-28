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
import matplotlib.patches as mplt #for custon legend
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
        self.rate=rospy.Rate(50) #sampelling rate
        self.dt = 0.0
        self.time_start = 0.0 #start time
        self.end = False
        self.time_prev = 0.0
        self.time_start = rospy.get_time()
        #odom values
        self.cur_pos_x=0.0
        self.cur_pos_y=0.0
        self.cur_orient_x=0.0
        self.cur_orient_y=0.0
        self.cur_orient_z=0.0
        self.cur_orient_w=0.0
        #initialization parameters
        self.flag = True
        self.dist2goal=1 # distance to goal 
        self.dist2goal_limit=0.01  #set threshold value for distance to goal
        self.dist2goal_prev = 0  #for control gain
        self.error_angle_prev = 0  #for control gain

    def contoller(self, pos_des, current_position, current_orientaion):
        self.cur_pos_x = current_position.x
        self.cur_pos_y = current_position.y
        self.cur_rot_z = current_orientaion

        #while self.dist2goal > self.dist2goal_limit:
        t = rospy.get_time() - self.time_start
        self.dt = t - self.time_prev
        self.time_prev = t
        #quaternion to euler conversion
        #cur_rpy = tftr.euler_from_quaternion((self.cur_orient_x, self.cur_orient_y, self.cur_orient_z, self.cur_orient_w))  
        
        #Calcultion for distance to goal
        delta_x=pos_des[0]-self.cur_pos_x #delta x to goal 
        delta_y=pos_des[1]-self.cur_pos_y #delta y to goal
        self.dist2goal=sqrt(delta_x**2+delta_y**2) #distance to goal
        angle2goal=arctan2(delta_y, delta_x) #angle to goal
        error_angle = self.cur_rot_z -angle2goal #arror angle wrt angle to goal

        #before_error_angle=error_angle  #for printing

        #angle error correction for -pi, +pi, + and -
        if error_angle > pi:  
            error_angle =(2*pi)-error_angle
        elif error_angle < -pi:  
            error_angle =-(2*pi)-error_angle 
        else:
            error_angle =-error_angle*2

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
        #manual control
        if error_angle < pi/25 and error_angle > -pi/25: #+-0.13
            velocity.linear.x = min(control_signal_distance, 0.45)
        elif error_angle < pi/15 and error_angle > -pi/15: #+-0.21
            velocity.linear.x = min(control_signal_distance, 0.30)
        elif error_angle < pi/10 and error_angle > -pi/10: #+-0.31
            velocity.linear.x = min(control_signal_distance, 0.20)
        else: # > +- 0.31
            velocity.linear.x = min(control_signal_distance, 0.10)

        #cosine function based
        # if control_signal_distance > 0.0:
        #     velocity.linear.x = min(control_signal_distance*abs(cos(error_angle)), 0.3)
        # else:
        #     velocity.linear.x = max(control_signal_distance*abs(cos(error_angle)), -0.3)

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
        else:
            pass

        #update dist2goal_prev and error_angle_prev for next iteration
        self.dist2goal_prev=self.dist2goal 
        self.error_angle_prev=error_angle 

        return velocity,self.dist2goal
    
if __name__ == "__main__":
    rospy.init_node('point2point_node')
    task1 = point2point()
    task1.main()