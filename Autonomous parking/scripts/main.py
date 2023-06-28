import rospy
import threading
from collections import OrderedDict
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist, Pose2D
from sensor_msgs.msg import LaserScan
from math import atan2
import tf.transformations as tftr
from numpy import matrix, cos, arctan2, sqrt, pi, sin, cos
import numpy as np
import time
import matplotlib.pyplot as plt
import matplotlib as mpl
import os
from matplotlib import animation
import math

from Wall_Follower import wall_follower_class
from mapping import mapping_class
from path_planing import path_generator
from path_follower import point2point

class Parking():
    def __init__(self):
        self.lock = threading.Lock()

        rospy.init_node('follow_wall')
        self.RATE = rospy.get_param('/rate', 100)
        self.rate = rospy.Rate(self.RATE)
        self.pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1, latch=False)
        rospy.Subscriber("/odom", Odometry, self.odometry_callback, queue_size=1)
        rospy.Subscriber('/scan', LaserScan, self.laser_callback, queue_size=1)

        #Code variables
        self.current_position = None
        self.angle_increment = 0
        self.intensity = 0
        self.velocity = Twist()
        self.t = 0.0
        self.current_state = 0
        self.goal_counter = 0
        '''
        current_state = 0 --> mapping and wall_follower
        current_state = 1 --> path planning
        current_state = 2 --> Go to goal
        '''

        #Mapping variables
        self.mapping_py = mapping_class()
        self.map=[[],[],[]]
        self.ranges = []
        self.goal= []
        self.point_dict = {}


        #Wall follower variables
        self.wall_follower_py = wall_follower_class()
        self.origin_pos_x = 0.0
        self.origin_pos_y = 0.0
        self.wall_follower_settings = 0 #0 -> initialized wall follower, 1 -> running first 20 seconds, 2 -> arrived back to the home position
        self.WF_start_time = 0
        self.WF_time = 0
        self.WF_following_state = 0

        #path_planing variables 
        self.path_to_parking = []

    def odometry_callback(self, msg):
        self.lock.acquire()
        self.current_position = msg.pose.pose.position
        cur_q= msg.pose.pose.orientation
        cur_rpy = tftr.euler_from_quaternion((cur_q.x, cur_q.y, cur_q.z, cur_q.w))  # roll pitch yaw
        self.current_orientation = cur_rpy[2]
        self.speed_linear=msg.twist.twist.linear.x
        self.speed_angular=msg.twist.twist.angular.z
        self.lock.release()

    def laser_callback(self,msg):
        self.lock.acquire()
        self.ranges = msg.ranges
        self.intensity = msg.intensities
        self.angle_increment = msg.angle_increment
        self.lock.release()

    def wall_follower(self):
        self.velocity, self.WF_following_state = self.wall_follower_py.callback(self.ranges)
        self.pub_cmd_vel.publish(self.velocity)
        distance2orign_x = self.origin_pos_x - self.current_position.x
        distance2orign_y = self.origin_pos_y - self.current_position.y
        distance2origin = sqrt(distance2orign_x**2 + distance2orign_y**2)
        if self.wall_follower_settings == 0:
            self.WF_start_time = rospy.get_time()
            self.wall_follower_settings = 1
        elif self.wall_follower_settings == 1:
            try:
                self.WF_time = rospy.get_time() - self.WF_start_time
            except:
                pass
            if self.WF_time > 10 and self.origin_pos_x != 0:
                if (distance2origin < 0.2 and len(self.goal) != 0 )or (self.WF_time > 120 and len(self.goal)):
                    print("Robot is back to its origin")
                    self.path_planning_start_point = [self.current_position.x, self.current_position.y]
                    print("Stop point is: ", self.path_planning_start_point)
                    self.wall_follower_settings = 2
                    self.velocity.linear.x = 0
                    self.velocity.angular.z = 0
                    self.pub_cmd_vel.publish(self.velocity)
                    self.current_state = 1
        else:
            pass

#######################################################################################################################################################################
    def mapping(self):
        self.map, self.goal = self.mapping_py.mapping(self.ranges, self.current_position, self.current_orientation, self.angle_increment, self.intensity)

    def plot_map(self):

        plt.clf()

        if len(self.goal) > 0:
            for point in self.goal:
                plt.plot(point[0], point[1], "xr") 

        try:
            # plt.quiver(self.current_position.x, self.current_position.y, 1, 0)
            marker, scale = self.gen_arrow_head_marker(self.current_orientation)
            markersize = 25
            plt.scatter(self.current_position.x, self.current_position.y, marker=marker, s=(markersize*scale)**2)
        except:
            pass

        plt.scatter(task.map[0],task.map[1], c =self.map[2],cmap = "Spectral", s=0.5)

        if len(self.path_to_parking) != 0:
            plt.plot([x for (x, y) in self.path_to_parking], [y for (x, y) in self.path_to_parking], '-r')

        plt.colorbar()
        plt.draw()
        plt.pause(0.01)
        plt.clf()

########################################################################################################################################################################

        self.finding_tabe()
        
    def finding_tabe(self):
        pass
        #define the coordinates of goal

    def gen_arrow_head_marker(self,angle):

        arr = np.array([[1, 0],[.1, .3], [.1, -.3], [.3, 0]])  # arrow shape
        rot_mat = np.array([
            [cos(angle), sin(angle)],
            [-sin(angle), cos(angle)]
            ])
        arr = np.matmul(arr, rot_mat)  # rotates the arrow

        # scale
        x0 = np.amin(arr[:, 0])
        x1 = np.amax(arr[:, 0])
        y0 = np.amin(arr[:, 1])
        y1 = np.amax(arr[:, 1])
        scale = np.amax(np.abs([x0, x1, y0, y1]))
        codes = [mpl.path.Path.MOVETO, mpl.path.Path.LINETO,mpl.path.Path.LINETO, mpl.path.Path.CLOSEPOLY]
        arrow_head_marker = mpl.path.Path(arr, codes)
        return arrow_head_marker, scale
        
    def path_generation(self):
        coordinates = list(zip(self.map[0],self.map[1]))
        reduced_list = [] 
        points_to_remove = []
        start_time = rospy.get_time()
        t = 0
    
        while t < 2.0:
            t = rospy.get_time() - start_time
        
        # print(len(coordinates))
        for index, point in enumerate(coordinates.copy()):
        
            flag = True 

            if abs(abs(point[0]) - abs(self.current_position.x)) < 0.2 and abs(abs(point[1]) - abs(self.current_position.y)) < 0.2:
                    flag = False

            for goal in self.goal:
                if abs(abs(point[0]) - abs(goal[0])) < 0.3 and abs(abs(point[1]) - abs(goal[1])) < 0.3:

                    flag = False
            if flag:
                reduced_list.append(point)

        start_point = task.current_position.x ,task.current_position.y
        self.path_to_parking = path_generator(input_points=reduced_list,start=start_point,goal_points=self.goal, maxIter=None)
        self.current_state=2
        self.path_follower_py = point2point()   
        
    def go_to_parking(self):
        threshold = 0.1
        self.velocity, dist2goal = task.path_follower_py.contoller(self.path_to_parking[self.goal_counter],self.current_position,self.current_orientation)
        if dist2goal < threshold:
            self.goal_counter += 1
        if len(self.path_to_parking) == self.goal_counter:
            self.velocity.linear.x = 0
            self.velocity.angular.z = 0
            self.pub_cmd_vel.publish(self.velocity)
            self.current_state +=1
            print("The robot is parked!")

        self.pub_cmd_vel.publish(self.velocity)

    def go_to_home(self):
        pass

if __name__ == "__main__":
    
    task= Parking()
    task.rate.sleep()
    start_time = rospy.get_time()
    task.t = 0
    
    while task.t < 2.0:
        task.t = rospy.get_time() - start_time

    while not rospy.is_shutdown():

        if (task.origin_pos_x == 0 or task.origin_pos_y == 0) and task.WF_following_state != 0:
            task.origin_pos_x = task.current_position.x
            task.origin_pos_y = task.current_position.y
        if task.current_state == 0:
            task.wall_follower()
            task.mapping()
            task.plot_map()
            print("Goals found: ", task.goal)
        elif task.current_state == 1:
            task.path_generation()
            # print(task.path_to_parking)
        elif task.current_state==2:
            task.go_to_parking()
            task.plot_map()
        task.rate.sleep()

