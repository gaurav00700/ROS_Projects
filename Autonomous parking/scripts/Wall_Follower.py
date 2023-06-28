#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

class wall_follower_class:

    def __init__(self):
        self.speed = 0.08 #Set the speed of the robot
        self.DIR = None
        self.velocity = Twist()
        
        self.follow_dir = 0
        '''
        self.follow_dir = 0 -> wall finding loop
        self.follow_dir = 1 -> Follow right wall
        self.follow_dir = 2 -> Follow left wall
        '''

    def callback(self,msg):

        try:
            front_min_list = []
            left_min_list = []
            right_min_list = []

            ranges = msg
            radar_len=len(ranges)

            #Sorting the ranges into direction lists
            front_list = ranges[int(345*radar_len/360):radar_len] + ranges[0:int(15*radar_len/360)]
            left_list = ranges[int(70*radar_len/360):int(90*radar_len/360)]
            right_list = ranges[int(270*radar_len/360):int(290*radar_len/360)]

            #Finding the minimum range from the direction lists
            for i in range(len(front_list)):
                if front_list[i] != 0:
                    front_min_list.append(front_list[i])
                else:
                    pass
            front_min = min(front_min_list)
                    
            for i in range(len(left_list)):
                if left_list[i] != 0:
                    left_min_list.append(left_list[i])
                else:
                    pass
            left_min = min(left_min_list)

            for i in range(len(right_list)):
                if right_list[i] != 0:
                    right_min_list.append(right_list[i])
                else:
                    pass
            right_min = min(right_min_list)

            #Assigning minimum values to a dictionary
            self.DIR = {
            'left': left_min,
            'front': front_min,
            'right': right_min,
            }
            
            #Calling the controller function
            self.movement()
        except:
            pass
        return self.velocity, self.follow_dir

    def movement(self):

        self.velocity = Twist()

        b = 0.5   # maximum threshold distance
        a = 0.4  # minimum threshold distance

        if self.DIR['front'] > b and self.DIR['left'] > b and self.DIR['right'] > b:
            if self.follow_dir == 0:
                 #Move forward till you find a wall
                self.move_ahead()

        elif self.follow_dir == 0:
            #Setting the direction of wall to follow
            if self.DIR['left'] < b:
                #Follow left wall
                self.turn_right()
                self.follow_dir = 2
            elif self.DIR['right'] < b:
                #Follow right wall
                self.turn_left()
                self.follow_dir = 1
            else:
                if self.follow_dir == 0:
                    #Go right on the very first time you find the wall
                    self.turn_left()
                elif self.follow_dir == 1:
                    #When you follow right wall and find right opening, go right
                    self.turn_left()
                elif self.follow_dir == 2:
                    #When you follow left wall and find left opening, go left
                    self.move_diag_left()

        if self.follow_dir == 2:
            #Algorithm for left wall follower
            if self.DIR['left'] >= b and self.DIR['front'] >= b and self.DIR['right'] >= b:
                self.move_diag_left()
            elif self.DIR['left'] >= b and self.DIR['front'] >= b and self.DIR['right'] <= b:
                self.move_diag_left()
            elif self.DIR['left'] >= b and self.DIR['front'] <= a and self.DIR['right'] >= b:
                self.turn_right()
            elif self.DIR['left'] >= b and self.DIR['front'] <= b and self.DIR['right'] <= b:
                self.turn_right()
            elif self.DIR['left'] <= b and self.DIR['front'] >= b and self.DIR['right'] >= b:
                self.move_ahead()
            elif self.DIR['left'] <= b and self.DIR['front'] >= b and self.DIR['right'] <= b:
                self.move_ahead()
            elif self.DIR['left'] <= b and self.DIR['front'] >= b and self.DIR['right'] <= a:
                self.move_diag_left()
            elif self.DIR['left'] <= a and self.DIR['front'] >= b and self.DIR['right'] <= b:
                self.move_diag_right()
            elif self.DIR['left'] <= b and self.DIR['front'] <= b and self.DIR['right'] >= b:
                self.turn_right()
            elif self.DIR['left'] <= b and self.DIR['front'] <= b and self.DIR['right'] <= b:
                self.turn_right()
            else:
                self.move_ahead_slow()

        elif self.follow_dir == 1:
            #Algorithm for right wall follower
            if self.DIR['left'] >= b and self.DIR['front'] >= b and self.DIR['right'] >= b:
                self.move_diag_right()
            elif self.DIR['left'] >= b and self.DIR['front'] >= b and self.DIR['right'] <= b:
                self.move_ahead()
            elif self.DIR['left'] >= b and self.DIR['front'] <= a and self.DIR['right'] >= b:
                self.turn_left()
            elif self.DIR['left'] >= b and self.DIR['front'] <= b and self.DIR['right'] <= b:
                self.turn_left()
            elif self.DIR['left'] <= b and self.DIR['front'] >= b and self.DIR['right'] >= b:
                self.move_diag_right()
            elif self.DIR['left'] <= b and self.DIR['front'] >= b and self.DIR['right'] <= b:
                self.move_ahead()
            elif self.DIR['left'] <= a and self.DIR['front'] >= b and self.DIR['right'] <= b:
                self.move_diag_right()
            elif self.DIR['left'] <= b and self.DIR['front'] >= b and self.DIR['right'] <= a:
                self.move_diag_left()
            elif self.DIR['left'] <= b and self.DIR['front'] <= b and self.DIR['right'] >= b:
                self.turn_left()
            elif self.DIR['left'] <= b and self.DIR['front'] <= b and self.DIR['right'] <= b:
                self.turn_left()
            else:
                self.move_ahead_slow()

    def move_diag_left(self):
        print("move d left")
        self.velocity.linear.x = self.speed
        self.velocity.angular.z = 3*self.speed

    def move_diag_right(self):
        print("move d right")
        self.velocity.linear.x = self.speed
        self.velocity.angular.z = -3*self.speed

    def move_ahead(self):
        print("move ahead")
        self.velocity.linear.x = 3*self.speed
        self.velocity.angular.z = 0

    def turn_right(self):
        print("move right")
        self.velocity.linear.x = 0
        self.velocity.angular.z = -3*self.speed

    def turn_left(self):
        print("move left")
        self.velocity.linear.x = 0
        self.velocity.angular.z = 3*self.speed
    
    def move_ahead_slow(self):
        print("move ahead slow")
        self.velocity.linear.x = self.speed
        self.velocity.angular.z = 0

    def move_reverse(self):
        print("move reverse")
        self.velocity.linear.x = -3*self.speed
        self.velocity.angular.z = 0