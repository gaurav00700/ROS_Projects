import rospy
import matplotlib.pyplot as plt
import math
import numpy as np
from numpy import matrix, cos, arctan2, sqrt, pi, sin, cos
import tf.transformations as tftr


class mapping_class():
    
    def __init__(self):
        self.map=[[],[],[]]
        self.ranges = []
        self.goal=[]
        self.point_dict = {}
        self.goal_dict = {}
        self.current_position = None
        self.current_orientation =  None
        self.angle_increment = None
        self.intensity = None
        
    def mapping(self,ranges,current_position,current_orientation,angle_increment, intensity):

        self.ranges = ranges
        self.current_position = current_position
        self.current_orientation = current_orientation
        self.angle_increment = angle_increment
        self.intensity = intensity

        # initialize counter
        angle_counter = 0 
        pattern_lenght_counter = 0
        pattern_start_index = 0

        
        # Loop through LIDAR Values, convert to X/Y Coordinates and save in List
        for count, value in enumerate(self.ranges):
            angle_counter += self.angle_increment  

        # check if intensity is high, if yes increase leanght count, if no reset it.
            if len(self.intensity) == len(self.ranges):
                if self.intensity[count-1] < 80:
                    pattern_lenght_counter += 1
    
                    if pattern_start_index == 0:
                        pattern_start_index = count
                
                else:
                    pattern_lenght_counter = 0
                    pattern_start_index = 0

            if not math.isnan(value) and not math.isinf(value) and self.current_position != None:

                x = value * cos(angle_counter)
                y = value * sin(angle_counter)
                
                xr = (x * cos(self.current_orientation)) - (y * sin(self.current_orientation)) 
                yr = (x * sin(self.current_orientation)) + (y * cos(self.current_orientation)) 
                
                xr += self.current_position.x
                yr += self.current_position.y

                xr = round(xr, 1) 
                yr = round(yr, 1) 

                if len(self.intensity) == len(self.ranges):
                    intens = self.intensity[count-1]
                else:
                    intens = 230

                coordinates = list(zip(self.map[0],self.map[1]))
                #   print(coordinates)

                if (xr,yr) not in coordinates:

                    if self.point_dict.get(f"{(xr,yr)}") == None:
                        self.point_dict[f"{(xr,yr)}"] = (1,intens)
                    else:
                        self.point_dict[f"{(xr,yr)}"] = (self.point_dict.get(f"{(xr,yr)}")[0] + 1, self.point_dict.get(f"{(xr,yr)}")[1] + intens /2)

                    #if abs(self.velocity.linear.x) < 0.31 and abs(self.velocity.angular.z) < 0.2:
                    if self.point_dict.get(f"{(xr,yr)}")[0] > 200:
                        self.map[0].append(xr)
                        self.map[1].append(yr)
                        self.map[2].append(intens)

                    # writes map coordinates in txt file
                    if False:
                        print("writing")
                        file1 = open("myfile.txt", "w+")  # write mode
                        for coordinates in list(zip(self.map[0],self.map[1])):
                            file1.write(f'{coordinates[0]},{coordinates[1]}\n')
                        file1.close()
                        print("done")

                #get coordinates of tape if pattern in long enough

                if pattern_lenght_counter >= 12:

                    q = self.ranges[int(pattern_start_index + (pattern_lenght_counter / 2))]

                    x = q * cos(angle_counter)
                    y = q * sin(angle_counter)
                    
                    xr = (x * cos(self.current_orientation)) - (y * sin(self.current_orientation)) 
                    yr = (x * sin(self.current_orientation)) + (y * cos(self.current_orientation)) 
                    
                    xr += self.current_position.x
                    yr += self.current_position.y

                    xr = round(xr, 1) 
                    yr = round(yr, 1) 

                    if self.goal_dict.get(f"{(xr,yr)}") == None:
                        self.goal_dict[f"{(xr,yr)}"] = (1)
                    else:
                        self.goal_dict[f"{(xr,yr)}"] = (self.goal_dict.get(f"{(xr,yr)}") + 1)

                    #if abs(self.velocity.linear.x) < 0.31 and abs(self.velocity.angular.z) < 0.2:
                    if self.goal_dict.get(f"{(xr,yr)}") > 3:
    
                        not_aleady_detected_flag = True
                        for point in self.goal:
                            if abs(xr-point[0]) < 1.0 and abs(yr-point[1]) < 1.0:
                                not_aleady_detected_flag = False
                        if not_aleady_detected_flag:
                            if not math.isnan(xr) and not math.isinf(xr) and not math.isnan(yr) and not math.isinf(yr):
                                self.goal.append((xr,yr))
                    
                    pattern_lenght_counter = 0
                    pattern_start_index = 0
                
            else:
                pass

        return self.map, self.goal