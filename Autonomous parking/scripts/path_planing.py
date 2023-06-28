import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))

import copy
import math
import random
from scipy.spatial.transform import Rotation as Rot
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

show_animation = True

class InformedRRTStar:
    def __init__(self, start, goal, obstacleList, randArea, expandDis, goalSampleRate, maxIter):
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        self.x_rand = randArea[0]
        self.y_rand = randArea[1]
        self.expand_dis = expandDis
        self.goal_sample_rate = goalSampleRate
        self.max_iter = maxIter
        self.obstacle_list = obstacleList
        self.node_list = None

    def informed_rrt_star_search(self, animation=True):

        self.node_list = [self.start]
        # max length we expect to find in our 'informed' sample space,
        # starts as infinite
        cBest = float('inf')
        solutionSet = set()
        path = None

        # Computing the sampling space
        cMin = math.sqrt(pow(self.start.x - self.goal.x, 2) + pow(self.start.y - self.goal.y, 2)) #minimum distance between start and goal
        xCenter = np.array([[(self.start.x + self.goal.x) / 2.0], [(self.start.y + self.goal.y) / 2.0], [0]]) # coordiante of centre point of cMin ?? tbc
        a1 = np.array([[(self.goal.x - self.start.x) / cMin], [(self.goal.y - self.start.y) / cMin], [0]]) # coordiante (cos , sin)

        e_theta = math.atan2(a1[1], a1[0])
        # first column of identity matrix transposed
        id1_t = np.array([1.0, 0.0, 0.0]).reshape(1, 3)
        M = a1 @ id1_t
        U, S, Vh = np.linalg.svd(M, True, True)
        C = np.dot(np.dot(U, np.diag( [1.0, 1.0, np.linalg.det(U) * np.linalg.det(np.transpose(Vh))])), Vh)
        #list for saving part points
        path_list=[]
        path_len=[]
        path_count=0
        i=0
        while path_count <= 2 :
        # while path == None or optim_count ==50 :
            i +=1
        # for i in range(i, self.max_iter):
            # Sample space is defined by cBest
            # cMin is the minimum distance between the start point and the goal
            # xCenter is the midpoint between the start and the goal
            # cBest changes when a new path is found
            # print('Node number:', i, '|',self.max_iter )
            print('Node number:', i )
            print('Number of path found:', path_count )

            rnd = self.informed_sample(cBest, cMin, xCenter, C)
            n_ind = self.get_nearest_list_index(self.node_list, rnd)
            nearestNode = self.node_list[n_ind]
            # steer
            theta = math.atan2(rnd[1] - nearestNode.y, rnd[0] - nearestNode.x)
            newNode = self.get_new_node(theta, n_ind, nearestNode)
            d = self.line_cost(nearestNode, newNode)

            noCollision = self.check_collision(nearestNode, theta, d)

            if noCollision:
                nearInds = self.find_near_nodes(newNode)
                newNode = self.choose_parent(newNode, nearInds)
                self.node_list.append(newNode) #add current node in node list
                self.rewire(newNode, nearInds) #rewire existing edges
                
                #condition for drawing ellipse (new naodes will be created inside the ellipse) and calculate path distance
                if self.check_segment_collision(newNode.x, newNode.y, self.goal.x, self.goal.y) or self.is_near_goal(newNode) :
                    path_count +=1
                    solutionSet.add(newNode)
                    lastIndex = len(self.node_list) - 1
                    tempPath = self.get_final_course(lastIndex)
                    tempPathLen = self.get_path_len(tempPath)
                    path_list.append(tempPath) #add point to path list
                    path_len.append(tempPathLen) #add length in path list
                    if tempPathLen < cBest: #selection of best path
                        path = tempPath
                        cBest = tempPathLen
            #plot node and ellipse (if condition met)
            if animation:
                self.draw_graph(xCenter=xCenter, cBest=cBest, cMin=cMin, e_theta=e_theta, rnd=rnd)
            
            #increase sample rate (help in getting optimal path if ellipse not achieved by more nodes around goal)
            # if i > self.max_iter*0.60 and cBest == float('inf'):
            if i > 200 and cBest == float('inf'):
                #self.goal_sample_rate=min(self.goal_sample_rate*1.01, 40) #restrict sample rate to 80%
                print('Goal sample rate:', round(self.goal_sample_rate,1))
            elif cBest == float('inf'):
                print('Goal sample rate:', round(self.goal_sample_rate,1))
            else:
                print('Goal sample rate: Inside Ellipse')
            print('---------------------------------------------')
        #increase number of iteration and adjust sampling rate if goal is not found
        # self.max_iter += 50
        # self.goal_sample_rate= max(int(self.goal_sample_rate*0.50), 20)

        if path != None :
            path.reverse() #reverse the path from start to end
            if path[-1] != [self.goal.x, self.goal.y]:
                print('Goal point missing')
                path.insert(-1,[self.goal.x, self.goal.y]) #insert goal points if not there
        goal_list=[path_len, path_list] #compile all goal lists
        return path, cBest, goal_list

    def choose_parent(self, newNode, nearInds):
        if len(nearInds) == 0:
            return newNode

        dList = []
        for i in nearInds:
            dx = newNode.x - self.node_list[i].x
            dy = newNode.y - self.node_list[i].y
            d = math.hypot(dx, dy)
            theta = math.atan2(dy, dx)
            if self.check_collision(self.node_list[i], theta, d):
                dList.append(self.node_list[i].cost + d)
            else:
                dList.append(float('inf'))

        minCost = min(dList)
        minInd = nearInds[dList.index(minCost)]

        if minCost == float('inf'):
            print("min cost is inf")
            return newNode

        newNode.cost = minCost
        newNode.parent = minInd

        return newNode

    def find_near_nodes(self, newNode):
        n_node = len(self.node_list)
        r = 50.0 * math.sqrt((math.log(n_node) / n_node))
        d_list = [(node.x - newNode.x) ** 2 + (node.y - newNode.y) ** 2 for node in self.node_list]
        near_inds = [d_list.index(i) for i in d_list if i <= r ** 2]
        return near_inds

    def informed_sample(self, cMax, cMin, xCenter, C):
        if cMax < float('inf'): #check for ellipsoidal condition
            r = [cMax / 2.0, math.sqrt(abs(cMax ** 2 - cMin ** 2)) / 2.0, math.sqrt(abs(cMax ** 2 - cMin ** 2)) / 2.0]
            L = np.diag(r)
            xBall = self.sample_unit_ball() #generating inside the ellipse
            rnd = np.dot(np.dot(C, L), xBall) + xCenter
            rnd = [rnd[(0, 0)], rnd[(1, 0)]]
        else:
            rnd = self.sample_free_space()

        return rnd

    @staticmethod
    def sample_unit_ball():
        a = random.random()
        b = random.random()
        if b < a:
            a, b = b, a
        sample = (b * math.cos(2 * math.pi * a / b),  b * math.sin(2 * math.pi * a / b))
        
        return np.array([[sample[0]], [sample[1]], [0]])

    def sample_free_space(self):
        #randon place based on probability/sample rate
        value=random.randint(0, 100)
        if value > self.goal_sample_rate: #condition 1: 100-sample_rate
            x=random.uniform(self.x_rand[0], self.x_rand[1])
            y=random.uniform(self.y_rand[0], self.y_rand[1])
        elif value <= self.goal_sample_rate and value > self.goal_sample_rate*0.3: #condition 2: sample_rate - sample_rate/2
            x=random.uniform(self.goal.x-self.expand_dis*0.3, self.goal.x+self.expand_dis*0.3) #multiplied--> to cover more radial area
            y=random.uniform(self.goal.y-self.expand_dis*0.3, self.goal.y+self.expand_dis*0.3) # --''--
        else: #condition 3: < sample_rate
            x=self.goal.x
            y=self.goal.y
        # #ensure node point generated inside the map space    
        # x=self.x_rand[1]*0.8 if x >= self.x_rand[1] else x #multiplied--> to get point inside the world
        # x=self.x_rand[0]*0.8 if x <= self.x_rand[0] else x # --''--
        # y=self.y_rand[1]*0.8 if y >= self.x_rand[1] else y # --''--
        # y=self.y_rand[0]*0.8 if y <= self.y_rand[0] else y # --''--
        
        rnd = [x, y]
        return rnd

    @staticmethod
    def get_path_len(path):
        pathLen = 0
        for i in range(1, len(path)):
            node1_x = path[i][0]
            node1_y = path[i][1]
            node2_x = path[i - 1][0]
            node2_y = path[i - 1][1]
            pathLen += math.sqrt((node1_x - node2_x) ** 2 + (node1_y - node2_y) ** 2)

        return pathLen

    @staticmethod
    def line_cost(node1, node2):
        return math.sqrt((node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2)

    @staticmethod
    def get_nearest_list_index(nodes, rnd):
        dList = [(node.x - rnd[0]) ** 2
                 + (node.y - rnd[1]) ** 2 for node in nodes]
        minIndex = dList.index(min(dList))
        return minIndex

    def get_new_node(self, theta, n_ind, nearestNode):
        newNode = copy.deepcopy(nearestNode)

        newNode.x += self.expand_dis * math.cos(theta)
        newNode.y += self.expand_dis * math.sin(theta)

        newNode.cost += self.expand_dis
        newNode.parent = n_ind
        return newNode

    def is_near_goal(self, node):
        d = self.line_cost(node, self.goal)
        if d < self.expand_dis*0.2: #near to goal
            return True
        return False

    def rewire(self, newNode, nearInds):
        n_node = len(self.node_list)
        for i in nearInds:
            nearNode = self.node_list[i]
            d = math.sqrt((nearNode.x - newNode.x) ** 2 + (nearNode.y - newNode.y) ** 2)
            s_cost = newNode.cost + d
            if nearNode.cost > s_cost:
                theta = math.atan2(newNode.y - nearNode.y, newNode.x - nearNode.x)
                if self.check_collision(nearNode, theta, d):
                    nearNode.parent = n_node - 1
                    nearNode.cost = s_cost

    @staticmethod
    def distance_squared_point_to_segment(v, w, p):
        # Return minimum distance between line segment vw and point p
        if np.array_equal(v, w):
            return (p - v).dot(p - v)  # v == w case
        l2 = (w - v).dot(w - v)  # i.e. |w-v|^2 -  avoid a sqrt
        # Consider the line extending the segment,
        # parameterized as v + t (w - v).
        # We find projection of point p onto the line.
        # It falls where t = [(p-v) . (w-v)] / |w-v|^2
        # We clamp t from [0,1] to handle points outside the segment vw.
        t = max(0, min(1, (p - v).dot(w - v) / l2))
        projection = v + t * (w - v)  # Projection falls on the segment
        return (p - projection).dot(p - projection)

    def check_segment_collision(self, x1, y1, x2, y2):
        for (ox, oy, size) in self.obstacle_list:
            dd = self.distance_squared_point_to_segment( np.array([x1, y1]), np.array([x2, y2]), np.array([ox, oy]))
            if dd <= size ** 2:
                return False  # collision
        return True

    def check_collision(self, nearNode, theta, d):
        tmpNode = copy.deepcopy(nearNode)
        end_x = tmpNode.x + math.cos(theta) * d
        end_y = tmpNode.y + math.sin(theta) * d
        return self.check_segment_collision(tmpNode.x, tmpNode.y, end_x, end_y)

    def get_final_course(self, lastIndex):
        path = [[self.goal.x, self.goal.y]]
        while self.node_list[lastIndex].parent is not None:
            node = self.node_list[lastIndex]
            path.append([node.x, node.y])
            lastIndex = node.parent
        path.append([self.start.x, self.start.y])
        return path

    def draw_graph(self, xCenter=None, cBest=None, cMin=None, e_theta=None,  rnd=None):
        plt.clf()
        plt.title('Informed RRT*')
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect('key_release_event', lambda event: [exit(0) if event.key == 'escape' else None])
        #for obatacles
        for (ox, oy, size) in self.obstacle_list: 
            plt.plot(ox, oy, "ok", ms=30 * size)
        #for random node points and ellipse
        if rnd is not None:
            plt.plot(rnd[0], rnd[1], "^b")
            if cBest != float('inf'):
                self.plot_ellipse(xCenter, cBest, cMin, e_theta)
        # for connecting nodes and edges
        for node in self.node_list:
            if node.parent is not None:
                if node.x or node.y is not None:
                    plt.plot([node.x, self.node_list[node.parent].x], [node.y, self.node_list[node.parent].y], "-g", markersize=10)
        #for start and goal points
        plt.plot(self.start.x, self.start.y, "xr") 
        plt.plot(self.goal.x, self.goal.y, "xr")
        plt.axis([self.x_rand[0], self.x_rand[1], self.y_rand[0], self.y_rand[1]]) #plot area
        plt.grid(True)
        plt.pause(0.001)

    @staticmethod
    def plot_ellipse( xCenter, cBest, cMin, e_theta):  # pragma: no cover

        a = math.sqrt(abs(cBest ** 2 - cMin ** 2)) / 2.0
        b = cBest / 2.0
        angle = math.pi / 2.0 - e_theta
        cx = xCenter[0]
        cy = xCenter[1]
        t = np.arange(0, 2 * math.pi + 0.1, 0.1)
        x = [a * math.cos(it) for it in t]
        y = [b * math.sin(it) for it in t]
        #rotation matrix function
        def rot_mat_2d(angle):
            """
            Create 2D rotation matrix from an angle
            Parameters
            ----------
            angle :
            Returns
            -------
            A 2D rotation matrix
            Examples
            --------
            >>> angle_mod(-4.0)
            """
            return Rot.from_euler('z', angle).as_matrix()[0:2, 0:2]

        fx = rot_mat_2d(-angle) @ np.array([x, y])
        px = np.array(fx[0, :] + cx).flatten()
        py = np.array(fx[1, :] + cy).flatten()
        plt.plot(cx, cy, "xc")
        plt.plot(px, py, "--c")

class Node:

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.cost = 0.0
        self.parent = None

def path_generator(start=None,goal_points=None, maxIter=None, input_points=None,):
    print("Start informed rrt star planning")
    points = np.array(input_points)
    obstacleList=[]
    for point in points:
        point=list(point)
        point.insert(2,0.2)
        obstacleList.append(point)
    #for node random value & plot area
    x_min=min([point[0] for point in points])
    y_min=min([point[1] for point in points])
    x_max=max([point[0] for point in points])
    y_max=max([point[1] for point in points])
    path_pnt_list=[]
    path_len_list=[]
    path=None
    # START PARAMETERS
    expandDis=0.1 #min lenght of step
    goalSampleRate=20 # rate of node generation (on goal/not on goal) (max is 100)
    maxIter=100 #numbers of iteration/nodes
    randArea=[(x_min,x_max), (y_min,y_max)] #random point generation range(min ,max)
    for goal in goal_points:
        rrt = InformedRRTStar(start=start, goal=goal, randArea=randArea, obstacleList= obstacleList, expandDis=expandDis, goalSampleRate=goalSampleRate, maxIter=maxIter)
        path ,path_len, goal_list = rrt.informed_rrt_star_search(animation=show_animation)
        
        # Plot and save data of path
        if path != None:
            print("Goal Path found Successfuly...!!")
            # pd.DataFrame(goal_list).to_csv("goal_list.csv", index=False) #save all possible paths in current loop in csv file
            #draw path in plot
            rrt.draw_graph()
            plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
            plt.grid(True)
            plt.pause(2)
            plt.savefig('informed_rrt*_goal_path.png', dpi=500) #save figure
            plt.draw()
        else:
            print('Goal Path not found ..!!')
        # append path and path length in list
        path_pnt_list.append(path)
        path_len_list.append(path_len)
    
    #return shortest path
    index_min=path_len_list.index(min(path_len_list)) #index of shortest path
    path_points=path_pnt_list[index_min] #shortest path
    
    #save best path points
    print('----------Best Goal length and path points:-----------')
    # print(path_points)
    print("Path_points saved in current directory")
    np.savetxt('path_points.txt', path_points, delimiter=',', fmt='%f', comments='') 
    
    return path_points