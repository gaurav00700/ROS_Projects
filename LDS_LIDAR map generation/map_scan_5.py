import rospy
import threading
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
from numpy import sin,cos,pi,radians,degrees, matmul,amin,amax,abs
import matplotlib.pyplot as plt
import matplotlib as mpl
import tf.transformations as tftr

class rosbot():
    def __init__(self):
        self.lock = threading.Lock()
        self.rate=rospy.Rate(5) #rate
        self.cur_pos_x, self.cur_pos_y = 0.0, 0.0
        self.cur_rot=0.0
        self.cur_wz=0.0
        self.scan_ranges=[]
        self.scan_save=[]
        self.pre_pos_x,self.pre_pos_y=0.0, 0.0
        self.map_pre=[]
        self.map_cur=[]
        self.counter=0

        #defining publisher and subscriber
        self.sub_scan = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        sub_odom = rospy.Subscriber("/odom", Odometry, self.odom_callback)

    def scan_callback(self,msg):
        self.scan_ranges=msg.ranges
        self.scan=msg
        # self.rate.sleep()
    def odom_callback(self,msg):
        self.lock.acquire()

        #position
        self.cur_pos_x = msg.pose.pose.position.x
        self.cur_pos_y = msg.pose.pose.position.y
        #orientation
        cur_r = msg.pose.pose.orientation
        cur_pos1 = tftr.euler_from_quaternion((cur_r.x, cur_r.y, cur_r.z, cur_r.w))
        self.cur_rot = cur_pos1[2]
        #angular velocity
        self.cur_wz=msg.twist.twist.angular.z

        self.lock.release()

    def laser_scan(self):
        while not rospy.is_shutdown():
            data=self.scan_ranges
            x_1,y_1=[],[] #for scatter plot
            for angle in range(len(self.scan_ranges)):
                theta=angle/len(self.scan_ranges)*360 #for converting angle as per lidar angle area
                if self.scan_ranges[angle]=="inf": 
                    pass
                else:
                    x_range= self.scan_ranges[angle]*cos(radians(theta)) #sin, cos in radian
                    y_range= self.scan_ranges[angle]*sin(radians(theta)) #sin, cos in radian
                    x_range,y_range = self.rotate_matrix(x_range,y_range,self.cur_rot) #for rotation matrix
                    x_range=x_range+self.cur_pos_x #adding robot x
                    y_range=y_range+self.cur_pos_y #adding robot y
                    #for dot plot
                    x_1.append(x_range)
                    y_1.append(y_range)

            #send data for plotting if angluar velocity is less
            if self.cur_wz < 0.35 and self.cur_wz > -0.35:
                self.map_plot(x_1, y_1)
            else:
                pass
        
            self.scan_save.append(data)
            np.savetxt('/turtlebot3_ws/scripts/maps/laser_scan_data.csv',np.array(self.scan_save), fmt='%s')
            self.pre_pos_x,self.pre_pos_y=self.cur_pos_x,self.cur_pos_y
            self.rate.sleep()

    def map_plot(self, x_1, y_1):
        # plt.clf()
        plt.title('Laser Scan')
        plt.scatter(x_1, y_1 ,s=0.5, c='black', zorder=2) #for dots
        plt.plot((self.pre_pos_x,self.cur_pos_x),(self.pre_pos_y,self.cur_pos_y),c='r', linewidth=1) #for robot position
        #To plot orientation of robot
        # marker, scale = self.robot_marker(self.cur_rot) 
        # markersize = 10 #Size of robot
        # plt.scatter(self.cur_pos_x, self.cur_pos_y, marker=marker, s=(markersize*scale)**2, color='k') #Plotting robot position and orientation
        start_x,end_x=plt.xlim()
        start_y,end_y=plt.ylim()
        plt.yticks(np.arange(start_x, end_x, 0.25))
        plt.xticks(np.arange(start_y, end_y, 0.25))
        # plt.xlim([-4,4])
        # plt.ylim([-4,4])
        plt.grid(visible=True)
        plt.pause(.001)
        plt.show
        plt.savefig('/turtlebot3_ws/scripts/maps/scan_map.png')
        self.pre_pos_x,self.pre_pos_y=self.cur_pos_x,self.cur_pos_y #previos position memory

    def rotate_matrix (self, x, y, angle):
        # Rotation matrix multiplication to get rotated x & y
        xr = (x * cos(angle)) - (y * sin(angle)) 
        yr = (x * sin(angle)) + (y * cos(angle))
        return xr, yr

    def robot_marker(self,angle):
        #Creating marker for robot
        arr = np.array([[0, 1],[.3, .1], [-.3, .1], [.1, .3]])  # robot shape
        rot_mat = np.array([
            [cos(angle+pi/2), -sin(angle+pi/2)],
            [sin(angle+pi/2), cos(angle+pi/2)]
            ])
        arr = matmul(arr, rot_mat)  # rotates the arrow

        # scale
        x0 = amin(arr[:, 0])
        x1 = amax(arr[:, 0])
        y0 = amin(arr[:, 1])
        y1 = amax(arr[:, 1])
        scale = amax(abs([x0, x1, y0, y1]))
        codes = [mpl.path.Path.MOVETO, mpl.path.Path.LINETO,mpl.path.Path.LINETO, mpl.path.Path.CLOSEPOLY]
        arrow_head_marker = mpl.path.Path(arr, codes)
        return arrow_head_marker, scale

    def shutdownhook(self):
        self.ctrl_c=True
   
if __name__=="__main__":
   rospy.init_node('laser_scan')
   rosbot_obj=rosbot()
   rosbot_obj.laser_scan()
   # rosbot_obj.save_map()
   # try:
   #    rosbot_obj.map()
   # except rospy.ROSInterruptExecution:
   #    pass