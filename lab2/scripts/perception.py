#!/usr/bin/python

import rospy
from sensor_msgs.msg import LaserScan,PointCloud2
import laser_geometry.laser_geometry as lg
import math
import random
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point,PoseStamped, PoseWithCovarianceStamped, Pose2D,Twist
from nav_msgs.msg import Odometry





class Perception:

    def __init__(self):
        self.drive_pub = None
        self.pcPub = None
        self.rvPub = None
        self.initialpose_pub = None
        self.velocity_pub = None


    def cal_distance_from_line(self,point,line_point_one,line_point_two):

        y = point[1]
        x = point[0]
        # distance = abs(y0 - (slope* x0) - constant ) / math.sqrt((1 +slope **2 )) 
        numerator = abs((line_point_two[0]-line_point_one[0])*(line_point_one[1]- y)-(line_point_one[0]-x)*(line_point_two[1]-line_point_one[1]))
        denominator = math.sqrt(((line_point_two[0]-line_point_one[0])**2) + ((line_point_two[1]-line_point_one[1]) ** 2))
        distance = numerator/denominator

        return distance

    
    def publish_to_marker(self,best_fit_arr):
        marker = Marker()
        marker.header.frame_id = "/base_link"
        marker.type = Marker.LINE_LIST
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.color.a = 1.0
        marker.color.g = 1.0;
        marker.scale.x = 0.02
        for i in range(len(best_fit_arr)):
            if best_fit_arr[i] != None:
                for j in range(len(best_fit_arr[i])):
                    p = Point()
                    p.x = best_fit_arr[i][j][0]
                    p.y = best_fit_arr[i][j][1]
                    marker.points.append(p)

        if marker.points != None:
            self.rvPub.publish(marker)



    def scan_callback(self,data):
        self.scan_data = data.ranges
        angel_min = data.angle_min
        angle_max = data.angle_max
        angle_increment = data.angle_increment
        pi = math.pi
        co_ordinates = []

        for i in range(len(data.ranges)):
            theta_rad = angel_min + (angle_increment * i)
            rad = data.ranges[i]
            if rad < 3:
                x = rad * math.cos(theta_rad)
                y = rad * math.sin(theta_rad)
                co_ordinates.append((x,y))

        min_coordinate_len = len(co_ordinates) * 0.1
        best_fit_length = 0
        best_fit_arr = []
        while len(co_ordinates) > min_coordinate_len :
            best_fit_length = 0
            best_fit_sample = None
            K_iterations = 40
            current_outliners = []
            for i in range(K_iterations):
                first_sample = random.choice(co_ordinates)   
                second_sample = random.choice(co_ordinates)                
                if first_sample == second_sample:
                    continue
                inliners = []
                outliners = []
                for i in co_ordinates:
                    dis = self.cal_distance_from_line(i,first_sample,second_sample)
                    if dis < 0.09 :
                        inliners.append(i)
                    else :
                        outliners.append(i)
                current_inliners_length = len(inliners)

                if current_inliners_length > best_fit_length and current_inliners_length > min_coordinate_len:
                    best_fit_length = current_inliners_length
                    a = inliners[0]
                    b = inliners[-1]
                    best_fit_sample = [a,b]
                    current_outliners = outliners
            best_fit_arr.append(best_fit_sample)
            co_ordinates = current_outliners

        if len(best_fit_arr) > 0:
            self.publish_to_marker(best_fit_arr)
      



    def run_evader(self):
        rospy.init_node('~', anonymous=True)
        rospy.Subscriber('/base_scan',LaserScan,self.scan_callback)
        self.rvPub = rospy.Publisher("ransac_vis", Marker, queue_size=10)
        rospy.spin()

if __name__ == "__main__":
    pe = Perception()
    pe.run_evader()
