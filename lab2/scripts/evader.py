#!/usr/bin/python2.7

import rospy
from sensor_msgs.msg import LaserScan


class Evader:

    def __init__(self):
        self.drive_pub = None
        self.max_speed = 0
        self.max_steering_angle = None
        self.scan_data = None
        self.current_speed = None
        self.current_angle = None
        self.flag = -1
    
    def scan_callback(self,data):
        self.scan_data = data.ranges
        print(data)
        # drive_msg.steering_angle =  self.current_angle      
        # drive_msg.speed = self.current_speed
        # drive_st_msg.drive = drive_msg


        # self.drive_pub.publish(drive_st_msg)



    def run_evader(self):
        rospy.init_node('per', anonymous=True)
        # drive_topic = rospy.get_param("~evader_drive_topic")
        scan_topic = rospy.get_param("~scan_topic")
        # self.max_speed = rospy.get_param("~max_speed", default=0.0)
        # self.max_steering_angle = rospy.get_param("~max_steering_angle", default=0.0)
        # self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10)
        rospy.Subscriber(scan_topic,LaserScan,self.scan_callback)
        rospy.spin()

if __name__ == "__main__":
    ev = Evader()
    ev.run_evader()
