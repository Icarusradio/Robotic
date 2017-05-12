#!/usr/bin/env python
import rospy
import time
import math

# the message that we get from the arduino
from std_msgs.msg import Int32
from sensor_msgs.msg import LaserScan

# the output message controlling the speed and direction of the robot
from geometry_msgs.msg import Twist 

list_ang = []
count = 0
timer = 0

def ir_callback(data):

    global list_ang
    global count
    global timer
    # Twist is a message type in ros, here we use an Twist message to control kobuki's speed
    # twist. linear.x is the forward velocity, if it is zero, robot will be static, 
    # if it is grater than 0, robot will move forward, otherwise, robot will move backward
    # twist.angular.axis is the rotatin velocity around the axis 
    # 
    # Around which axis do we have to turn? In wich direction will it turn with a positive value? 
    # Right hand coordinate system: x forward, y left, z up

    #twist = Twist()
    #twist.linear.x = 0.
    #twist.angular.z = 0. 



    # write your code here
    raw_data = data.data
    ang = raw_data >> 16
    dis = raw_data & 65535
    count += 1
    list_ang.append(ang)
    if (ang < 280) and (count > 20):
        temp = time.clock()
        delta_time = temp - timer
        if delta_time < 0.2:
            print("Delta time = {}".format(delta_time))
            print("Time increment = {}".format(delta_time / count))
            scan_pub = rospy.Publisher('scan', LaserScan, queue_size=50)
            r = rospy.Rate(1.0)
            if not rospy.is_shutdown():
                current_time = rospy.Time.now()
                scan = LaserScan()
                scan.header.stamp = current_time
                scan.header.frame_id = 'laser_frame'
                scan.angle_min = -math.pi / 2
                scan.angle_max = math.pi / 2
                scan.angle_increment = math.pi / count
                scan.time_increment = delta_time / count
                scan.range_min = 200
                scan.range_max = 950
                scan.ranges = []
                scan.intensities = []
                for i in list_ang:
                    scan.ranges.append(i)
                scan_pub.publish(scan)
                r.sleep()
        list_ang = []
        count = 0
        timer = temp

    # actually publish the twist message
    #kobuki_velocity_pub.publish(twist)  
    
    
def range_controller():
    
    # define the publisher globally
    #global kobuki_velocity_pub
    
    # initialize the node
    #rospy.init_node('range_controller', anonymous=True)
    rospy.init_node('laser_scan_publisher')

    # initialize the publisher - to publish Twist message on the topic below...
    #kobuki_velocity_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)

    # subscribe to the topic '/ir_data' of message type Int32. The function 'ir_callback' will be called
    # every time a new message is received - the parameter passed to the function is the message
    rospy.Subscriber("/ir_data", Int32, ir_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

# start the line follow
if __name__ == '__main__':
    range_controller()
    
