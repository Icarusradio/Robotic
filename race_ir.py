#!/usr/bin/env python
import rospy
import time
import math

# the message that we get from the arduino
from std_msgs.msg import Int32

# the type that we want to publish
from sensor_msgs.msg import LaserScan

# in order to get the buttom and bumper status
from kobuki_msgs.msg import ButtonEvent
from kobuki_msgs.msg import BumperEvent

# the output message controlling the speed and direction of the robot
from geometry_msgs.msg import Twist

# the bumper is not bumped at first
Bumped = False
Range = []
count = 0
FristPeriod = True
timer = 0
velocity = 0.0
angle_velocity = 0.0

def ir_callback(data):
    # define the publisher globally
    global kobuki_velocity_pub
    global scan_pub
    global Bumped
    global count
    global Range
    global timer
    global velocity
    global angle_velocity
    # Twist is a message type in ros, here we use an Twist message to control kobuki's speed
    # twist. linear.x is the forward velocity, if it is zero, robot will be static,
    # if it is grater than 0, robot will move forward, otherwise, robot will move backward
    # twist.angular.axis is the rotatin velocity around the axis
    #
    # Around which axis do we have to turn? In wich direction will it turn with a positive value?
    # Right hand coordinate system: x forward, y left, z up

    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.0



    # write your code here
    A1 = data.data >> 16   # the big sensor
    A3 = data.data & 65535 # the small sensor
    #print("A1 = {}, A3 = {}".format(A1, A3))
    count += 1
    if A3 < 65:
        if (count > 340) and (count < 380):
            #print(count, len(Range))
            MaxRight = max(Range[0:(count - 1) // 4])
            MaxLeft = max(Range[(count - 1) // 4:(count - 1) // 2])
            if MaxRight - MaxLeft > 200:
                twist.angular.z = 0.5
                angle_velocity = 0.5
                velocity = 0.0
                print('Turn Left')
            elif MaxLeft - MaxRight > 200:
                twist.angular.z = -0.5
                angle_velocity = -0.5
                velocity = 0.0
                print('Turn Right')
            else:
                twist.linear.x = 0.1
                angle_velocity = 0.0
                velocity = 0.1
                print('Go straight')
            print(MaxLeft, MaxRight)
            '''temp = time.clock()
            delta_time = temp - timer
            #print("Delta time = {}".format(delta_time))
            #print("Time increment = {}".format(delta_time / count))
            scan_pub = rospy.Publisher('scan', LaserScan, queue_size=50)
            r = rospy.Rate(1.0)
            if not rospy.is_shutdown():
                current_time = rospy.Time.now()
                scan = LaserScan()
                scan.header.stamp = current_time
                scan.header.frame_id = 'laser_frame'
                scan.angle_min = -math.pi
                scan.angle_max = math.pi
                scan.angle_increment = 2 * math.pi / count
                scan.time_increment = delta_time / count
                scan.range_min = 0
                scan.range_max = 1023
                scan.ranges = []
                scan.intensities = []
                for i in Range:
                    scan.ranges.append(i)
                scan_pub.publish(scan)
                r.sleep()'''
        else:
            twist.linear.x = velocity
            twist.angular.z = angle_velocity
        count = 0
        Range = []
    else:
        twist.linear.x = velocity
        twist.angular.z = angle_velocity
        Range.append(A1)




    # if bumped, stop the robot
    if Bumped:
        twist.linear.x = 0.0
        twist.angular.z = 0.0

    # actually publish the twist message
    kobuki_velocity_pub.publish(twist)

def ButtonEventCallback(data):
    global Bumped
    if Bumped and data.button == ButtonEvent.Button0:
        Bumped = False
        print("B0 pressed, reseume.")
    elif data.button == ButtonEvent.Button0:
        print("B0 pressed, nothing happens.")
    elif data.button == ButtonEvent.Button1:
        print("B1 pressed.")
    else:
        print("B2 pressed.")


def BumperEventCallback(data):
    global Bumped
    if data.state == BumperEvent.PRESSED:
        Bumped = True
        print("Opps, bumped.")


def range_controller():

    # define the publisher globally
    global kobuki_velocity_pub

    # initialize the node
    rospy.init_node('range_controller', anonymous=True)

    # initialize the publisher - to publish Twist message on the topic below...
    kobuki_velocity_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)

    # initialize the publisher - to publish LaserScan message on the topic below...
    scan_pub = rospy.Publisher('/scan', LaserScan, queue_size=50)

    # subscribe to the topic '/ir_data' of message type Int32. The function 'ir_callback' will be called
    # every time a new message is received - the parameter passed to the function is the message
    rospy.Subscriber("/ir_data", Int32, ir_callback)

    # subscribe to the topic in order to get the bumper and buttom status
    rospy.Subscriber("/mobile_base/events/button", ButtonEvent, ButtonEventCallback)
    rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, BumperEventCallback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

# start the line follow
if __name__ == '__main__':
    range_controller()
