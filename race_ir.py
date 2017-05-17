#!/usr/bin/env python
import rospy

# the message that we get from the arduino
from std_msgs.msg import Int32
from std_msgs.msg import String
from kobuki_msgs.msg import ButtonEvent
from kobuki_msgs.msg import BumperEvent

# the output message controlling the speed and direction of the robot
from geometry_msgs.msg import Twist 

count = 0
sensorData = dict()
Bumped = False

def ir_callback(data):

    global Bumped
    global count
    global pubA1
    global pubA3
    global sensorData

    #--------------参数------------------#
    thre = 200
    # 判断的阈值，左右两边最大值大于这个的时候会转弯，否则直走；此值越大，机器人转弯时离墙越近
    # 如果机器人拐弯方向反了，就把这个值调成负的（别把下面的角速度调成负的，不然打印向左向右拐的信息就反了）
    linearSpeed = 0.2
    #直行时线速度
    angularSpeed = 0.3
    #转弯时角速度
    #--------------参数------------------#


    raw_data = data.data
    # Twist is a message type in ros, here we use an Twist message to control kobuki's speed
    # twist. linear.x is the forward velocity, if it is zero, robot will be static, 
    # if it is grater than 0, robot will move forward, otherwise, robot will move backward
    # twist.angular.axis is the rotatin velocity around the axis 
    # 
    # Around which axis do we have to turn? In wich direction will it turn with a positive value? 
    # Right hand coordinate system: x forward, y left, z up

    twist = Twist()

    



    # write your code here
    count += 1
    A1 = raw_data >> 16
    #A3 = raw_data - (A1 << 16)
    A3 = raw_data & 65535
    sensorData[count]=A1
    if A3 < 600:
        if count > 300:
            T = count
            count = 0
            sl = sorted([sensorData[i] for i in range(10,T//4)], reverse = True)[3]
            sr = sorted([sensorData[i] for i in range(T//4,T//2)], reverse = True)[3]
            # 取出左边和右边的第三大值（不取最大，怕数据波动），两个range函数里的范围可能还需要修改
            if sl - sr < -thre: 
                twist.angular.z = angularSpeed
                twist.linear.x = 0
                print("Turn right")
            elif sl - sr > thre: 
                twist.angular.z = -angularSpeed
                twist.linear.x = 0
                print("Trun left")
            else: 
                twist.linear.x = linearSpeed
                twist.angular.z = 0.0 
                print("Go straight")
            
            print(sl,sr)
            print("-----")
    if Bumped: 
        twist.linear.x = 0.0
        twist.angular.z = 0.0
    #如果遭到撞击，就停止运行，否则继续运行

    kobuki_velocity_pub.publish(twist)  
    
    
def range_controller():
    
    # define the publisher globally
    global kobuki_velocity_pub
    global pubA1
    global pubA3
    
    # initialize the node
    rospy.init_node('range_controller', anonymous=True)

    # initialize the publisher - to publish Twist message on the topic below...
    kobuki_velocity_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
    #pubA1 = rospy.Publisher('Serial_A1', String, queue_size = 10)
    #pubA3 = rospy.Publisher('Serial_A3', String, queue_size = 10)

    # subscribe to the topic '/ir_data' of message type Int32. The function 'ir_callback' will be called
    # every time a new message is received - the parameter passed to the function is the message
    rospy.Subscriber("/ir_data", Int32, ir_callback)
    rospy.Subscriber("/mobile_base/events/button",ButtonEvent,ButtonEventCallback)
    rospy.Subscriber("/mobile_base/events/bumper",BumperEvent,BumperEventCallback)
    #订阅button和bumper的node以获取信息

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def ButtonEventCallback(data):
    global Bumped
    if data.button == ButtonEvent.Button0:
        Bumped = False
        print("B0 pressed")
    elif data.button == ButtonEvent.Button1:
        print("B1 pressed")
        Bumped = False
    else:
        print("B2 pressed")
        Bumped = False
    #按钮被按下的时候清零flag，允许机器人继续运行（目前机器人对所有按钮都会反应）

def BumperEventCallback(data):
    global Bumped
    if data.state == BumperEvent.PRESSED:
        Bumped = True
        print("Opps, bumped.")
    #被撞到的时候将flag置位，以停止机器人运行

# start the line follow
if __name__ == '__main__':
    range_controller()
    
