#!/usr/bin/env python
import rospy

# the message that we get from the arduino
from std_msgs.msg import Int32
from std_msgs.msg import String

# the output message controlling the speed and direction of the robot
from geometry_msgs.msg import Twist 

count = 0
sensorData = dict()

def ir_callback(data):

    global count
    global pubA1
    global pubA3
    global sensorData
    thre = 400
    raw_data = data.data
    # Twist is a message type in ros, here we use an Twist message to control kobuki's speed
    # twist. linear.x is the forward velocity, if it is zero, robot will be static, 
    # if it is grater than 0, robot will move forward, otherwise, robot will move backward
    # twist.angular.axis is the rotatin velocity around the axis 
    # 
    # Around which axis do we have to turn? In wich direction will it turn with a positive value? 
    # Right hand coordinate system: x forward, y left, z up

    twist = Twist()
    twist.linear.x = 1.0
    twist.angular.z = 0.0
    



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
            sl,sr = 0,0
            for i in range(10,T//4):
                if sensorData[i] > sl: sl = sensorData[i]
            for i in range(T//4,T//2):
                if sensorData[i] > sr: sr = sensorData[i]
            if sl > thre or sr > thre:
                if sl < sr: 
                    twist.angular.z = 0.5
                    print("Turn right")
                else: 
                    twist.angular.z = -0.5
                    print("Trun left")
            else: 
                twist.angular.z = 0.0 
                print("Go straight")
            #for i in sensorData: print(i,sensorData[i])
            print(sl,sr)
            print("-----")
    #print (A1)
                           
              
            
    #try:
    '''rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        Serial_A1 = String(str(A1))
        Serial_A3 = String(str(A3))
        rospy.loginfo(Serial_A1)
        pubA1.publish(Serial_A1)
        rospy.loginfo(Serial_A3)
        pubA3.publish(Serial_A3)
        rate.sleep()'''
    #except rospy.ROSInterruptException:
    #    pass
                    
    #print('A1 = {}, A3 = {}'.format(A1, A3))
    #print('No.{}\tA1 = {}, A3 = {} T = {}'.format(count, A1, A3, T))
    #print("A1 = {}".format(A1))
    #print(T)


	
    # actually publish the twist message
    #print(count)
    '''if A1 < 400:
        twist.linear.x = 0.0
    else:
        if count < T // 4:
            twist.angular.z = 0    # clockwise
            #print('Turning right                                count =', count)
        elif (count > T // 4) and (count < T // 2):
            twist.angular.z = 0  # anti-clockwise
            #print('Turning left count =', count)
        else:
            twist.linear.x = 0.0'''
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

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

# start the line follow
if __name__ == '__main__':
    range_controller()
    
