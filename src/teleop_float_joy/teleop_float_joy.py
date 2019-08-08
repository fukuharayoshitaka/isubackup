#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray

def joy_cb(msg,float_pub):
    floatdata = [None,None]
    floatdata[0]=(1*msg.axes[1])
    floatdata[1]=(1*msg.axes[2])
    float_pub.publish(data=floatdata)

if __name__== '__main__':
    rospy.init_node('teleop_float_joy')
    float_pub=rospy.Publisher('axes_vel',Float32MultiArray,queue_size=1)
    rospy.Subscriber('joy',Joy,joy_cb,float_pub)
    rospy.spin()
