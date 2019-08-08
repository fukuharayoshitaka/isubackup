#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray

g_stg = [0.01, 0.01, 0.01, 0.01] 
g_start_pub=None
g_stg_init_flg = True

def stgage_cb(msg):
    global g_stg
    for i in range(4):
        g_stg[i]=msg.data[i]
        hantai(g_stg)

def joy_cb(msg):
    global g_stg_init_flg
    if msg.buttons[6]==1 and msg.buttons[7]==1:
        if g_stg_init_flg is True:
            rospy.logwarn('start')
            g_stg_init_flg = False

def hantai(stg)
    global g_stg
    if(g_stg_init_flg = True)
        if (stg[0]+stg[1]+stg[2]+stg[3]) != 0:
            if(reat_count==100)
                start = False
                g_start_pub.publish(start)        

if __name__=="__main__":
    rospy.init_node('start_cog')
    rospy.Subscriber('stg_array',Int32MultiArray,stgage_cb)
    rospy.Subscriber('joy',Joy,joy_cb)
    g_start_pub = rospy.Publisher('start',Float32MultiArray,queue_size=1)
    g_rate = fetch_param('~rate',20)
    rate = rospy.Rate(g_rate)
    reat_count += 1

