#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
import wiringpi as w

w.wiringPiSetup()
w.pinMode(21,1)

g_run_state = False

def send_state(state):
    g_state_pub.publish(state)

def joy_cb(msg):
    global g_run_state
    if msg.buttons[6]==1 and msg.buttons[7]==1:
        if g_run_state is False:
            g_run_state = True
            send_state(True)
            rospy.logwarn('start')
	    w.digitalWrite(21,0)
            return
            
    if msg.buttons[4]==1 and msg.buttons[5]==1:
        if g_run_state is True:
            g_run_state = False
            send_state(False)
            rospy.logwarn('stop')
	    w.digitalWrite(21,1)


if __name__ == "__main__":
    w.digitalWrite(21,1)
    rospy.init_node('isu_state')
    rospy.Subscriber('joy',Joy,joy_cb)
    g_state_pub = rospy.Publisher('isu_state',Bool,queue_size=1)
    rospy.spin()
