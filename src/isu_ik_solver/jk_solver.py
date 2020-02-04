#! /usr/bin/env python
import rospy
import math
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from time import sleep

g_cart_angle = None
g_robot_radius = None
g_robot_tread = None
g_wheel_radius = None
g_rate = None

#t = 0

def jk_solver(vel,cart_angle,radius,tread,wheelr, i):
    global g_cart_angle,g_rate
    w_perimeter = math.pi*2*wheelr

    if i < 20 :
        l_spd = i * 1.3 / 20
        r_spd = i * 1.3 / 20
        w_spd = 0
    elif i >= 20 and i < 100 :
        l_spd = 1.4
        r_spd = 1.4
    elif i >= 100 and i <= 120 :
        l_spd = 1.4 - ((i - 100) * 1.4 / 20)
        r_spd = 1.4 - ((i - 100) * 1.4 / 20)
   
    else :
        l_spd = 0.0
        r_spd = 0.0

    axes_vel = [l_spd,r_spd,i]#-w_spd]
    return axes_vel

def twist_cb(vel):
    global g_cart_angle,g_robot_radius,g_robot_tread,g_wheel_radius, t
    if vel.data is True:
      sleep(1.5)
      for t in range(120):
        float_spds = jk_solver(vel,g_cart_angle,g_robot_radius,g_robot_tread,g_wheel_radius, t)
        float_pub.publish(data = float_spds)
#        t = t + 1
        sleep(0.05)

def fetch_param(name,default):
    if rospy.has_param(name):
        return rospy.get_param(name)
    else:
        return default
        
if __name__=='__main__':
    rospy.init_node('jk_solver')
    float_pub = rospy.Publisher('/wg/axes_vel',Float32MultiArray,queue_size=1)
    rospy.Subscriber('/wg/isu_state',Bool,twist_cb)
    g_cart_angle = 0.0
    g_robot_radius = fetch_param('~active_caster_radius',0.139625)
    g_robot_tread = fetch_param('~active_caster_half_tread',0.139625)
    g_wheel_radius = fetch_param('~wheel_radius',0.05)
    g_rate = fetch_param('~rate',20)
    rospy.spin()
