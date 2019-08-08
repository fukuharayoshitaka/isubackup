#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

g_twist_pub = None
g_target_twist = None 
g_last_twist = None
g_last_twist_send_time = None
g_accel_scales = [0.1, 0.1, 0.1]
g_max_accel = [1, 1, 1] # units: meters per second^2 [x,y,z]
g_cog_max = None #units: milimeter
g_vel_max = [0.5,0.5]
g_angular_max = None
g_cogpos = Float32MultiArray()#Centor Of Gravity Position on seat [x,y]
g_deadzone = None
g_state = False

def accel_vel(v_prev, cogpos, t_prev, t_now, accel_scale,accel_max,v_max):
  v_target = v_prev + (cogpos * accel_scale * (t_now - t_prev).to_sec())
  if v_target > v_max:
    v_target = v_max
  elif v_target < -v_max:
    v_target = -v_max 
  # compute maximum velocity step
  step = accel_max * (t_now - t_prev).to_sec()
  sign = 1.0 if (v_target > v_prev) else -1.0
  error = math.fabs(v_target - v_prev)
  if error < step: # we can get there within this timestep. we're done.
    return v_target
  else:
    return v_prev + sign * step  # take a step towards the target

def accel_twist(prev, cogpos, t_prev, t_now, accel_scale,accel_max, v_max):
    tw = Twist()
    tw.linear.x = accel_vel(prev.linear.x, cogpos[0], t_prev,
                           t_now, accel_scale[0],accel_max[0],v_max)
    tw.linear.y = accel_vel(prev.linear.y, cogpos[1], t_prev,
                           t_now, accel_scale[1],accel_max[1],v_max)
    #tw.angular.z = ramped_vel(prev.angular.z, target.angular.z, t_prev,
    #                        t_now, ramps[2])
    return tw

def calc_cogpos(cogpos,cog_max,deadzone):
    cogpos_cmd=[None,None]
    #convert cartesian to polar coordinate
    vel_angle = math.atan2(cogpos.data[1],cogpos.data[0])
    vel_r = math.sqrt(math.pow(cogpos.data[0],2)+math.pow(cogpos.data[1],2))
    #set maxcog
    if vel_r > cog_max:
      vel_r = cog_max
    #set deadzone
    if vel_r < deadzone:
        vel_r = 0.0
    #convert polar to cartesian coordinate
    cogpos_cmd[0] = math.cos(vel_angle)*vel_r
    cogpos_cmd[1] = math.sin(vel_angle)*vel_r
    return cogpos_cmd

def send_twist():
    global g_last_twist_send_time, g_target_twist, g_last_twist,\
            g_accel_scales,g_max_accel, g_twist_pub,g_cogpos,\
            g_vel_max,g_deadzone,g_cog_max,g_state
    cogpos_cmd=calc_cogpos(g_cogpos,g_cog_max,g_deadzone)
    t_now = rospy.Time.now()
    g_last_twist = accel_twist(g_last_twist, cogpos_cmd,
                              g_last_twist_send_time, t_now, g_accel_scales,g_max_accel,g_vel_max)
    g_last_twist_send_time = t_now
    if g_state is False:
      g_last_twist.linear.x = 0.0
      g_last_twist.linear.y = 0.0
    g_twist_pub.publish(g_last_twist)

def cogpos_cb(msg):
    global g_cogpos
    g_cogpos = msg
    send_twist()

def state_cb(msg):
  global g_state
  g_state = msg.data

def fetch_param(name, default):
  if rospy.has_param(name):
    return rospy.get_param(name)
  else:
    return default

if __name__=="__main__":
    rospy.init_node('cogpos_twist_accel')
    g_accel_scales[0] = fetch_param('~linear_accelscale_x',0.2)
    g_accel_scales[1] = fetch_param('~linear_accelscale_y',0.2)
    #g_accel_scales[2] = fetch_param('~angular_scale_z',1)
    g_max_accel[0] = fetch_param('~linear_maxaccel_x', 0.1)
    g_max_accel[1] = fetch_param('~linear_maxaccel_y', 0.1)
    #g_max_accel[2] = fetch_param('~angular_accel_z', 1)
    g_vel_max = fetch_param('~linear_maxspeed',0.1)
    g_cog_max = fetch_param('~cog_area_max_radius',1.0)
    #g_angular_max = fetch_param("~angular_maxspeed",1.0)
    g_deadzone = fetch_param('~deadzone_radius',0.1)
    g_last_twist_send_time = rospy.Time.now()
    g_target_twist = Twist() # initializes to zero 
    g_last_twist = Twist()
    rospy.Subscriber('cogpos',Float32MultiArray,cogpos_cb)
    rospy.Subscriber('isu_state' ,Bool,state_cb)
    g_twist_pub = rospy.Publisher('cmd_vel',Twist,queue_size=1)
    rospy.spin()