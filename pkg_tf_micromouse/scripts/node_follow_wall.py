#! /usr/bin/env python

# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *
from pid_tune.msg import PidTune
from std_msgs.msg import Float32,String


import math

active_ = False
pub_ = None
regions_ = {
        'right': 0,
        'fright': 0,
        'front': 0,
        'fleft': 0,
        'left': 0,
}
state_ = 0
state_dict_ = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall',
}


def avg(lst):
    return sum(lst)/len(lst)

def wall_follower_switch(req):
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res

def clbk_laser(msg):
    global regions_
    #360/5 = 72
    regions_ = {
        'right':  min(avg(msg.ranges[0:71]),10.0),
        'fright': min(avg(msg.ranges[72:143]),10.0),
        'front':  min(avg(msg.ranges[144:215]),10.0),
        'fleft':  min(avg(msg.ranges[216:287]),10.0),
        'left':   min(avg(msg.ranges[288:359]),10.0),
    	}
    take_action()

def change_state(state):
    global state_, state_dict_

    if state is not state_:
        print 'Wall follower - [%s] - %s' % (state, state_dict_[state])
        state_ = state

    if state_ == 0:
        msg = find_wall()
    elif state_ == 1:
        msg = turn_left()
    elif state_ == 2:
        msg = follow_the_wall()
    else:
        rospy.logerr('Unknown state!')

    pub_.publish(msg)

def take_action():
    global regions_
    regions = regions_
    msg = Twist()
    linear_x = 0
    angular_z = 0

    state_description = ''

    d = 0.8
    e = 0.8

    if regions['front'] > d and regions['fleft'] > e and regions['fright'] > e:
        state_description = 'case 1 - nothing'
        change_state(0)
    elif regions['front'] < d and regions['fleft'] > e and regions['fright'] > e:
        state_description = 'case 2 - front'
        change_state(1)
    elif regions['front'] > d and regions['fleft'] < e and regions['fright'] < e:
        state_description = 'case 3 - fright'
        change_state(2)
    elif regions['front'] > d and regions['fleft'] < e and regions['fright'] > e:
        state_description = 'case 4 - fleft'
        change_state(2)
    elif regions['front'] < d and regions['fleft'] > e and regions['fright'] < e:
        state_description = 'case 5 - front and fright'
        change_state(2)
    elif regions['front'] < d and regions['fleft'] < e and regions['fright'] > e:
        state_description = 'case 6 - front and fleft'
        change_state(2)
    elif regions['front'] < d and regions['fleft'] < e and regions['fright'] < e:
        state_description = 'case 7 - front and fleft and fright'
        change_state(2)
    elif regions['front'] > d and regions['fleft'] < e and regions['fright'] < e:
        state_description = 'case 8 - fleft and fright'
        change_state(2)
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)
    print(regions_)
    print(state_description)

def find_wall():
    msg = Twist()
    msg.linear.x = 0.2
    msg.angular.z = -0.3
    return msg

def turn_left():
    msg = Twist()
    msg.angular.z = 0.3
    return msg

def follow_the_wall():
    global regions_
    global kp,ki,kd
    global long_error, zero_error
    msg = Twist()
    integ = 0
    diff = 0
    dist = 0.1
    error = dist - regions_['left']
    integ += error
    output = kp*error + ki*integ + kd*diff
    diff = error
    msg.linear.x = 0.1
    msg.angular.z = 0.0 - output
    long_error.publish(output)
    zero_error.publish(0.0)

    return msg

def wall_follow(alt):
    global kp,ki,kd
    kp = alt.Kp * 0.006
    ki = alt.Ki * 0.0008
    kd = alt.Kd * 0.03

def main():
    global pub_, active_
    global kp,ki,kd
    global long_error, zero_error
    kp = 459 * 0.006
    ki = 0
    kd = 1288 *0.03
    rospy.init_node('reading_laser')

    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    sub = rospy.Subscriber('/my_mm_robot/laser/scan', LaserScan, clbk_laser)

    rospy.Subscriber('/pid_tuning_altitude', PidTune, wall_follow)

    long_error = rospy.Publisher('/long_error',Float32, queue_size=1)
    zero_error = rospy.Publisher('/zero_error',Float32, queue_size=1)

    srv = rospy.Service('wall_follower_switch', SetBool, wall_follower_switch)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if not active_:
            rate.sleep()
            continue

        msg = Twist()
        if state_ == 0:
            msg = find_wall()
        elif state_ == 1:
            msg = turn_left()
        elif state_ == 2:
            msg = follow_the_wall()
            pass
        else:
            rospy.logerr('Unknown state!')

        pub_.publish(msg)

        rate.sleep()

if __name__ == '__main__':
    main()
