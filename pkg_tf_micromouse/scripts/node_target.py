#! /usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist,Point
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *
from pid_tune.msg import PidTune
from std_msgs.msg import Float32,String
import math

position_ = Point()
active_ = False
pub_ = None

# nodelist = [['x y coordinates, distance betw nodes, node type, direction']]
nodelist = [['1.2234253454 1.2342345532 0.9321323,2 1']]
# node cases = {0 - left, 1 - right,2 - left and right, 3 - left and straight, 4 - right and straight, 5 - all , 6 - Uturn}
# priority = {0 - Left, 1 - Straight, 2 - Right, 3 - Uturn}
# 'Wall follower - [%s] - %s' % (state, state_dict_[state])


def clbk_laser(msg):
    global regions_
    #360/5 = 72
    regions_ = {
        'right':  min(min(msg.ranges[0:29]),10.0),
        'fright': min(min(msg.ranges[72:143]),10.0),
        'front':  min(min(msg.ranges[159:215]),10.0),
        'fleft':  min(min(msg.ranges[216:287]),10.0),
        'left':   min(min(msg.ranges[330:359]),10.0),
    	}
    take_action()

def clbk_odom(msg):
    global position_
    global yaw_

    # position
    position_ = msg.pose.pose.position

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]

#Func for editing node list
def edit_node_list(case, dir):
    global position_
    print('case, dir')
    # prev_node = nodelist[-1].split(' ')
    # posx = prev_node
    # distance = if(nodelist[-1])
    # new = '%s %s %s %s' % (position_.x, position_.y, distance, case, dir)
    print('New node added')

# func for following the wall
def follow_the_wall():
    global regions_
    global kp,ki,kd
    # global long_error, zero_error
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
    # long_error.publish(output)
    # zero_error.publish(0.0)
    pub_.publish(msg)

#func for stopping the bot
def stop_bot():
    msg = Twist()
    msg.linear.x = 0.0
    msg.angular.z = 0.0
    pub_.publish(msg)

#func for classifying the
def take_action():
    global regions_
    regions = regions_
    msg = Twist()
    linear_x = 0
    angular_z = 0

    state_description = ''

    d = 0.2
    e = 0.2
    if regions['front'] > d and regions['left'] < e and regions['right'] < e:
        state_description = 'case 8 - fleft and fright'
        follow_the_wall()
    else:
        stop_bot()
        if regions['front'] < d and regions['left'] > e and regions['right'] > e:
            state_description = 'case 2 - front obs'
            edit_node_list(2,0)
        elif regions['front'] > d and regions['left'] > e and regions['right'] < e:
            state_description = 'case 3 - fright'
            edit_node_list(3,0)
        elif regions['front'] > d and regions['left'] > e and regions['right'] > e:
            state_description = 'case 3 - no obs'
            edit_node_list(5,0)
        elif regions['front'] > d and regions['left'] < e and regions['right'] > e:
            state_description = 'case 4 - left obs'
            edit_node_list(4,1)
        elif regions['front'] < d and regions['left'] > e and regions['right'] < e:
            state_description = 'case 5 - front and right obs'
            edit_node_list(0,0)
        elif regions['front'] < d and regions['left'] < e and regions['right'] > e:
            state_description = 'case 6 - front and left obs'
            edit_node_list(1,2)
        elif regions['front'] < d and regions['left'] < e and regions['right'] < e:
            state_description = 'case 7 - front and fleft and fright'
            edit_node_list(6,3)
        else:
            state_description = 'unknown case'
            rospy.loginfo(regions)
        print(regions_)
    print(state_description)

def wall_follower_switch(req):
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res

def main():
    global pub_, active_
    global kp,ki,kd
    # global long_error, zero_error
    kp = 459 * 0.006
    ki = 0
    kd = 1288 *0.03
    rospy.init_node('reading_laser')

    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    sub = rospy.Subscriber('/my_mm_robot/laser/scan', LaserScan, clbk_laser)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)

    # rospy.Subscriber('/pid_tuning_altitude', PidTune, wall_follow)

    # long_error = rospy.Publisher('/long_error',Float32, queue_size=1)
    # zero_error = rospy.Publisher('/zero_error',Float32, queue_size=1)

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
