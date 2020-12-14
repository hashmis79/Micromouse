#!/usr/bin/env python3
# -- coding: utf-8 --
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations

co_ords = []
def clbk_odom(msg):
    global x_dist
    global y_dist
    global yaw_
    # position
    position_ = msg.pose.pose.position
    # gives x and y distance of the bot
    x_dist = position_.x
    y_dist = position_.y
    co_ords.append([round(x_dist,3),round(y_dist,3)])
    # yaw
    # convert quaternions to euler angles, only extracting yaw angle for the robot
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    #print(euler)
    yaw_ = euler[2]
    # print("yaw is here :                                                                                                                   hahaahahahahahah")
    # print(yaw_)
    # print(x_dist, y_dist)

nodes = []#[Prev_node_number,dist_from_prev_node,co_ordinates]
num_nodes = 1
num_edges = 0
node_co_ords = []
prev_node = 1
nodes.append([0,0,[0,0]])

def clbk_laser(msg):
    # 360 / 5 = 72
    # regions = [
    #     min(min(msg.ranges[0:71]), 10),
    #     min(min(msg.ranges[72:143]), 10),
    #     min(min(msg.ranges[144:215]), 10),
    #     min(min(msg.ranges[216:287]), 10),
    #     min(min(msg.ranges[288:359]), 10),
    # ]
    print("Below lines may contain loginfo(regions).....Stay Prepared......Keep the watch")
    # rospy.loginfo(regions)
    region = {
        'p' : msg.ranges[:],
    }
    node_presence = 0
    turn_front = 0
    turn_left = 0
    turn_right = 0
    for i in range(30):
        # print(region['p'][i])
        if(region['p'][i] > 0.2):
            node_presence += 1
            turn_right = 1
        elif(region['p'][i] > 0.07 and region['p'][i] < 0.09):
            turn_right = -1
    # print("Lower happened")
    for i in range(30,330):
        # print(region['p'][i])
        if(region['p'][i] > 0.2):
            node_presence += 1
            turn_front = 1
        elif(region['p'][i] > 0.07 and region['p'][i] < 0.09):
            turn_front = -1
    # print("Mid happened")
    for i in range(330,360):
        # print(region['p'][i])
        if(region['p'][i] > 0.2):
            node_presence += 1
            turn_left = 1
        elif(region['p'][i] > 0.07 and region['p'][i] < 0.09):
            turn_left = -1
    # print("Upper happened")
    if(node_presence > 1 and ((turn_front == -1 or turn_front == 1) and (turn_left == -1 or turn_left == 1) and (turn_right == -1 or turn_right == 1))):
        if(co_ords in node_co_ords):
            nodes[prev_node - 1].append(node_co_ords.index(co_ords),abs(node_co_ords[prev_node - 1] - co_ords),co_ords)
            prev_node = node_co_ords.index(co_ords)
        else:
            node_co_ords.append(co_ords)
            nodes.append([prev_node,abs(node_co_ords[prev_node - 1] - co_ords),co_ords])
            num_nodes += 1
            prev_node = num_nodes
        num_edges += 1
    print(nodes)
    print("These were the nodes until this cycle                                             ..........................................................")




def main():
    prev_node = 1
    rospy.init_node('reading_laser')
    # print("Below lines may contain loginfo(regions).....Stay Prepared......Keep the watch")
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    sub = rospy.Subscriber('/my_mm_robot/laser/scan', LaserScan, clbk_laser)
    co_ords.clear()
    rospy.spin()

if _name_ == '_main_':
    main()
