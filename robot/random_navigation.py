# This script is used for the robot patrol.
# The next target point is chosen randomly and the robot will move there by using move_base.


#!/usr/bin/env python

import rospy
import actionlib
import tf
from nav_msgs.msg import Odometry
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
import numpy as np

global previous_status, client, pose, current_room

def callback(data):
    global previous_status, client, pose, current_room

    rooms = np.array((
    (-10,10),
    (-10,10-20/3),
    (-10,-10+20/3),
    (2.5,10),
    (2.5,10-20/3),
    (2.5,-10+20/3)))

    xy = np.array((0.5,-0.5))

    rooms_nav = rooms+xy

    if not data.status_list:
        return

    goal = data.status_list.pop()

    # When the navigation has started
    if (goal.status == GoalStatus.ACTIVE and goal.status != previous_status):
        rospy.loginfo("navigation: " + goal.text)
        rospy.loginfo("New Goal is ({0},{1}) (ROOM {2})".format(pose[0],pose[1],current_room+1))
        previous_status = goal.status

    # When the robot set an unreachable goal, re-chose the proper goal.
    elif (goal.status == GoalStatus.ABORTED and goal.status != previous_status):
        rospy.loginfo("navigation: " + goal.text)
        rospy.loginfo("Setting new goal")
        previous_status = goal.status

        w = np.random.random() * (7.5-xy[0]*2)
        h = np.random.random() * (20/3-xy[1]*2)
        pose = (rooms_nav[current_room][0]+w,rooms_nav[current_room][1]-h)

        # pose = np.random.random(2) * 18 - 9 # new goal location (-9.0<x<9.0, -9.0<y<9.0)
        goal = goal_pose(pose)
        client.send_goal(goal)

    # When the robot reached the goal or lost
    elif (goal.status != GoalStatus.ACTIVE and goal.status != GoalStatus.PENDING and
        goal.status != previous_status):
        rospy.loginfo("navigation: " + goal.text)
        previous_status = goal.status

        current_room = (current_room+1)%6

        w = np.random.random() * (7.5-xy[0]*2)
        h = np.random.random() * (20/3+xy[1]*2)
        pose = (rooms_nav[current_room][0]+w,rooms_nav[current_room][1]-h)

        # pose = np.random.random(2) * 18 - 9 # new goal location (-9.0<x<9.0, -9.0<y<9.0)
        goal = goal_pose(pose)
        client.send_goal(goal)



def goal_pose(pose):
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose.position.x = float(pose[0])
    goal_pose.target_pose.pose.position.y = float(pose[1])
    goal_pose.target_pose.pose.position.z = 0.0
    goal_pose.target_pose.pose.orientation.x = 0.0
    goal_pose.target_pose.pose.orientation.y = 0.0
    goal_pose.target_pose.pose.orientation.z = 0.0
    goal_pose.target_pose.pose.orientation.w = 1.0

    return goal_pose


if __name__ == '__main__':

    global client, pose, current_room

    pose = [0,0]
    current_room = 5

    rospy.init_node('patrol')
    listener = tf.TransformListener()

    rospy.Subscriber('move_base/status', GoalStatusArray, callback)
    previous_status = 0

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    listener.waitForTransform("map", "base_link", rospy.Time(), rospy.Duration(4.0))

    goal = goal_pose(pose)
    client.send_goal(goal)

    rospy.spin()