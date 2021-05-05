#!/usr/bin/env python

import sys
import time

import rospy
from std_msgs.msg import String
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped
from drones_controller.msg import DronePath


sys.path.append('/home/eyal/ros_workspaces/src/drones_sim/drones_controller/scripts/Algo')
from Grid import *


def addPointToPath(pathArr, x, y, z):

    goal = PoseStamped()

    goal.header.seq = 1
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = "map" #TODO

    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.position.z = z

    goal.pose.orientation.x = 0.0
    goal.pose.orientation.y = 0.0
    goal.pose.orientation.z = 0.0
    goal.pose.orientation.w = 1.0

    pathArr.append(goal)



rospy.init_node('run_algo_with_sim_node')

droneId = 0

armPub = rospy.Publisher('/uav'+str(droneId)+'/safety/cmd/arm', Empty, queue_size=10)
offboardModePub = rospy.Publisher('/uav'+str(droneId)+'/safety/cmd/set_mode', String, queue_size=10)

setPath0Pub = rospy.Publisher('/uav0/safety/set_path', DronePath, queue_size=10)
setPath1Pub = rospy.Publisher('/uav1/safety/set_path', DronePath, queue_size=10)
setPath2Pub = rospy.Publisher('/uav2/safety/set_path', DronePath, queue_size=10)
setPath3Pub = rospy.Publisher('/uav3/safety/set_path', DronePath, queue_size=10)


drone1PathMsg = DronePath()
drone1PathMsg.waypoints = []
addPointToPath(drone1PathMsg.waypoints, 2,2,5)
addPointToPath(drone1PathMsg.waypoints, 2,-2,5)
addPointToPath(drone1PathMsg.waypoints, -2,-2,5)
addPointToPath(drone1PathMsg.waypoints, -2,2,5)
drone1PathMsg.waypointWaitSeconds = 2.0
drone1PathMsg.isPathCyclic = False

drone2PathMsg = DronePath()
drone2PathMsg.waypoints = []
addPointToPath(drone2PathMsg.waypoints, 2,2,3)
addPointToPath(drone2PathMsg.waypoints, 2,-2,3)
addPointToPath(drone2PathMsg.waypoints, -2,-2,3)
addPointToPath(drone2PathMsg.waypoints, -2,2,3)
drone2PathMsg.waypointWaitSeconds = 2.0
drone2PathMsg.isPathCyclic = False

drone3PathMsg = DronePath()
drone3PathMsg.waypoints = []
addPointToPath(drone3PathMsg.waypoints, 2,2,1)
addPointToPath(drone3PathMsg.waypoints, 2,-2,1)
addPointToPath(drone3PathMsg.waypoints, -2,-2,1)
addPointToPath(drone3PathMsg.waypoints, -2,2,1)
drone3PathMsg.waypointWaitSeconds = 2.0
drone3PathMsg.isPathCyclic = False


emptyMessage = Empty()
time.sleep(2.0)
armPub.publish(emptyMessage)

offboardMsg = String()
offboardMsg.data = 'OFFBOARD'
time.sleep(2.0)
offboardModePub.publish(offboardMsg)


time.sleep(2.0)

# pathArr = [setPath0Pub,setPath1Pub,setPath2Pub,setPath3Pub]
# msgArr = [drone0PathMsg,drone1PathMsg,drone2PathMsg,drone3PathMsg]

# for i in range(4):
#     for j in range(4):
#         pathArr[j].publish(msgArr[(i+j)%4])
#         time.sleep(2.0)

# setPath1Pub.publish(drone1PathMsg)
# setPath2Pub.publish(drone2PathMsg)
# setPath3Pub.publish(drone3PathMsg)



grid = Grid(4, 20, 1)

cell1 = grid.next_cell((0, 0))

cell2 = grid.next_cell(cell1)


drone0PathMsg = DronePath()
drone0PathMsg.waypoints = []
addPointToPath(drone0PathMsg.waypoints, cell1[0], cell1[1], 7)
addPointToPath(drone0PathMsg.waypoints, cell2[0], cell2[1], 10)
drone0PathMsg.waypointWaitSeconds = 2.0
drone0PathMsg.isPathCyclic = False


setPath0Pub.publish(drone0PathMsg)

# i = 0
# while True:
#     print i
#     pathArr[i%4].publish(msgArr[i%4])
#     time.sleep(2.0)
#     i+=1

time.sleep(2.0)

print 'published'
sys.exit()