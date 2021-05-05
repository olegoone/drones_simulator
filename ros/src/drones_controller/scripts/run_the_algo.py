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


grid = Grid(4, 20, 1)

nextCell = grid.next_cell((0, 0))

print nextCell

# i = 0
# while True:
#     print i
#     pathArr[i%4].publish(msgArr[i%4])
#     time.sleep(2.0)
#     i+=1



print 'published'
sys.exit()