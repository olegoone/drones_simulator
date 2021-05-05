#!/usr/bin/env python

import sys
import time

import rospy
from std_msgs.msg import UInt64

rospy.init_node('ticker_node')

pub = rospy.Publisher('/world_tick', UInt64, queue_size=1)

r = rospy.Rate(1)

currentTick = 0

while not rospy.is_shutdown():

    pub.publish(currentTick)

    print('tick: ' + str(currentTick))

    r.sleep()

    currentTick += 1
