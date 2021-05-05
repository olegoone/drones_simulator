#!/usr/bin/python3
import setup_path 
import airsim
from airsim import Vector3r, Quaternionr, Pose
from airsim.utils import to_quaternion
import numpy as np
import time
import sys
import os
import csv

import rospy
from std_msgs.msg import String
from std_msgs.msg import UInt64

g_worldTick = 0

onSpawnMarkerPublisher = None


client = airsim.VehicleClient()
client.confirmConnection()

markersParams = {}
spawnedMarkersId = []

def loadMarkers():

    global client

    allMarkerDurationSeconds = 0

    with open('/home/eyal/Projects/AirSim/ros/src/drones_controller/config/markers_positions.csv', 'r') as csvfile:

        csv_reader = csv.reader(csvfile, delimiter=',', quotechar='|')

        next(csv_reader)

        for row in csv_reader:

            markerId = row[0]
            
            markerX = row[1]
            markerY = row[2]
            markerZ = row[3]

            markerSize = int(row[4])

            markerRevealTick = int(row[5])
            markerDurationTicks = int(row[6])

            markerColorRed = float(row[7])
            markerColorGreen = float(row[8])
            markerColorBlue = float(row[9])
            markerColorOpacity = float(row[10])

            markersParams[markerId] = [ [markerX, markerY, markerZ], [markerColorRed, markerColorGreen, markerColorBlue, markerColorOpacity], markerSize, markerDurationTicks, markerRevealTick ]


    # time.sleep(allMarkerDurationSeconds)


def onWorldTick(msg):

    global g_worldTick 
    global client

    g_worldTick = msg.data

    # print(g_worldTick)


    for markerId in markersParams:

        markerParam = markersParams[markerId]

        if (g_worldTick < markerParam[4]):
            continue

        if (markerId in spawnedMarkersId):
            continue
        
        markerX = markerParam[0][0]
        markerY = markerParam[0][1]
        markerZ = markerParam[0][2]

        client.simPlotPoints(points = [Vector3r(float(markerX), float(markerY), float(markerZ))], color_rgba = markerParam[1], size = markerParam[2], duration = markerParam[3], is_persistent = False)


        onSpawnMarkerPublisher.publish('' + str(markerId) + ';' + str(markerX) + ';' + str(markerY) + ';' + str(markerZ) + ';' + str(markerParam[4]) + ';' + str(int(markerParam[4]) + int(markerParam[3])))
        
        # Mark as fired
        # markerParam[4] = 2147483647
        spawnedMarkersId.append(markerId)


if __name__ == '__main__':

    rospy.init_node('marker_spawner_node', anonymous=False)

    loadMarkers()
    
    rospy.Subscriber('/world_tick', UInt64, onWorldTick)

    onSpawnMarkerPublisher = rospy.Publisher('/marker_spawned', String, queue_size=10)

    rospy.spin()

    client.simFlushPersistentMarkers()