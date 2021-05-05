import numpy as np
import cv2
import glob
import rospy
import math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from std_msgs.msg import UInt64
from nav_msgs.msg import Odometry


g_worldTick = 0

spawnedMarkers = {}

detectionsLog = {}

def dist(x1, y1, x2, y2):
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def onBlobDetected(msg):

    global g_worldTick

    blobData = msg.data.split(';')

    detectorX = float(blobData[0])
    detectorY = float(blobData[1])

    minDist = 999999
    nearestMarkerId = -1

    print(len(spawnedMarkers))

    for markerId, markerData in list(spawnedMarkers.items()):

        if (int(markerData[4]) < g_worldTick):            
            del spawnedMarkers[markerId]

            # Example:
            # if (len(set(detectionsLog[markerId])) < X):
            #     ledsPoints += 1

            continue
            
        # print('markerId: ' + str(markerId))
        # print('markerId: ' + str(markerData[0]) + ';' + str(markerData[0]))

        currentMrakerX = float(markerData[0])
        currentMrakerY = float(markerData[1])

        distance = dist(detectorX, detectorY, currentMrakerX, currentMrakerY)

        # print('distance to ' + str(markerId) + ' is ' + str(distance))

        if (minDist > distance):
            nearestMarkerId = markerId
            minDist = distance

    print('Detected marker ' + str(nearestMarkerId))

    if (-1 != nearestMarkerId):
        detectionsLog[nearestMarkerId].append(g_worldTick)

        # Detections count:
        # Here you can decide whether drones get the point
        print(len(set(detectionsLog[nearestMarkerId])))

        # Example:
        # if (len(set(detectionsLog[nearestMarkerId])) >= X):
        #     dronesPoints += 1


def onMarkerSpawned(msg):

    global spawnedMarkers

    markerData = msg.data.split(';')
    # print(markerData)

    markerId = markerData[0]
    markerX = markerData[1]
    markerY = markerData[2]
    markerZ = markerData[3]
    markerStartTick = markerData[4]
    markerEndTick = markerData[5]

    # print(markerEndTick)

    spawnedMarkers[markerId] = [markerX, markerY, markerZ, markerStartTick, markerEndTick]

    detectionsLog[markerId] = []

def onWorldTick(msg):

    global g_worldTick 

    g_worldTick = msg.data

    # print(g_worldTick)




if __name__ == '__main__':

    rospy.init_node('patrol_referee_node', anonymous=False)

    rospy.Subscriber('/world_tick', UInt64, onWorldTick)
    from nav_msgs.msg import Odometry

    # rostopic echo /marker_spawned
    rospy.Subscriber('/marker_spawned', String, onMarkerSpawned)

    # rostopic echo /detected_blobs
    rospy.Subscriber('/detected_blobs', String, onBlobDetected)



    rospy.spin()