#!/usr/bin/python3
import numpy as np
import cv2
import glob
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from std_msgs.msg import UInt64
from nav_msgs.msg import Odometry

onDetectedPublisher = None

g_worldTick = 0
g_lastPositionStr = '0;0;0'
g_lastDetectionWorldTick = 1
g_latestImage = None

def checkAndNotifyOnBlobDetected(img):

    global g_worldTick
    global g_lastPositionStr
    global g_lastDetectionWorldTick

    if (g_lastDetectionWorldTick >= g_worldTick):
        return

    # img = cv2.imread('/home/eyal/Projects/AirSim/ros/444.png')

    # Blur image to remove noise
    frame=cv2.GaussianBlur(img, (3, 3), 0)

    # Switch image from BGR colorspace to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # cv2.imwrite('aa.png', hsv)

    purpleMin = np.array([30,60,210])
    purpleMax = np.array([40,170,240])

    # Sets pixels to white if in purple range, else will be set to black
    mask = cv2.inRange(hsv, purpleMin, purpleMax)

    whitePixelsCount = np.sum(mask == 255)

    if (whitePixelsCount < 100):
        return False

    # print(whitePixelsCount)


    params = cv2.SimpleBlobDetector_Params()

    # Change thresholds
    params.minThreshold = 0
    params.maxThreshold = 256

    params.filterByArea = False
    # params.minArea = 100

    params.filterByCircularity = False
    params.filterByConvexity = False
    params.filterByInertia =False
    params.filterByColor = False


    # Create a detector with the parameters
    # detector = cv2.SimpleBlobDetector(params)
    detector = cv2.SimpleBlobDetector_create(params)


    # Detect blobs.
    # cv2.imshow('',mask)
    # cv2.waitKey(0)

    keypoints = detector.detect(mask)


    # print(keypoints)

    # print(len(keypoints))
    # print(keypoints[0].size)
    if (len(keypoints) == 1 and keypoints[0].size > 9 and whitePixelsCount > 1000):
        
        # print(g_lastPositionStr)
        onDetectedPublisher.publish('' + g_lastPositionStr + ';' + str(keypoints[0].size))

        g_lastDetectionWorldTick = g_worldTick

    # Draw detected blobs as red circles.
    # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures
    # the size of the circle corresponds to the size of blob

    # im_with_keypoints = cv2.drawKeypoints(img, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    # Show blobs
    # cv2.imshow("Keypoints", im_with_keypoints)
    # cv2.waitKey(0)




# Instantiate CvBridge
bridge = CvBridge()

def onWorldTick(msg):

    global g_worldTick 
    global g_latestImage

    g_worldTick = msg.data

    checkAndNotifyOnBlobDetected(g_latestImage)

    # print(g_worldTick)

def imageCallback(msg):

    global g_latestImage

    try:
        # Convert your ROS Image message to OpenCV2
        cv2Img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)
    else:


        # cv2.imwrite('camera_image.jpeg', cv2Img)
        
        # if (isImageContainsBlob(cv2Img)):
            # print('Found!')
        # checkAndNotifyOnBlobDetected(cv2Img)
        g_latestImage = cv2Img


def odometryCallback(data):

	global g_lastPositionStr

	X_pos = round(data.pose.pose.position.x, 2)
	Y_pos = round(data.pose.pose.position.y, 2)
	Z_pos = round(data.pose.pose.position.z, 2)

	g_lastPositionStr = str(X_pos) + ';' + str(Y_pos) + ';' + str(Z_pos)


if __name__ == '__main__':

    rospy.init_node('blob_detector_node', anonymous=False)

    rospy.Subscriber('MainCamera/Scene', Image, imageCallback)
    
    rospy.Subscriber('/world_tick', UInt64, onWorldTick)

    rospy.Subscriber('odom_local_ned', Odometry, odometryCallback)


    onDetectedPublisher = rospy.Publisher('/detected_blobs', String, queue_size=10)
    # onDetectedPublisher = rospy.Publisher('detected_blobs', String, queue_size=10)
    
    # r = rospy.Rate(10) # 10hz

    # while not rospy.is_shutdown():
    #     pub.publish("hello world")
    #     r.sleep()

    rospy.spin()