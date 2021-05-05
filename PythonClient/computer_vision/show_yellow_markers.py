import setup_path 
import airsim
from airsim import Vector3r, Quaternionr, Pose
from airsim.utils import to_quaternion
import numpy as np
import time
import sys
import os

client = airsim.VehicleClient()
client.confirmConnection()

markerDurationSeconds = int(sys.argv[4])

client.simPlotPoints(points = [Vector3r(float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]))], color_rgba=[1.0, 1.0, 0.0, 1.0], size = 20, duration = markerDurationSeconds, is_persistent = False)

time.sleep(markerDurationSeconds)

client.simFlushPersistentMarkers() 