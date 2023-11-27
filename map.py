#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
import csv
import numpy as np

def map_callback(data):
    # map_data = data.data
    map_info = data.info
    map_array = np.array(data.data, dtype=np.int8).reshape(384, 384)


def map_mmd_callback(data):
    print(data)

if __name__ == '__main__':
    rospy.init_node('map_subscriber')

    rospy.Subscriber('/map', OccupancyGrid, map_callback)
    rospy.Subscriber('/map_metadata', MapMetaData, map_mmd_callback)
    rospy.spin()
