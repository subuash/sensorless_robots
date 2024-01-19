#!/usr/bin/env python

#Ashwin Subramanian, subraash@oregonstate.edu
#Generate a CSV file representing the map / obstacles as 1s and empty space as 0s

import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
import csv
import numpy as np
import pandas as pd

class Map():

    def __init__(self):
        self.width = 0
        self.height = 0

    def map_callback(self, data):

        data_path = 'data/corridor/'

        # map_data = data.data
        map_info = data.info
        map_array = np.array(data.data, dtype=np.int8).reshape(self.width, self.height)

        df = pd.DataFrame(map_array)
        df.to_csv(data_path + 'map_data.csv', index=False)

        print("done", self.width, self.height)

    def map_mmd_callback(self, data):
        self.width = int(data.__getattribute__('width'))
        self.height = int(data.__getattribute__('height'))
        print(data)

if __name__ == '__main__':
    rospy.init_node('map_subscriber')
    m = Map()

    rospy.Subscriber('/map_metadata', MapMetaData, m.map_mmd_callback)
    rospy.Subscriber('/map', OccupancyGrid, m.map_callback)
    rospy.spin()
