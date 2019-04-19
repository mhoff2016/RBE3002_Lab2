#!/usr/bin/env python
import rospy
import sys
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point
from PriorityQueue import *
import math
import copy
from Robot import *

class Expand_Map:

    def __init__(self):
        """
        Use this node to expand the map to ensure that the turtlebot will not enter 
        a space too small for it to enter.
        """
        rospy.init_node("expand_map")
        # stores data coming in
        self.map = []
        self.width = None
        self.height = None
        self.resolution = None

        # stores data realte to paths
        self.path = []
        self.cost = []
        self.mapPose = []

        # pub to gridcells
        self.pubClosedX = rospy.Publisher("/closed_set", GridCells, queue_size=10)

        self.pubFrontier = rospy.Publisher("/frontier_set", GridCells, queue_size=10)

        # self.pubPath = rospy.Publisher("/path_set", GridCells, queue_size=10)
        # publishes to path
        self.pubPath = rospy.Publisher("/path", Path, queue_size=10)
        # SUB TO map
        self.subMap = rospy.Subscriber("/map", OccupancyGrid, self.mapCallback)
   

    def map_callback(self, msg):
        """
            This is a callback for the /map topic
            :param msg: map
            :return: None
        """
        expanded_map = self.expand(msg)
        occo_map = OccupancyGrid()
        occo_map.header = msg.header
        occo_map.data = expanded_map
        occo_map.info = msg.info
        self.expanded_map = occo_map
        self.map_pub.publish(occo_map)

    def handle_map(self, req):

        """
            Service call to get map and expand it
            :return:
        """



    def expand(self,my_map):
        """
            Expand the map and return it
            :param my_map: map
            :return: map
        """
        map_width = my_map.info.width

        for x in my_map:
            if my_map[x] == 1:
                break
            else:
                if my_map[x + 1] == 1 or my_map[x - 1] == 1:
                    my_map[x] = 1
                if my_map[x-map_width] == 1 or my_map[x+map_width] == 1:
                    my_map[x] = 1
                if my_map[(x+1) - map_width] or my_map[(x+1) + map_width] or\
                        my_map[(x-1) + map_width] or my_map[(x-1) - map_width]:
                    my_map[x] = 1
        return my_map

      

if __name__ == '__main__':

    Expand_Map()
