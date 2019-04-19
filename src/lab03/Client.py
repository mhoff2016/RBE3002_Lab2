#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetPlan
from nav_msgs.msg import GridCells, OccupancyGrid
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point
from tf.transformations import euler_from_quaternion
from sklearn.cluster import KMeans
import numpy as np
import math
from Robot import *

##fix this
class Client:
    def __init__(self):
        #store start and end
        self.start = None
        self.end = None
        #Store some map properties
        self.map = []
        self.width = None
        self.height = None
        self.resolution = None
        #name client
        rospy.init_node('client')
        #sub to rviz, navPose
        self.subNav = rospy.Subscriber("/navPose", PoseStamped, self.endCallback)

        #sub to initial Pose
        self.subInitial = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.initialCallback)

        #SUB TO map
        self.subMap = rospy.Subscriber("/map", OccupancyGrid, self.mapCallback)

    def a_star_client(self, start, goal):
        print "in_client"
        rospy.wait_for_service('a_star_path')
        startPose = PoseStamped()
        endPose = PoseStamped()
        message = GetPlan()
        startPose.pose = self.start
        endPose.pose = self.end
        message.start = startPose
        message.goal = endPose
        tolerance = .5
        try:
            a_star_path = rospy.ServiceProxy('a_star_path', GetPlan)
            resp1 = a_star_path(startPose, endPose, tolerance )
            print "swoosh (ie message sent)"
            #Create new Robot  for later
            robot = Robot()
            rospy.sleep(1)
            # #take the output and send the path to NavToPose
            # robot.drive_straight(.5, .5)
            # robot.rotate(-math.pi/2)
            # robot.drive_straight(.5, .5)
            robot.nav_to_pose(resp1.plan.poses)
            return resp1.plan
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def mapCallback(self, msg):
        #store appropriate data
        self.map = msg.data
        self.width = msg.info.width
        self.height = msg.info.height
        self.resolution = msg.info.resolution
        twoMap = np.reshape(self.map, (-1, self.width))
        print twoMap
        #print self.resolution

#callback that sets appropriate fields in self.
    def endCallback(self, msg):
        #PoseStamped
        self.end = msg.pose
        #verify position is correct
        print"goal x:", self.end.position.x
        print "goal y:", self.end.position.y
        #make sure we have a start position before proceeding
        if self.start is not None:
            #Send to Service
            self.a_star_client(self.start, self.end)

#callback that sets appropriate fields for start
    def initialCallback(self, msg):
        #PoseWithCovarianceStamped
        self.start = msg.pose.pose
        print "start x:",self.start.position.x
        print "start y:", self.start.position.y



#       [0 1 0 0 0]
#       [0 1 0 0 0]
#       [0 0 0 0 0]
#       [0 0 0 1 0]
#       [0 0 0 1 0]
#top to bottom
testMap = [0, 100, 0, 0, 0, 0, 100, 0, 0 ,0, 0, 0, 0, 0, 0, 0, 0, 0, 100, 0, 0 ,0, 0, 100, 0]


if __name__ == '__main__':
    client1 = Client()
    B = np.reshape(testMap, (-1, 5))
    print B
    print("running client")
    while not rospy.is_shutdown():
        pass
