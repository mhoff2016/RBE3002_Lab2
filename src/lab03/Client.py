#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetPlan
from nav_msgs.msg import GridCells, OccupancyGrid
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point
from tf.transformations import euler_from_quaternion
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
        #self.subMap = rospy.Subscriber("/map", OccupancyGrid, self.mapCallback)

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
            # robot = Robot()
            # #take the output and send the path to NavToPose
            # robot.navToPose(resp1.plan)
            return resp1.plan
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

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




if __name__ == '__main__':
    client1 = Client()
    print("running client")
    while not rospy.is_shutdown():
        pass
