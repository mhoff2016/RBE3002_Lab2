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

class Client:
    def __init__(self):
        self.start = None
        self.end = None


        self.map = []
        self.width = None
        self.height = None
        self.resolution = None



        rospy.init_node('client')
        #sub to rviz, navPose
        self.subNav = rospy.Subscriber("/navPose", PoseStamped, self.endCallback)

        #sub to initial Pose
        self.subInitial = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.initialCallback)

        #SUB TO map
        #self.subMap = rospy.Subscriber("/map", OccupancyGrid, self.mapCallback)

    def a_star_client(self, start, goal):
        print "in_clinet"
        rospy.wait_for_service('a_star_path')
        print("after wait")
        startPose = PoseStamped()
        endPose = PoseStamped()
        message = GetPlan()
        startPose.pose = self.start
        endPose.pose = self.end
        message.start = startPose
        message.goal = endPose
        tolerance = .5
        try:
            print "in try"
            a_star_path = rospy.ServiceProxy('a_star_path', GetPlan)
            resp1 = a_star_path(startPose, endPose, tolerance )
            print "message sent"
            return resp1.path
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

#callback that sets appropriate fields in self.
    def endCallback(self, msg):
        #PoseStamped
        print "in end"
        self.end = msg.pose
        print self.end.position.x
        print self.end.position.y
        #print self.start.position.x
        if self.start is not None:
            #print "in if"
            self.a_star_client(self.start, self.end)
#callback that sets appropriate fields for start
    def initialCallback(self, msg):
        print "in inital"
        #PoseWithCovarianceStamped
        self.start = msg.pose.pose
        print self.start.position.x
        print self.start.position.y




if __name__ == '__main__':
    client1 = Client()
    print("running client")
    while not rospy.is_shutdown():
        pass
