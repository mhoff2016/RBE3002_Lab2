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
        #pub to gridcells
        self.pubClosedX = rospy.Publisher("/closed_set", GridCells, queue_size=40)

        self.pubFrontier = rospy.Publisher("/frontier_set", GridCells, queue_size=10)

        self.pubPath = rospy.Publisher("/path_set", GridCells, queue_size=10)

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

    def endCallback(self, msg):
        #PoseStamped
        print "in end"
        self.end = msg.pose
        print self.end.position.x
        #print self.start.position.x
        if self.start is not None:
            #print "in if"
            self.a_star_client(self.start, self.end)

    def initialCallback(self, msg):
        print "in inital"
        #PoseWithCovarianceStamped
        self.start = msg.pose.pose
        print self.start.position.x

    '''def mapCallback(self, msg):
        print "in inital"
        self.map = msg.data
        self.width = msg.info.width
        self.height = msg.info.height
        self.resolution = msg.info.resolution
        print "all good"
        print self.resolution'''


    def publishClosed(self):
        out = GridCells()
        width = 37
        height = 37
        point1 = Point()
        point2 = Point()
        point1.x = 1
        point2.x = 5
        point1.y = 5
        point2.y = 5
        point1.z = 1
        point2.z = 1
        cells = tuple((point1, point2))
        out.cells = cells
        out.cell_width = .3
        out.cell_height = .3
        out.header.frame_id= "map"
        for i in range(0,40):
            #print out
            self.pubClosedX.publish(out)
        print "published"

    def world_to_map(x, y, my_map):
        """
            converts a point from the world to the map
            :param x: float of x position
            :param y: float of y position
            :return: tuple of converted point
        """

    def map_to_world(x, y, my_map):
        """
            converts a point from the map to the world
            :param x: float of x position
            :param y: float of y position
            :return: tuple of converted point
        """


if __name__ == '__main__':
    client1 = Client()
    rospy.sleep(1)
    client1.publishClosed()
    print("running client")
    while not rospy.is_shutdown():
        pass
