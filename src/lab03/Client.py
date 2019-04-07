#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion
import math

class Client:
    def __init__(self):
        self.start = None
        self.end = None
        rospy.init_node('client')
        #sub to rviz, navPose
        self.subNav = rospy.Subscriber("/navPose", PoseStamped, self.endCallback)

        #sub to initial Pose
        self.subNav = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.initialCallback)

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





if __name__ == '__main__':
    client = Client()
    print("running client")
    while not rospy.is_shutdown():
        pass
