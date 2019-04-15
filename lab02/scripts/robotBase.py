#!/usr/bin/env python
import rospy
import copy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose, Point
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion
import math


class Robot:

    def __init__(self):
        pub  = rospy.Publisher('chatter', String, queue_size=10)
        # rospy.init_node('talker', anonymous=True)
        """"
        Set up the node here

        """
        self.start = None
        self.end = None

        navY = 0
        # initial pos
        self.inX = 0
        self.inY = 0
        # current yaw
        self.yaw = 0

        # Called "lab02"
        rospy.init_node('lab02')

        # subscribe to odom
        self.subOdom = rospy.Subscriber("/odom", Odometry, self.oDomCallback)

        # sub to rviz, navPose
        self.subNav = rospy.Subscriber("/navpos", PoseStamped, self.endCallback)

        # sub to initial Pose
        self.subNav = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.initialCallback)

        # pub to cmd_vel
        self.pubVel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    def endCallback(self, msg):
        # PoseStamped
        self.end = msg.pose
        print(self.end.position.x)
        # print self.start.position.x
        print(type(self.end))
        if self.start is not None:
            self.nav_to_pose(self.start, self.end)

    def initialCallback(self, msg):
        # PoseWithCovarianceStamped
        self.start = msg.pose.pose
        print(self.start.position.x)

    def nav_to_pose(self, start, goal):
        # type: (PoseStamped) -> None

        """This is a callback function. It should extract data from goal, drive in a straight line to reach
        the goal and then spin to match the goal orientation.
        :param
        goal: PoseStamped
        :return:"""
        print(type(goal))
        # get position and orientation from start and goal
        goalX = goal.position.x
        goalY = goal.position.y
        startX = start.position.x
        startY = start.position.y
        quatn = goal.orientation
        qn = [quatn.x, quatn.y, quatn.z, quatn.w]
        navX = goalX - startX
        navY = goalY - startY
        # convert into roll pitch and yaw in radians
        (self.rollN, self.pitchN, self.yawN) = euler_from_quaternion(qn)

        # calculate next position angle
        init_ang = math.atan(navY / navX)

        # rotate angle needed to get to next position
        self.rotate(init_ang)
        # Calculate distance necessary to drive forward (pyhtagoreum theorum)
        pythagLength = ((navX ** 2) + (navY ** 2)) ** (0.5)
        # drive straight the appropriate distance
        self.drive_straight(.22, pythagLength)

    def drive_straight(self, speed, distance):
        """
        Make the turtlebot drive shraight
        :type speed: float
        :type distance: float
        :param speed: speed to drive
        :param distance: distance to drive
        :return:
        """
        velocity = Twist()
        print(distance)
        # set speed in x
        velocity.linear.x = abs(speed)
        # set rest of directions to 0
        velocity.linear.y = 0
        velocity.linear.z = 0
        velocity.angular.x = 0
        velocity.angular.y = 0
        velocity.angular.z = 0
        # determine initial position
        initX = copy.deepcopy(self.inX)
        initY = copy.deepcopy(self.inY)
        # determine initial distance
        init_pos = ((self.inX - initX) ** 2 + (self.inY - initY) ** 2) ** (0.5)
        current_pos = ((self.inX - initX) ** 2 + (self.inY - initY) ** 2) ** (0.5)
        while current_pos <= (init_pos + distance):
            # send velocity
            self.pubVel.publish(velocity)
            # reset current position
            current_pos = ((self.inX - initX) ** 2 + (self.inY - initY) ** 2) ** (0.5)
        # stop robot
        velocity.linear.x = 0
        self.pubVel.publish(velocity)

    def rotate(self, angle):
        """
        Rotate in place
        :param angle: angle to rotate
        :param angle: angle to rotate (assuming angle in degrees)
        :return:
        """
        print("yaw:", self.yaw)
        goal = self.yaw - angle
        if goal > math.pi:
            goal = -(2 * math.pi - goal)

        # To begin rotating

        turn = true  # start turning
        while turn and not rospy.is_shutdown():

            current = self.yaw
            print("Current Pos: " + str(current))
            print("End Orient: " + str(goal))
            difference = abs((current - goal))
            # print(diff)
            if .05 < difference < -.05:
                self.rotateWheels(0)
                turn = false
            else:
                if goal < 0:
                    self.rotateWheels(-.22)
                else:
                    self.rotateWheels(.22)

    def rotateWheels(self, angSpeed):
        vel_msg = Twist()
        vel_msg.linear.x = 0
        # Since we are moving just in x-axis
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        # set angular velocity
        vel_msg.angular.z = angSpeed
        self.pubVel.publish(vel_msg)
        rospy.sleep(.0001)

    def oDomCallback(self, msg):
        """
        update the state of the robot
        :type msg: Odom
        :return:
        """
        # set variables from position
        self.inX = msg.pose.pose.position.x
        self.inY = msg.pose.pose.position.y
        quat = msg.pose.pose.orientation
        q = [quat.x, quat.y, quat.z, quat.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(q)


if __name__ == '__main__':
    print("running lab02")
    rob = Robot()
    while not rospy.is_shutdown():
        pass
