#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion
import math



class Robot:

    def __init__(self):
        """"
        Set up the node here
        set up publisher and subscriber
        subscribe to the topic that rviz pubs to
        publish to the topic that turtle subs to
        create variable that can store the data that comes in from the messages
        """
        self.start = None
        self.end = None

        #target pos
        navX = 0#.py", line 57, in initialCallback
        #print self.start.position.x

        navY = 0
        #initial pos
        self.inX = 0
        self.inY = 0
        #current yaw
        self.yaw = 0

        #Called "lab02"
        rospy.init_node('lab02')

        #subscribe to odom
        self.subOdom = rospy.Subscriber("/odom", Odometry, self.oDomCallback)

        #sub to rviz, navPose
        self.subNav = rospy.Subscriber("/navPose", PoseStamped, self.endCallback)

        #sub to initial Pose
        self.subNav = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.initialCallback)

        #pub to cmd_vel
        self.pubVel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def endCallback(self, msg):
        #PoseStamped
        self.end = msg.pose
        print self.end.position.x
        #print self.start.position.x
        if self.start is not None:
            #print "in if"
            self.a_star_client(self.start, self.end)

    def initialCallback(self, msg):
        #PoseWithCovarianceStamped
        self.start = msg.pose.pose
        print self.start.position.x

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

    def nav_to_pose(self, msg):
        # type: (PoseStamped) -> None
        """
        This is a callback function. It should extract data from goal, drive in a striaght line to reach the goal and
        then spin to match the goal orientation.
        :param goal: PoseStamped
        :return:
        """
        #get desired position and orientation
        navX = msg.pose.position.x
        navY = msg.pose.position.y
        quatn = msg.pose.orientation
        qn = [quatn.x, quatn.y, quatn.z, quatn.w]

        #convert into roll pitch and yaw in radians
        (self.rollN, self.pitchN, self.yawN) = euler_from_quaternion(qn)

        #calculate angle needed to get to next position
        initialAngle = math.atan((navY - self.inY)/(navX - self.inX))

        #rotate angle needed to get to next position
        self.rotate(initialAngle)
        #Calculate distance necessary to drive forward (pyhtagoreum theorum)
        pythagLength = ((navX**2)+(navY ** 2))** (0.5)
        #drive straigh the appropriate distance
        self.drive_straight(.22, pythagLength)
        #rotate to match desired pose
        self.rotate(self.yawN)

    def drive_straight(self, speed, distance):
        """
        Make the turtlebot drive shraight
        :type speed: float
        :type distance: float
        :param speed: speed to drive
        :param distance: distance to drive
        :return:
        """
        #make type twist
        vel_msg = Twist()

        # Twist is a datatype for velocity. set speed for x direction
        vel_msg.linear.x = abs(speed)
        #Since we are moving just in x-axis
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        #determine initial x and y positions
        initialPosX = self.inX
        initialPosY = self.inY
        #determine inititla distance from start for initial and current
        initialPos = ((self.inX-initialPosX)**2+(self.inY - initialPosY)**2) **(0.5)
        currentPos = ((self.inX-initialPosX)**2+(self.inY - initialPosY)**2) **(0.5)

        #Compare currrent position to initial position
        while currentPos <= initialPos + distance:
	        # publish the velocity
            self.pubVel.publish(vel_msg)
            #reset current position
            currentPos = ((self.inX-initialPosX)**2+(self.inY - initialPosY)**2) **(0.5)
        #stop bot when done
        vel_msg.linear.x = 0
        self.pubVel.publish(vel_msg)


    def rotate(self, angle):
        """
        Rotate in place
        :param angle: angle to rotate
        :return:
        """
        angSpeed = .262 #radian equivalent of 15 degrees
        radAngle = angle
        #not needed
        seconds = float(radAngle)/angSpeed

        vel_msg = Twist()
        r = rospy.Rate(1); #not needed
        vel_msg.linear.x = 0
        #Since we are moving just in x-axis
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        #set angular velocity
        vel_msg.angular.z = angSpeed

        initialYaw = self.yaw
        currentYaw = self.yaw
        while  currentYaw <= (initialYaw + angle):
	        # publish the velocity
            self.pubVel.publish(vel_msg)
	        # wait for 0.1 seconds (10 HZ) and publish again
            currentYaw = self.yaw
        #stop bot when done
        vel_msg.angular.z = 0
        self.pubVel.publish(vel_msg)


    def oDomCallback(self, msg):
        #extract position and yaw and set variables
        self.inX = msg.pose.pose.position.x
        self.inY = msg.pose.pose.position.y
        quat = msg.pose.pose.orientation
        q = [quat.x, quat.y, quat.z, quat.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(q)




if __name__ == '__main__':
    print("running lab2")
    rob = Robot()
    #rob.drive_straight(1, 2)
    #rob.rotate(3*math.pi/2)
    #rob.rotate(math.pi/2)
    while not rospy.is_shutdown():
        pass
