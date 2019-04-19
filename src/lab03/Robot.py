#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion
import math
import copy



class Robot:
    ## TODO: use acml to have the robot snap to the position set with 2d

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
        #initial pos3. Find interesting areas to expand the map
        self.inX = 0
        self.inY = 0
        #current yaw
        self.yaw = 0

        #Called "lab02"
        #rospy.init_node('lab02')

        #subscribe to odom
        self.subOdom = rospy.Subscriber("/odom", Odometry, self.oDomCallback)

        #sub to rviz, navPose
        #self.subNav = rospy.Subscriber("/navPose", PoseStamped, self.endCallback)

        #sub to initial Pose
        #self.subNav = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.initialCallback)

        #pub to cmd_vel
        self.pubVel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)


    # def nav_to_pose(self, msg):
    #     # type: (PoseStamped) -> None
    #     """
    #     This is a callback function. It should extract data from goal, drive in a striaght line to reach the goal and
    #     then spin to match the goal orientation.
    #     :param goal: PoseStamped
    #     :return:
    #     """
    #     #get desired position and orientation
    #     navX = msg.pose.position.x
    #     navY = msg.pose.position.y
    #     quatn = msg.pose.orientation
    #     qn = [quatn.x, quatn.y, quatn.z, quatn.w]
    #
    #     #convert into roll pitch and yaw in radians
    #     (self.rollN, self.pitchN, self.yawN) = euler_from_quaternion(qn)
    #
    #     #calculate angle needed to get to next position
    #     initialAngle = math.atan((navY - self.inY)/(navX - self.inX))
    #
    #     #rotate angle needed to get to next position
    #     self.rotate(initialAngle)
    #     #Calculate distance necessary to drive forward (pyhtagoreum theorum)
    #     pythagLength = ((navX**2)+(navY ** 2))** (0.5)
    #     #drive straigh the appropriate distance
    #     self.drive_straight(.22, pythagLength)
    #     #rotate to match desired pose
    #     self.rotate(self.yawN)

    def nav_to_pose(self, distlist):
        # type: (PoseStamped) -> None
        """
        This is a callback function. It should extract data from goal, drive in a straight line to reach the goal and
        then spin to match the goal orientation.
        :param goal: PoseStamped
        :return:
        """
        i = 0 #start
        j = 1 #goal
        print "type:", type(distlist)
        while j < len(distlist):
            # get desired position and orientation
            goalX = distlist[j].pose.position.x
            goalY = distlist[j].pose.position.y
            startX = distlist[i].pose.position.x
            startY = distlist[i].pose.position.y
            quatn = distlist[j].pose.orientation
            qn = [quatn.x, quatn.y, quatn.z, quatn.w]
            navX = goalX - startX
            navY = goalY - startY
            i += 1
            j += 1
            # convert into roll pitch and yaw in radians
            (self.rollN, self.pitchN, self.yawN) = euler_from_quaternion(qn)
            print "yawN", self.yawN

            # calculate angle needed to get to next position
            if goalX == startX:
                initialAngle = 0
            else:
                initialAngle = math.atan2(navY, navX)
                print "initialAngle", initialAngle

            # rotate angle needed to get to next position
            self.rotate(initialAngle)
            # Calculate distance necessary to drive forward (pyhtagoreum theorum)
            pythagLength = ((navX ** 2) + (navY ** 2)) ** (0.5)
            # drive straigh the appropriate distance
            self.drive_straight(.22, pythagLength)
            rospy.sleep(1)

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


    # def rotate(self, angle):
    #     """
    #     Rotate in place
    #     :param angle: angle to rotate
    #     :return:
    #     """
    #     angSpeed = .262 #radian equivalent of 15 degrees
    #     radAngle = angle
    #     #not needed
    #     seconds = float(radAngle)/angSpeed
    #
    #     vel_msg = Twist()
    #     vel_msg.linear.x = 0
    #     #Since we are moving just in x-axis
    #     vel_msg.linear.y = 0
    #     vel_msg.linear.z = 0
    #     vel_msg.angular.x = 0
    #     vel_msg.angular.y = 0
    #     #set angular velocity
    #     vel_msg.angular.z = angSpeed
    #
    #     initialYaw = copy.deepcopy(self.yaw)
    #     currentYaw = self.yaw
    #     while  currentYaw <= (initialYaw + angle):
	#         # publish the velocity
    #         self.pubVel.publish(vel_msg)
	#         # wait for 0.1 seconds (10 HZ) and publish again
    #         currentYaw = self.yaw
    #     #stop bot when done
    #     vel_msg.angular.z = 0
    #     self.pubVel.publish(vel_msg)


    # def rotate(self, angle):
    #     print "in Rotate"
    #        # Rotate in place
    #        # :param angle: angle to rotate
    #        # :return:
    #     #make type twist
    #     vel_msg = Twist()
    #
    # 	startYaw = self.yaw
    #
    # 	while (angle > (((startYaw-self.yaw)**2)**.5)):	#threshold of arrived at the angle or not
    #
    # 		vel_msg.linear.x = 0
    # 		vel_msg.linear.y = 0
    # 		vel_msg.linear.z = 0
    #
    # 		vel_msg.angular.x = 0
    # 		vel_msg.angular.y = 0
    # 		vel_msg.angular.z = .1
    #
    # 		self.pubVel.publish(vel_msg)      #publishes the linear and angular speeds that were set in vel_msg
    #
    # 		#self.rate.sleep()
    #
    # 	vel_msg.angular.z = 0         #sets the robot to stop spinning
    #
    # 	self.pubVel.publish(vel_msg)  #publish the stop message


    def rotate(self, angle):
        """
        Rotate in place
        :param angle: angle to rotate (assuming angle in degrees)
        :return:
        """
        print "yaw:", self.yaw
        goal = self.yaw - angle
        # if goal > (math.pi):
        #     goal = -(2*math.pi - goal)


        #To begin rotating

        flag = False #means rotation not complete
        while(not flag and not rospy.is_shutdown()):

            current = self.yaw
            #print("Current Pos: " + str(current))
            #print("End Orient: " + str(goal))
            diff = abs((current - goal))
            #print(diff)
            if ((diff < .05 and diff > - .05)):
                self.rotateWheels(0)
                flag = True
            else:
                if (goal < 0):
                    self.rotateWheels(-.22)
                else:
                    self.rotateWheels(.22)

    def rotateWheels(self, angSpeed):
        vel_msg = Twist()
        vel_msg.linear.x = 0
        #Since we are moving just in x-axis
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        #set angular velocity
        vel_msg.angular.z = angSpeed
        self.pubVel.publish(vel_msg)
        rospy.sleep(.0001)



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
    rospy.sleep(1)
    rob.rotate(math.pi/2)
    #rob.drive_straight(1, 2)
    #rob.rotate(3*math.pi/2)
    #rob.rotate(math.pi/2)
    while not rospy.is_shutdown():
        pass
