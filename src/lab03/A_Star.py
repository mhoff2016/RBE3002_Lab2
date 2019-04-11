#!/usr/bin/env python
import rospy
import sys
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from nav_msgs.srv import GetPlan, GetPlanResponse
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point
from PriorityQueue import *
import math
import copy
from Robot import *


class A_Star:

    def __init__(self):

        """
            This node handle A star paths requests.
            It is accessed using a service call. It can the publish grid cells
            to show the frontier,closed and path.
        """
        rospy.init_node("a_star")  # start node
        #stores data coming in
        self.map = []
        self.width = None
        self.height = None
        self.resolution = None
        #stores data related to paths
        self.path = []
        self.cost = []
        self.mapPose = []
        #pub to gridcells
        self.pubClosedX = rospy.Publisher("/closed_set", GridCells, queue_size=10)
        #pub to frontier
        self.pubFrontier = rospy.Publisher("/frontier_set", GridCells, queue_size=10)
        #pub to path gridcell
        #self.pubPath = rospy.Publisher("/path_set", GridCells, queue_size=10)
        #publishes to path
        self.pubPath = rospy.Publisher("/path", Path, queue_size=10)
        #SUB TO map
        self.subMap = rospy.Subscriber("/map", OccupancyGrid, self.mapCallback)

        #Publishes to closed set
    def publishClosed(self, cells):
            #converts data to message type gridcell and Publishes
            out = GridCells()
            width = self.width
            height = self.height
            out.cells= cells
            out.cell_width = self.resolution
            out.cell_height = self.resolution
            out.header.frame_id= "map"
            for i in range(0,2):
                #print out
                self.pubClosedX.publish(out)
            #print "published closed"

    #paints to frontier
    def publishFrontier(self, cells):
        #converts data to message type gridcell and Publishes
            out = GridCells()
            width = self.width
            height = self.height
            #cells = tuple((point1))
            out.cells= cells
            out.cell_width = self.resolution
            out.cell_height = self.resolution
            out.header.frame_id= "map"
            for i in range(0,2):
                #print out
                self.pubFrontier.publish(out)
            #print "published"

    #paints to path
    def publishPath(self, cells):
        #initializes path object
            out = Path()
            #convert list of tuples to list of posestampeds
            pointCells = self.convertToPose(cells)
            #Fills path with required params
            out.poses= pointCells
            self.mapPose = pointCells
            out.header.frame_id= "map"
            #publishes twice just to be safe
            for i in range(0,2):
                self.pubPath.publish(out)


    def handle_a_star(self, req):

        """
            service call that uses A* to create a path.
            This can be altered to also grab the orentation for better heuristic
            :param req: GetPlan
            :return: Path()
        """
        print ("Returning path...")
        return  self.a_star(req.start, req.goal)

    #spins up
    def a_star_server(self):
        #rospy.init_node('a_star_path_server')
        s =  rospy.Service('a_star_path', GetPlan, self.handle_a_star)
        print "ready to star that A"
        rospy.spin()

    #convert list of tuples to list of posestampeds
    def convertToPose(self, cells):
        pointCells = []
        width = self.width
        height = self.height
        for cell in cells:
            celly = self.worldToMap(cell)
            posey = PoseStamped()
            posey.pose.position.x = celly[0]
            posey.pose.position.y  = celly[1]
            somePoint2 = copy.deepcopy(posey)
            pointCells.append(somePoint2)
        return pointCells

    def mapCallback(self, msg):
        #store appropriate data
        self.map = msg.data
        self.width = msg.info.width
        self.height = msg.info.height
        self.resolution = msg.info.resolution
        #print self.resolution


    def heuristic(a, b):
        #not being used but pretty straightforward
        (x1, y1) = a
        (x2, y2) = b
        return abs(x1 - x2) + abs(y1 - y2)

    def euclidean_heuristic(self, point1, point2):
        """
            calculate the dist between two points
            :param point1: tuple of location
            :param point2: tuple of location
            :return: dist between two points
        """
        x1 = point1[0]
        y1 = point1[1]

        x2 = point2[0]
        y2 = point2[1]

        out =  math.sqrt((x1 - x2)**2 + (y1 - y2)**2)
        return out


    def a_star(self, startIn, goalIn):
        """
            A*
            This is where the A* algorithum belongs
            :param start: tuple of start pose
            :param goal: tuple of goal pose
            :return: dict of tuples
        """
        #extracts x and y values
        startPoseX =  startIn.pose.position.x
        startPoseY =  startIn.pose.position.y
        # creates tuple of x and y values
        startPoint = (startPoseX, startPoseY)
        print "actual start: ", startPoint
        start = self.mapToWorld(startPoint)
        print "start:", start
        #extracts x and y values
        goalPoseX = goalIn.pose.position.x
        goalPoseY = goalIn.pose.position.y
        goalPoint = (goalPoseX, goalPoseY)
        print "actual goal: ", goalPoint
        #converts to correct units
        goal = self.mapToWorld(goalPoint)
        print "goal: ", goal
        #initializes lists
        frontierSet = []
        closedSet = []
        pathSet = ()
        #initializes temporay point
        tempPoint = Point()

        #A space to separate the tedious stuff and the actual algorithm
        #INIIALIZES THE PRIORITY QUEUE
        frontier = PriorityQueue()
        frontier.put(start, 0)
        #initializes dicts
        came_from = {}
        cost_so_far = {}
        came_from[start] = None
        cost_so_far[start] = 0
        #print "In A Star"
        while not frontier.empty():
            #picks next in priority queue
            current = frontier.get()
            #creates a copy and adds it to a list
            temp = self.worldToMap(current)
            tempPoint.x = temp[0]
            tempPoint.y = temp[1]
            frontierSet.append(tempPoint)
            self.publishFrontier(frontierSet)

            #stops if current node is the goal
            if current == goal:
                break
            #search through each of the neighbors and determine costs
            for next in self.neighbors(current):
                #creates a deeeeeepcopy and adds it to a list
                temp = self.worldToMap(next)
                tempPoint.x = temp[0]
                tempPoint.y = temp[1]
                tempPoint2 = copy.deepcopy(tempPoint)
                closedSet.append(tempPoint2)
                self.publishClosed(closedSet)
                #previous cost + current cost
                new_cost = cost_so_far[current] + self.euclidean_heuristic(current, next) ##Should tthis just be euclidean???
                #adds to priority queue or updates with lower costs
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + self.euclidean_heuristic(goal, next)
                    frontier.put(next, priority)#adds to priority queue
                    came_from[next] = current
                    #rospy.sleep(2) for testing purposes
        #Set path equal to came from
        self.path =  came_from
        #print "came_from:", came_from
        self.cost =  cost_so_far
        resPath = self.reconstruct_path(start, goal, came_from)
        #optimum path
        self.publishPath(resPath)
        #print "path:", resPath
        output = self.convertToPose(resPath)
        somePath = Path()
        somePath.poses = output
        #initialize get plan rsponse
        zaPlan = GetPlanResponse()
        zaPlan.plan = somePath
        #returns a getplan response with the appropriate path
        return zaPlan


#Checks for neighbors or a current point
    def neighbors(self, node):
        #stores all valid neighbors
        neighborMap =[]
        #current nodes x and y
        xCurr = float(node[0])
        yCurr = float(node[1])

        for x in range(1, -2, -2): # check for valid locations when x +1 and x-1
            for y in range(1, -2, -2):# check for valid locations when y +1 and y-1
                if (self.validLoc((xCurr+(x)), yCurr)) and\
                    not ((xCurr+(x)), yCurr) in neighborMap:
                    neighborMap.append(((xCurr+(x)), yCurr))
                if self.validLoc((xCurr+(x)), (yCurr + (y))) and\
                    not ((xCurr+(x)), (yCurr + (y))) in neighborMap:
                    neighborMap.append(((xCurr+(x)), (yCurr + (y))))
                if self.validLoc((xCurr), (yCurr + (y))) and\
                    not ((xCurr), (yCurr + (y))) in neighborMap:
                    neighborMap.append(((xCurr), (yCurr + (y))))
        return neighborMap


    #determines if a position is a valid one
    def validLoc(self, x, y):
        if x <= 0 or y <= 0:
            return False
        index = int(y * self.width + x) #this may need to be adjusted
        if self.map[index] == 0:
            return True
        else:
            return False

#Converts units in order to comminicate with map, needs to be fine tuned
    def worldToMap(self, point):
        newX=(point[0]*self.resolution)
        newY= (point[1]*self.resolution)+self.resolution
        return (newX,newY)

#Converts units in order to comminicate with world
    def mapToWorld(self, point):
        return ((int((point[0])/.3)),(int((point[1])/.3)))


    #finds path from the list of all visted nodes by tracing backwards
    def reconstruct_path(self, start, goal, came_from):
        """
            Rebuild the path from a dictionary
            :param start: starting key
            :param goal: starting value
            :param came_from: dictionary of tuples
            :return: list of tuples
        """
        #starts wityh goal and searches backwards till it reaches "none"
        pathSet = [goal]
        prevPoint = came_from[goal]
        while prevPoint != None:
            pathSet.append(prevPoint)
            prevPoint = came_from[prevPoint]
        return pathSet


#optomizes path so that movements in a straight path are simplified
    def optimize_path(self, path):#NEEEDS WORKS GOES IN INFINITE LOOP.

        """
               remove redundant points in hte path
               :param path: list of tuples
               :return: reduced list of tuples
        """
        # print 'in opt'
        # print "path", path
        length = len(path)
        #needs a few tweaks. infinite loops are no bueno
        for x in range(0, length):
            for point in path:
                pointIndex = path.index(point)
                flag = True # True means not searching yet
                nextIndex = pointIndex + 1
                lastIndex = nextIndex +1
                print"in for"

                while lastIndex+1 < length:
                    print "lastIndex", lastIndex
                    print "length", length
                    if float(point[0]) == float(path[nextIndex][0]):
                        if  point[0] == path[lastIndex][0]:
                            del path[nextIndex]
                            break

                    if float(point[1]) == float(path[nextIndex][1]):
                        if  point[1] == path[lastIndex][1]:
                            del path[nextIndex]
                            length = len(path)
                            break
                    length = len(path)
                    pointIndex = path.index(point)
                    flag = True # True means not searching yet
                    nextIndex = pointIndex + 1
                    lastIndex = nextIndex +1
        return path


#       [0 1 0 0 0]
#       [0 1 0 0 0]
#       [0 0 0 0 0]
#       [0 0 0 1 0]
#       [0 0 0 1 0]
#top to bottom
testMap = [0, 100, 0, 0, 0, 0, 100, 0, 0 ,0, 0, 0, 0, 0, 0, 0, 0, 0, 100, 0, 0 ,0, 0, 100, 0]

if __name__ == '__main__':

    print "runnnnnnnning"
    #create a star object
    star = A_Star()
    #let things spin up
    rospy.sleep(1)
    star.a_star_server()
    while not rospy.is_shutdown():
        pass
