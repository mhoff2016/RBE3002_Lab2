#!/usr/bin/env python
import rospy
import sys
from queue import PriorityQueue
from nav_msgs.msg import GridCells, OccupancyGrid
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point
import math

class Node:
    x = None
    y = None
    f = None

class A_Star:

    def __init__(self):

        """
            This node handle A star paths requests.
            It is accessed using a service call. It can the publish grid cells
            to show the frontier,closed and path.
        """

        rospy.init_node("a_star")  # start node
        self.map = []
        self.width = None
        self.height = None
        self.resolution = None

        self.path = []
        self.cost = []

        #pub to gridcells
        self.pubClosedX = rospy.Publisher("/closed_set", GridCells, queue_size=40)

        self.pubFrontier = rospy.Publisher("/frontier_set", GridCells, queue_size=10)

        self.pubPath = rospy.Publisher("/path_set", GridCells, queue_size=10)

        #SUB TO map
        self.subMap = rospy.Subscriber("/map", OccupancyGrid, self.mapCallback)

    def publishClosed(self, start, goal):
            out = GridCells()
            width = 37
            height = 37
            point1 = Point()
            point2 = Point()
            point1.x = start[0]
            point2.x = goal[0]
            point1.y = start[1]
            point2.y = goal[1]
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

    def handle_a_star(self, req):

        """
            service call that uses A* to create a path.
            This can be altered to also grab the orentation for better heuristic
            :param req: GetPlan
            :return: Path()
        """
        print ("Returning path...")
        return  self.a_star(req.start, req.goal)


    def a_star_server(self):
        #rospy.init_node('a_star_path_server')
        s =  rospy.Service('a_star_path', GetPlan, self.handle_a_star)
        print "ready to star that A"
        rospy.spin()

    def mapCallback(self, msg):
        print "in inital"
        self.map = msg.data
        self.width = msg.info.width
        self.height = msg.info.height
        self.resolution = msg.info.resolution
        print "all good"
        print self.resolution




    def dynamic_map_client(self):

        """
            Service call to get map and set class variables
            This can be changed to call the expanded map
            :return:
        """
        pass

    def heuristic(a, b):
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
        startPoseX =  (int((startIn.pose.position.x)/.3))
        startPoseY =  (int((startIn.pose.position.y)/.3))
        start = (startPoseX, startPoseY)
        print "start:", start
        goalPoseX = (int((goalIn.pose.position.x)/.3))
        goalPoseY = (int((goalIn.pose.position.y)/.3))
        goal = (goalPoseX, goalPoseY)
        print "goal: ", goal
        self.publishClosed(start, goal)





        frontier = PriorityQueue()
        frontier.put(start, 0)
        came_from = {}
        cost_so_far = {}
        came_from[start] = None
        cost_so_far[start] = 0
        print "In A Star"
        while not frontier.empty():
            current = frontier.get()

            if current == goal:
                break

            for next in self.neighbors(current):
                #print "in astar for"
                new_cost = cost_so_far[current] + self.euclidean_heuristic(current, next) ##Should tthis just be euclidean???
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + self.euclidean_heuristic(goal, next)
                    frontier.put(next, priority)
                    came_from[next] = current
        self.path =  came_from
        print "cost_so_far:", came_from
        self.cost =  cost_so_far


    def neighbors(self, node):
        #print "in neighbors"
        neighborMap =[]
        xCurr = node[0]
        yCurr = node[1]
        #print type(xCurr)

        for x in range(1, -1, -1): # middle value may need to be -2
            for y in range(1, -1, -1):
                #print type(x)
                if self.validLoc((xCurr+x), yCurr):
                    neighborMap.append(((xCurr+x), yCurr))
                if self.validLoc((xCurr+x), (yCurr + y)):
                    neighborMap.append(((xCurr+x), (yCurr + y)))
                if self.validLoc((xCurr), (yCurr + y)):
                    neighborMap.append(((xCurr), (yCurr + y)))
        return neighborMap


    #determines if a position is a valid one
    def validLoc(self, x, y):
        #print "in ValidLoc"
        #print "x:", x
        #print "y:", y
        if x < 0 or y < 0:
            #print"returns false"
            return False
        index = int(y * self.width + x) #this may need to be adjusted
        if self.map[index] == 0:
            #print "returns true"
            return True
        else:
            return False










    def move_cost(self, current, next):
        """
              calculate the dist between two points
              :param current: tuple of location
              :param next: tuple of location
              :return: dist between two points
        """
        pass


    def reconstruct_path(self, start, goal, came_from):
        """
            Rebuild the path from a dictionary
            :param start: starting key
            :param goal: starting value
            :param came_from: dictionary of tuples
            :return: list of tuples
        """
        pass


    def optimize_path(self, path):
        """
            remove redundant points in hte path
            :param path: list of tuples
            :return: reduced list of tuples
        """
        pass

    def paint_cells(self, frontier, came_from):
        # type: (list, list) -> None
        """
            published cell of A* to Rviz
            :param frontier: tuples of the point on the frontier set
            :param came_from: tuples of the point on the closed set
            :return:, 0
        """
        pass


    def publish_path(self, points):
        """
            Create a Path() and publishes the cells if Paint is true
            :param points: list of tuples of the path
            :return: Path()
        """
        pass


#       [0 1 0 0 0]
#       [0 1 0 0 0]
#       [0 0 0 0 0]
#       [0 0 0 1 0]
#       [0 0 0 1 0]
#top to bottom
testMap = [0, 100, 0, 0, 0, 0, 100, 0, 0 ,0, 0, 0, 0, 0, 0, 0, 0, 0, 100, 0, 0 ,0, 0, 100, 0]

if __name__ == '__main__':
    print "runnnnnnnning"
    star = A_Star()
    rospy.sleep(1)
    star.a_star_server()
    while not rospy.is_shutdown():
        pass
