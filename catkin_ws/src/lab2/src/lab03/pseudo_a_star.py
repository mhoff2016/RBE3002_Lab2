class A_star

    def __init__(self):

        # store incoming data
        self.all data coming in
        # store path data
        self.path data
        # publish gridcells
        self.publish grid data
        # publish path
        self.publish path

    def publish frontier(self, grid cells):
        publish frontier data
        sends grid cells to grid cell publish in __init__()

    def publish closed set(self, grid cells):
        publish closed set
        sends grid cells to grid cell publish in __init()
        publishes seperate from frontier

    def publish path (self, grid cells):
        for cells
            create list of cells for path
        publish path to publish path in __init__()

    def run a* (self, request)
        runs a*
        returns a* data


    def a* service(self):
        rospy.Service(service data)

    def map data(self, message):
        write variables for self

    def euclidean heuristic(a, b):

        return sqrt((xa-xb)^2 + (ya-yb)^2)

    def  a* (self, start, goal):
        assign variables from start
        assign variables from goal
        while the frontier isnt empty:
            pick next point in queue
            add it to the frontier
            if the current point is the goal
                break
            for neighbors of current point
                add to frontier
                determine cost with euclidean heuristic
                if we havent recorded cost
                    record cost
        make path
        publish path
        return path

    def neighbors(self, node):
        store valid neighbors
        for x from -1 to 1
            for y from -1 to 1
                if all neighbors are valid
                    return true
                else:
                    return false

    def valid location(self, x, y)
        if x or y is not 0
            return false
        if map index is 0
            return true
        else
            return false

    def world to map(self, point):
        convert units to communicate with map

    def map to world(self, point):
        convert units to communicate with world

    def remake path(self, start, goal, came from):
        while come from isnt none
            add points to list
        return path

    