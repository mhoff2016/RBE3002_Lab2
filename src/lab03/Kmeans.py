from sklearn.cluster import KMeans
import rospy

map = None
width = None
height = None
neighborsSearched = []



def something(m, w, h):
    global map
    global width
    global height
    map = m
    width = w
    height = h
    frontierList = []
    #check entire map
    for i in range(0, len(map)):
        point = getXY(i, width)
        #if you find a valib point
        if map[i] == 0:
            #if the point has at least one neighbotr that is -1
            print neighborFrontier(point, map, width)
            if len(neighborFrontier(point, map, width)) > 0:
                #add point to a list
                #print "in if"
                frontierList.append(point)
                frontierList + (findSurroundings(point))
    return frontierList

def findSurroundings(point):
    global neighborsSearched
    resList = []
    for neighbor in neighbors(point):
        #if the neighbor has a neighbor that has -1
        if neighborFrontier(neighbor, map, width) and\
         not neighbor in neighborsSearched:
            #add neighbor to neaighbors searched
            neighborsSearched.append(neighbor)
            #add neighbor to relist
            resList.append(neighbor)
            resList + (findSurroundings(neighbor))
    return resList


    pass
#Checks for neighbors or a current point
def neighbors(node):
    #stores all valid neighbors
    neighborMap =[]
    #current nodes x and y
    xCurr = float(node[0])
    yCurr = float(node[1])

    for x in range(1, -2, -2): # check for valid locations when x +1 and x-1
        for y in range(1, -2, -2):# check for valid locations when y +1 and y-1
            if (validLoc((xCurr+(x)), yCurr)) and\
                not ((xCurr+(x)), yCurr) in neighborMap:
                neighborMap.append(((xCurr+(x)), yCurr))
            if validLoc((xCurr+(x)), (yCurr + (y))) and\
                not ((xCurr+(x)), (yCurr + (y))) in neighborMap:
                neighborMap.append(((xCurr+(x)), (yCurr + (y))))
            if validLoc((xCurr), (yCurr + (y))) and\
                not ((xCurr), (yCurr + (y))) in neighborMap:
                neighborMap.append(((xCurr), (yCurr + (y))))
    return neighborMap

#checks if a neighbor contains -1
def neighborFrontier(point, map, width):
    #stores all valid neighbors
    print "point:", point
    neighborMap =[]
    #current nodes x and y
    xCurr = point[0]
    yCurr = point[1]

    for x in range(1, -2, -2): # check for valid locations when x +1 and x-1
        for y in range(1, -2, -2):# check for valid locations when y +1 and y-1
            flagX = ((x + xCurr) <= width) or ((x + xCurr) >= 0) #equals 1 when in bounds
            flagY = ((y + yCurr) <= height) or ((y + yCurr) >= 0) #equals 1 when in bounds
            flag = flagX and flagY
            if flag:
                if (validFrontier((xCurr+(x)), yCurr)) and\
                    not ((xCurr+(x)), yCurr) in neighborMap:
                    #print "should append"
                    neighborMap.append(((xCurr+(x)), yCurr))
                if validFrontier((xCurr+(x)), (yCurr + (y))) and\
                    not ((xCurr+(x)), (yCurr + (y))) in neighborMap:
                    #print "should append"
                    neighborMap.append(((xCurr+(x)), (yCurr + (y))))
                if validFrontier((xCurr), (yCurr + (y))) and\
                    not ((xCurr), (yCurr + (y))) in neighborMap:
                    #print "should append"
                    neighborMap.append(((xCurr), (yCurr + (y))))
    #print "neighbormap", neighborMap
    return neighborMap

def validFrontier(x, y):
        #print"x", x
        #print"y", y
        #print "width:" , width
        #print "height:" , height
        #print "in validFrontier"
        if x <= 0 or y <= 0:
            #print "flase"
            return False

        # print"x:", x
        # print "y:",y
        # print index
        if x > width-1 or y > height-1:
            #print "false"
            return False
        else:
            index = int(y * width + x) #this may need to be adjusted
            #print "index",index
            if map[index] == -1:
                print "index",index
                #print"true"
                return True


#determines if a position is a valid one
def validLoc( x, y):
    if x <= 0 or y <= 0:
        return False
    index = int(y * width + x) #this may need to be adjusted
    if map[index] == 0:
        return True
    else:
        return False

#checks if a neighbors neighbor is also on the frontiers
def secondNeighbor():
    pass

#given an index, detemines the x and y of that gridcell
def getXY(index, width):
    row = int(index/width)
    column = int(index%width)
    return (column, row)

tester = (3, 4, 7, 1, 2, 9, 8, 2, 3)
tester2 = (0,0,0,-1,-1,0,0,0,0,-1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0)
width2 = 5
width = 3
if __name__ == '__main__':
    print("running kmeans,but not reaally")
    #tests
    print something(tester2, width2, 5)
    while not rospy.is_shutdown():
        pass
