#!/usr/bin/python

# Author: Sakshi Shrivastava
# Email: sshriva3@uncc.edu

## The file demonstrates:
    ##   1. Adding rooms and walls to the environment (refer to buildWorld.py as well)
    ##   2. Setting up a robot
    ##   3. Perform collision checking
    ##   4. Modify the robot configurations and visualize
    ##   5. Adding text objects and modifying them
import sys
from klampt import *
from klampt import vis
from klampt.robotsim import setRandomSeed
from klampt.vis.glcommon import GLWidgetPlugin
from klampt import RobotPoser
from klampt.model import ik,coordinates
from klampt.math import so3
import klampt.model.collide as collide
import time
import math
import buildWorld as bW
sys.path.append("./kinematics/")
from sphero6DoF import sphero6DoF
from kobuki import kobuki
from turtlebot import turtlebot
from decimal import Decimal
from kdTree import kdTree
from kdTree import node
from math import sqrt,cos,sin,atan2
import random
from klampt.model import coordinates
from klampt.model import trajectory
from priodict import priorityDictionary
from klampt.vis import gldraw


EPSILON = 0.2
DIM = 2
V1 = [1,-1,0]
V2 = [1,-1,0]
r = 0.1
dt = 0.01
u1 = 0.5
u2 = math.pi

def RRTAlgorithm(source, goal, nodes):
    print source, goal

    robot.setConfig(source)

    #Step 1: initialise Kd Tree
        
    RRTree = node(source, [], None, True) #actual RRTree
    Points = kdTree(None, None, 0, source, RRTree) #for storing generated points to increase the search complexity
    current = source
    iterNumber = 1

    while not check(current, goal) and not (iterNumber == nodes):
        #Step 2: Generate random sample

        #print ('iteration', iterNumber)
        iterNumber = iterNumber + 1
        rand = [(-4) + (random.random() * 8), (-4) + (random.random() * 8), random.random() * 6.28319]
            
        #Step 3: Check for nearest neighbour
            
        ret = Points.search(rand, 1000000, None, None, None, None, None)
        nearest_neighbour = ret[1]
                
        #Step 4: Check distance of nearest neighbour with the randomly generated sample. If its EPSILON distance away then add it in the tree else              #generate a new point in the direction of the random sample
                
        new_point = next_point(nearest_neighbour, rand)

        if new_point != None:
            nde = node(new_point, [], ret[2], True)
            ret[2].add_child(nde)
            Points.insert(new_point, DIM, nde)
            current = new_point
            pnt = [int(new_point[0]), int (new_point[1]), int (new_point[2])]

                
        #Step 5: collision check

        '''if not checkCollision(nearest_neighbour, new_point):
            nde = node(new_point, [], ret[2], True)
            ret[2].add_child(nde)
            Points.insert(new_point, DIM, nde)
            current = new_point
            pnt = [int(new_point[0]), int (new_point[1]), int (new_point[2])] '''

    Points.print_tree()
    tempPath = shortestPath(Points, goal, source)
    print("---------------------------------------------------------------")
    print("shortest path found", tempPath)
    return tempPath

def shortestPath(tree, start, end):
    
    path = []
    #currentNode = node(start, [], None, True)
    currentNode = start
    #path.append(currentNode)
    '''ret = tree.search(currentNode, 1000000, None, None, None, None, None)
    path.append(ret[2].point)
    currentNode = ret[2].point'''
    currentNode = [currentNode[0], currentNode[1], currentNode[2]]
    #print("node", currentNode)


    while not check(currentNode, end):

        #print("current: ", currentNode)
        #currentNode = [currentNode[0], currentNode[1], currentNode[2]]
        ret = tree.search(currentNode, 1000000, None, None, None, None, None)
        #print("NN", ret[2].point)
        point = [ret[2].point[0], ret[2].point[1], ret[2].point[2]]

        if check(point, currentNode):
            break
           
        else:
            path.append(ret[2].point)
            currentNode = ret[2].point
            currentNode = [currentNode[0], currentNode[1], currentNode[2]]
            #print("inside else")
    print("Hello")
    print("Path", path)
    return path

def check(point , goal): # checking if currently added node is at goal or not
    if point[0] > goal[0]-1 and point[0] < goal[0]+1 and point[1] > goal[1]-1 and point[1] < goal[1]+1:
        return True
                	
    return False

def dist(p1, p2): #returns euclid's distance between points p1 and p2
    return sqrt((p1[0] - p2[0]) * (p1[0] - p2[0]) + (p1[1] - p2[1]) * (p1[1] - p2[1]))

def distance_matrice(p1, p2, dis): #returns point with at most epsilon distance from nearest neighbour in the direction of randomly generated point

    theta = atan2(p2[1] - p1[1], p2[0] - p1[0])
    return [p1[0] + dis * cos(theta), p1[1] + dis * sin(theta), p2[2]]

def next_point(initialPos, randomPoint):
    U = [[1,1],[1,-1],[1,0],[-1,1],[-1,-1],[-1,0],[0,1],[0,-1],[0,0]]
    nn = initialPos
    new_point = []
    new_point_dist = []
    times = []

    for i in range(len(U)):

        s = 0
        t = 0
        old_distance = dist(nn, randomPoint)
        initialPos = nn
        while s <= EPSILON and t != 16: 

            vis.lock()
            robot.setConfig(initialPos)
            vis.unlock()
            time.sleep(0.01)

            s = s + dt*(U[i][0]*u1)
            
            robot.velControlKin(U[i][0]*u1, U[i][1]*u2, dt)
        
            if not checkCollision():
                collisionFlag = False
                t = t + 1
                n_p = distance_matrice(initialPos,randomPoint,s)
                 
            else:
                collisionFlag = True
                break

            if old_distance <= dist(n_p, randomPoint):
                break
            else:
                old_distance = dist(n_p,randomPoint) 
                initialPos = n_p

        #print("initialPos", initialPos)
        times.append(t)
        new_point.append(initialPos)
        new_point_dist.append(dist(initialPos,randomPoint))

    
    if not new_point or not new_point_dist:
        return None
    else:
        minInd = 0
        minVal = min(new_point_dist)
        for x in range(len(new_point_dist)):
            if minVal == new_point_dist[x]:
                minInd = x
        np = new_point[minInd]
        np.insert(3, times[minInd])
        np.insert(4, U[minInd][0])
        np.insert(5, U[minInd][1])
        
        return np
    

def checkCollision():
            
    collisionFlag = False
    collRT0 = collisionChecker.robotTerrainCollisions(world.robot(0), world.terrain(0))
    for i,j in collRT0:
        collisionFlag = True
        robot.velControlKin(0, 0, dt)
        break

    for iR in range(world.numRobots()):
        collRT2 = collisionChecker.robotObjectCollisions(world.robot(iR))
        for i,j in collRT2:
            collisionFlag = True
            robot.velControlKin(0, 0, dt)
            break
                        
    if not collisionFlag:
        return False
    else:
        return True


if __name__ == "__main__":
    if len(sys.argv)<=1:
        print "USAGE: kinematicSim.py [world_file]"
        exit()

    ## Creates a world and loads all the items on the command line
    world = WorldModel()
    for fn in sys.argv[1:]:
        res = world.readFile(fn)
        if not res:
            raise RuntimeError("Unable to load model "+fn)

    coordinates.setWorldModel(world)

        
    ## A rooms separated by a wall with a door 
    bW.getRoom(world, 8, 8, 1)

    ## Add the world to the visualizer
    vis.add("world",world)

    vp = vis.getViewport()
    vp.w,vp.h = 1200,800
    vis.setViewport(vp)

    ## Create robot object. Change the class to the desired robot. 
    ## Also, make sure the robot class corresponds to the robot in simpleWorld.xml file
    #robot = kobuki(world.robot(0), vis)
    #robot.setAltitude(0.01)

    robot = turtlebot(world.robot(0), "turtle", vis)
    robot.setAltitude(0.12)
    
    #robot = sphero6DoF(world.robot(0), "sphero", vis)

    ## Display the world coordinate system
    vis.add("WCS", [so3.identity(),[0,0,0]])
    vis.setAttribute("WCS", "size", 24)


    #print "Visualization items:"
    #vis.listItems(indent=2)

    #vis.autoFitCamera()
    vis.addText("textCol", "No collision")
    vis.setAttribute("textCol","size",24)
    collisionFlag = False
    collisionChecker = collide.WorldCollider(world)

    ## On-screen text display
    vis.addText("textConfig","Robot configuration: ")
    vis.setAttribute("textConfig","size",24)
    vis.addText("textbottom","WCS: X-axis Red, Y-axis Green, Z-axis Blue",(20,-30))

    print "Starting visualization window#..."

    ## Run the visualizer, which runs in a separate thread
    vis.setWindowTitle("Visualization for kinematic simulation")

    
    print("Starting.....")
    source = [0, 0, 0, 0, 0, 0]
    goal = [-4, -4, 0, 0 , 0, 0]

    '''vis.lock()
    robot.getConfig()
    vis.unlock()

    while s <= EPSILON and t != 16: 

            vis.lock()
            robot.setConfig(initialPos)
            vis.unlock()
            time.sleep(0.01)

            s = s + dt*(U[i][0]*u1)
            
            robot.velControlKin(U[i][0]*u1, U[i][1]*u2, dt)
        
            if not checkCollision():
                collisionFlag = False
                t = t + 1
                n_p = distance_matrice(initialPos,randomPoint,s)
                 
            else:
                collisionFlag = True
                break

            if old_distance <= dist(n_p, randomPoint):
                break
            else:
                old_distance = dist(n_p,randomPoint) 
                initialPos = n_p'''



    tree = RRTAlgorithm([0, 0, 0, 0, 0, 0], [-3, -3.5, 0, 0 , 0, 0], 2000)
    print("Completed")
    print(tree)

    waypoints = list(reversed(tree))
    if waypoints[0] != [0, 0, 0, 0, 0, 0]:
        waypoints.insert(0,[0, 0, 0, 0, 0, 0])

    print("Robot Config", waypoints)

    
    vis.show()
    for i in range(len(waypoints)-1):
        
        pt = waypoints[i+1]
        t = 0

        vis.lock()  
        robot.setConfig(waypoints[i])
        vis.unlock()
        time.sleep(0.01)
        
        while t != pt[3]:
            print(pt)
            robot.velControlKin(pt[4]*u1, pt[5]*u2, dt)
            t = t + 1
            
            

    simTime = 100
    startTime = time.time()
    oldTime = startTime
    while vis.shown() and (time.time() - startTime < simTime):
        vis.lock()

        q = robot.getConfig()
        q2f = [ '{0:.2f}'.format(elem) for elem in q]
        strng = "Robot configuration: " + str(q2f)
        vis.addText("textConfig", strng)


        vis.unlock()
        #changes to the visualization must be done outside the lock
        time.sleep(0.01)
    vis.clearText()

    print "Ending klampt.vis visualization."
    vis.kill()
