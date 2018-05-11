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
from kdTree import RRTTree
from kdTree import node
from math import sqrt,cos,sin,atan2
import random
from klampt.model import coordinates
from klampt.model import trajectory
from priodict import priorityDictionary
from klampt.vis import gldraw


EPSILON = 0.1
DIM = 3
V1 = [1,-1,0]
V2 = [1,-1,0]
r = 0.1
dt = 0.01
u1 = .5
u2 = math.pi
actions = [[1,1],[1,-1],[1,0],[-1,1],[-1,-1],[-1,0],[0,1],[0,-1],[0,0]]

def RRTAlgorithm(source, goal, nodes):
    print source, goal
    robot.setConfig(source)

    #Step 1: initialise Kd Tree
    sourceNode = node(source, [], None, True, source[0], source[1], source[2], source[3], source[4], source[5])
    goalNode = node(goal, [], None, True, goal[0], goal[1], goal[2], goal[3], goal[4], goal[5])
    RRTree = RRTTree(None, None, 0, source, sourceNode) #actual RRTree
    Points = kdTree(None, None, 0, source, sourceNode) #for storing generated points to increase the search complexity
    currentNode = sourceNode
    iterNumber = 1
    path = []
    goalFlag = False
    while not (iterNumber == nodes):
        if check(currentNode.point, goalNode.point):
            goalFlag = True
            break 
        #Step 2: Generate random sample

        print ('iteration: ' + str(iterNumber))
        iterNumber = iterNumber + 1
        rand = [(-4) + (random.random() * 8), (-4) + (random.random() * 8), random.random() * 6.28319]
            
        #Step 3: Check for nearest neighbour
            
        ret = Points.search(rand, 10000000, None, None, None, None, None)
        nearest_neighbour = ret[1]
                
        #Step 4: Check distance of nearest neighbour with the randomly generated sample. If its EPSILON distance away then add it in the tree else              #generate a new point in the direction of the random sample
                
        new_point = next_point(nearest_neighbour, rand)

        if new_point != None:
            nde = node(new_point, [], ret[2], True, new_point[0], new_point[1], new_point[2], new_point[3], new_point[4], new_point[5])
            ret[2].add_child(nde)
            Points.insert(new_point, DIM, nde)
            currentNode = nde
    if goalFlag:
        print("Current Node: " + str(currentNode.parent.point))
        while currentNode.parent != None:
            path.append(currentNode)
            currentNode = currentNode.parent

        print("shortest path found", path)
        return path
    else:
        return None

def check(point , goal): # checking if currently added node is at goal or not
    d = dist(point, goal)
    if d < EPSILON:
        return True            	
    return False

def dist(p1, p2): #returns euclid's distance between points p1 and p2
    return sqrt((p1[0] - p2[0]) * (p1[0] - p2[0]) + (p1[1] - p2[1]) * (p1[1] - p2[1]))

def distance_matrice(p1, p2, dis, dtheta): #returns point with at most epsilon distance from nearest neighbour in the direction of randomly generated point

    theta = dtheta
    return [p1[0] + dis * cos(theta), p1[1] + dis * sin(theta), theta]

def next_point(initialPos, randomPoint):
    
    nn = initialPos
    new_point = []
    new_point_dist = []
    times = []
    collisionFlag = False
    distanceFlag = False

    for i in range(len(actions)):

        s = 0
        t = 0
        theta = 0
        old_distance = dist(nn, randomPoint)
        initialPos = nn

        while s <= EPSILON and t != 16: 

            vis.lock()
            robot.setConfig(initialPos)
            robot.velControlKin(actions[i][0]*u1, actions[i][1]*u2, dt)
            vis.unlock()

            if not checkCollision():
                collisionFlag = False
                t = t + 1
                n_p = robot.getConfig()
            else:
                collisionFlag = True
                break

            if old_distance <= dist(n_p, randomPoint):
                distanceFlag = True
                break
            else:
                distanceFlag = False
                old_distance = dist(n_p,randomPoint) 
                initialPos = n_p

        if t != 0:
            
            times.append(t)
            new_point.append(n_p)
            new_point_dist.append(dist(initialPos,randomPoint))
        else:
            break

    
    if not new_point or not new_point_dist:
        return None
    else:
        minInd = 0
        print("new_point_dist", new_point_dist)
        minVal = min(new_point_dist)
        print("minVal", minVal)
        for x in range(len(new_point_dist)):
            if minVal == new_point_dist[x]:
                minInd = x
        np = new_point[minInd]
        np.insert(3, times[minInd])
        np.insert(4, actions[minInd][0])
        np.insert(5, actions[minInd][1])
        
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
    goal = [3, -3.5, 0, 0 , 0, 0]
    sourceCollisionFlag = False
    goalCollisionFlag = False
    successFlag = False

    vis.lock()
    robot.setConfig(source)
    vis.unlock()
    

    if checkCollision():
        sourceCollisionFlag = True
    
    vis.lock()
    robot.setConfig(goal)
    
    vis.unlock()

    if checkCollision():
        goalCollisionFlag = True


    if not sourceCollisionFlag and not goalCollisionFlag:
        path = RRTAlgorithm(source, goal, 6000)
        if path != None:
            print("Completed")
            successFlag = True
        else:
            successFlag = False
            print("Path Not Found!")
    else:
        successFlag = False
        print("Source or Goal at collison!")

    if successFlag:
        waypoints = list(reversed(path))

        for i in waypoints:
            print(i.printNode())
            

        vis.lock()
        vis.add("Initial", [source[0], source[1], 0.02])
        vis.setAttribute("Initial", "size", 14)
        vis.setColor("Initial",0, 0.4470, 0.7410)
        vis.add("Goal", [goal[0], goal[1], 0.02])
        vis.setAttribute("Goal", "size", 14)
        vis.setColor("Goal", 0.8500, 0.3250, 0.0980)
        vis.unlock()

        

        vis.show()
        time.sleep(15)
        vis.lock()
        robot.setConfig(waypoints[0].point)
        vis.unlock()
        for i in range(len(waypoints)-1):
            prWaypnt = waypoints[i+1]
            
            for j in range(prWaypnt.t -1):
                vis.lock()
                robot.velControlKin(prWaypnt.u1*u1, prWaypnt.u2*u2, dt)
                vis.unlock()
                time.sleep(dt)    
    

    time.sleep(15)
    vis.clearText()

    print "Ending klampt.vis visualization."
    vis.kill()
