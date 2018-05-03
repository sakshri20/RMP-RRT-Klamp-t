#!/usr/bin/python


## The functions are for creating additional components for the environment
##  1. getWall: Get the geometry of a wall based on given dimensions
##  2. getWall_terrain: Attach a single wall to terrain
##  3. getDoubleRoomDoor: Build two rooms with a door on the separating wall
##  4. getDoubleRoomWindow: Build two rooms with a window on the separating wall
import sys
from klampt import *
from klampt import vis
from klampt.robotsim import setRandomSeed
from klampt.vis.glcommon import GLWidgetPlugin
from klampt import RobotPoser
from klampt.model import ik,coordinates
from klampt.math import so3
import time
import math

def getWall(dimX, dimY, dimZ, pos = [0, 0, 0], rotZ = 0):
        ## Get the wall geometry
	wall = Geometry3D()
	wall.loadFile("cube.off") # The wall is based upon cube primitive of unit dimension
        ## Not sure why the scaling is done through the rotation matrix, works though!
	#wall.transform([dimX, 0, 0, 0, dimY, 0, 0, 0, dimZ], pos)
        wall.scale(dimX, dimY, dimZ)
        rotMat = so3.rotation((0, 0, 1), math.radians(rotZ))
        wall.transform(rotMat, pos)
        return wall

def getBlock(dimX, dimY, dimZ, pos = [0, 0, 0], rotZ = 0):
        ## Get the block geometry
	block = Geometry3D()
	block.loadFile("block.off") # The wall is based upon cube primitive of unit dimension
        ## Not sure why the scaling is done through the rotation matrix, works though!
	#wall.transform([dimX, 0, 0, 0, dimY, 0, 0, 0, dimZ], pos)
        block.scale(dimX, dimY, dimZ)
        rotMat = so3.rotation((1, 1, 1), math.radians(rotZ))
        block.transform(rotMat, pos)
        return block

def getWall_terrain(world, dimX, dimY, dimZ, pos = [0, 0, 0], nameWall="wall", color = [0.85, 0.85, 0.85, 1]):
        ## Attach a single wall to the world
        wall = getWall(dimX, dimY, dimZ, pos)
	world_wall = world.makeTerrain(nameWall)
	world_wall.geometry().set(wall)
        r = color[0]
        g = color[1]
        b = color[2]
        alpha = color[3]
	world_wall.appearance().setColor(r, g, b, alpha)
	return world_wall

def getRoom(world, dimX, dimY, dimZ, color = [0.85, 0.85, 0.85, 1], wall_thickness = 0.01):
    ## Build a double room with a single door in the middle of the wall
    ## The width of the door is dimX/4
    x2 = dimX/2.0
    x8 = dimX/8.0
    y2 = dimY/2.0
    w1 = getWall(dimX, wall_thickness, dimZ, [-x2, -y2, 0], 0)
    w2 = getWall(wall_thickness, dimY, dimZ, [-x2, -y2, 0], 0)
    w3 = getWall(dimX, wall_thickness, dimZ, [-x2, y2, 0], 0)
    w4 = getWall(wall_thickness, dimY, dimZ, [x2, -y2, 0], 0)
    x3 = dimX/16.0
    y3 = dimY/16.0
    #b11 = getWall(x3, wall_thickness, 0.5, [x3, 0, 0], 0)
    #b12 = getWall(wall_thickness, y3, 0.5, [2*x3, -y3, 0], 0)
    #b13 = getWall(x3, wall_thickness, 0.5, [x3, -y3, 0], 0)
    #b14 = getWall(wall_thickness, y3, 0.5, [x3, -y3, 0], 0)
    box1 = getBlock(0.2, 0.2, 4, [2*x3, y3, 0.25], 0)
    box2 = getBlock(0.1, 0.1, 4, [-(4*x3), y3, 0.25], 0)
    box3 = getBlock(0.15, 0.02, 4, [(4*x3), 4*y3, 0.25], 0)
    box4 = getBlock(0.02, 0.16, 4, [(4.6*x3), (3.45*y3), 0.25], 0)
    box5 = getBlock(0.02, 0.16, 4, [(4.6*x3), -(3.45*y3), 0.25], 0)
    box6 = getBlock(0.15, 0.02, 4, [(4*x3), -(4*y3), 0.25], 0)
    box7 = getBlock(0.02, 0.16, 4, [(3.4*x3), -(3.45*y3), 0.25], 0)
    box8 = getBlock(0.2, 0.2, 4, [-(3.4*x3), -(6*y3), 0.25], 0)
    #b21 = getWall(x3, wall_thickness, 0.5, [x3, 0, 0], 0)
    #b22 = getWall(wall_thickness, y3, 0.5, [2*x3, -y3, 0], 0)
    #b23 = getWall(x3, wall_thickness, 0.5, [x3, -y3, 0], 0)
    #b24 = getWall(wall_thickness, y3, 0.5, [x3, -y3, 0], 0)
    #w5 = getWall(3*x8, wall_thickness, dimZ, [-x2, 0, 0], 0)
    #w6 = getWall(3*x8, wall_thickness, dimZ, [x8, 0, 0], 0)
    DRDgeom = Geometry3D()
    DRDgeom.setGroup()
    for i,elem in enumerate([w1, w2, w3, w4,box1,box2,box3,box4,box5,box6,box7,box8]):
	g = Geometry3D(elem)
	DRDgeom.setElement(i,g)
    drd_setup = world.makeRigidObject("DRD")
    drd_setup.geometry().set(DRDgeom)
    r = color[0]
    g = color[1]
    b = color[2]
    alpha = color[3]
    drd_setup.appearance().setColor(r, g, b, alpha)

''' def getDoubleRoomDoor(world, dimX, dimY, dimZ, color = [0.85, 0.85, 0.85, 1], wall_thickness = 0.01):
    ## Build a double room with a single door in the middle of the wall
    ## The width of the door is dimX/4
    x2 = dimX/2.0
    x8 = dimX/8.0
    y2 = dimY/2.0
    w1 = getWall(dimX, wall_thickness, dimZ, [-x2, -y2, 0], 0)
    w2 = getWall(wall_thickness, dimY, dimZ, [-x2, -y2, 0], 0)
    w3 = getWall(dimX, wall_thickness, dimZ, [-x2, y2, 0], 0)
    w4 = getWall(wall_thickness, dimY, dimZ, [x2, -y2, 0], 0)
    w5 = getWall(3*x8, wall_thickness, dimZ, [-x2, 0, 0], 0)
    w6 = getWall(3*x8, wall_thickness, dimZ, [x8, 0, 0], 0)
    DRDgeom = Geometry3D()
    DRDgeom.setGroup()
    for i,elem in enumerate([w1, w2, w3, w4, w5, w6]):
	g = Geometry3D(elem)
	DRDgeom.setElement(i,g)
    drd_setup = world.makeRigidObject("DRD")
    drd_setup.geometry().set(DRDgeom)
    r = color[0]
    g = color[1]
    b = color[2]
    alpha = color[3]
    drd_setup.appearance().setColor(r, g, b, alpha)

def getDoubleRoomWindow(world, dimX, dimY, dimZ, color = [0.85, 0.85, 0.85, 1], wall_thickness = 0.01):
    ## Build a double room with a single window in the middle of the wall
    ## The dimensions of the window are dimX/4, dimZ/3  
    x2 = dimX/2.0
    x8 = dimX/8.0
    y2 = dimY/2.0
    z3 = dimZ/3.0
    w1 = getWall(dimX, wall_thickness, dimZ, [-x2, -y2, 0], 0)
    w2 = getWall(wall_thickness, dimY, dimZ, [-x2, -y2, 0], 0)
    w3 = getWall(dimX, wall_thickness, dimZ, [-x2, y2, 0], 0)
    w4 = getWall(wall_thickness, dimY, dimZ, [x2, -y2, 0], 0)
    w5 = getWall(dimX, wall_thickness, z3, [-x2, 0, 0], 0)
    w6 = getWall(3*x8, wall_thickness, z3, [-x2, 0, z3], 0)
    w7 = getWall(3*x8, wall_thickness, z3, [x8, 0, z3], 0)
    w8 = getWall(dimX, wall_thickness, z3, [-x2, 0, 2.0*z3], 0)
    DRDgeom = Geometry3D()
    DRDgeom.setGroup()
    for i,elem in enumerate([w1, w2, w3, w4, w5, w6, w7, w8]):
	g = Geometry3D(elem)
	DRDgeom.setElement(i,g)
    drd_setup = world.makeRigidObject("DRD")
    drd_setup.geometry().set(DRDgeom)
    r = color[0]
    g = color[1]
    b = color[2]
    alpha = color[3]
    drd_setup.appearance().setColor(r, g, b, alpha) '''
