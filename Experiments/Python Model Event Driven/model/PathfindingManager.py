from cmath import pi
from turtle import forward
import pygame
import colours
import numpy as np
from logging import root
import pygame
import numpy as np
import colours
import sys

import random
import math
from math import degrees, atan2


import numpy.linalg as LA


import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import String

from model.Listener import Listener
from model.VectorPublisher import VectorPublisher
from model.FloatPublisher import FloatPublisher
from model.ColourPublisher import ColourPublisher
from model.SimulationNode import SimulationNode
from geometry_msgs.msg import Pose, Vector3


from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder


class PathfindingManager():

    def __init__(self, screen, cfg):
        print("pathfinding engage")


        self.pathFindingWidth = cfg['path_finding_width']
        self.pathFindingHeight = cfg['path_finding_height']
        self.worldWidth = cfg['world_width']
        self.worldHeight = cfg['world_height']
        self.pathFindingWorld_x = int(self.pathFindingWidth)
        self.pathFindingWorld_y = int(self.pathFindingHeight)
        self.pathFindingGridSquareWidth = self.worldWidth / self.pathFindingWidth
        self.pathFindingGridSquareHeight = self.worldHeight / self.pathFindingHeight
        self.screen = screen 
        self.worldMatrix = [0,1]

        

    def GenerateWorldMatrix(self, stationaryAgentsTuple):

        stationaryAgents = []
        for group in stationaryAgentsTuple:
            for agent in group:
                stationaryAgents.append(agent)

        
        np.set_printoptions(threshold=np.inf)

        print("generating world")

        # generate numpy matrix
        worldMatrix = np.ones((self.pathFindingWorld_y ,self.pathFindingWorld_x ), dtype=float)


        for x in range(self.pathFindingWorld_x):
            for y in range (self.pathFindingWorld_y):
                curX = x * self.pathFindingGridSquareWidth
                curY = y * self.pathFindingGridSquareHeight

                agentsInTile = 0

                # check if there is an agent in the current square
                for agent in stationaryAgents:
                    if(agent.position[0] > curX and agent.position[0] < curX+self.pathFindingGridSquareWidth  and agent.position[1] > curY and agent.position[1] < curY + self.pathFindingGridSquareHeight):
                        agentsInTile = 1

                # if so, add 1 to the numpy matrix
                if(agentsInTile > 0):
                    worldMatrix[y, x] -= 1

                    # add 1 to adjacent tiles
                    #if(x < 15):
                    #    worldMatrix[y, x+1] = 1
                    #if(x > 0):
                    #    worldMatrix[y, x-1] = 1
                    #if(y < 8):
                    #    worldMatrix[y+1, x] = 1
                    #if(y > 0):
                    #    worldMatrix[y-1, x] = 1
        print(worldMatrix)

        self.worldMatrix = worldMatrix

        return worldMatrix

    def UpdatePositionOnWorldMatrix(self, agent):
        print("UpdatePositionOnWorldMatrix")

        
    def FindPath(self, agent, targetPos):
        # convert agent position in screen space to pathfinding space

        agentPos = agent.position
        pathFindingPosX = int(agentPos[0] / self.pathFindingGridSquareWidth)
        pathFindingPosY = int(agentPos[1] / self.pathFindingGridSquareHeight)

        print("pathfinding coordinates ", pathFindingPosX, " ", pathFindingPosY)

        # convert target position to pathfinding space
        pathFindingTargetPosX = int(targetPos[0] / self.pathFindingGridSquareWidth)
        pathFindingTargetPosY = int(targetPos[1] / self.pathFindingGridSquareHeight)

        print("pathfinding target coordinates ", pathFindingTargetPosX, " ", pathFindingTargetPosY)
        
        lastTileValue = self.worldMatrix[pathFindingPosY, pathFindingPosX]
        # if agent shares tile with stationary:
        self.worldMatrix[pathFindingPosY, pathFindingPosX] = 1

        # calculate path
       

        grid = Grid(matrix=self.worldMatrix)

        startPos = grid.node(pathFindingPosX, pathFindingPosY)

        endPos = grid.node(pathFindingTargetPosX, pathFindingTargetPosY)

        finder = AStarFinder(diagonal_movement=DiagonalMovement.never)
        
        print(startPos)
        print(endPos)
        print(grid)
        path, runs = finder.find_path(startPos, endPos, grid)
        print("here")
        print('operations:', runs, 'path length:', len(path))
        print(grid.grid_str(path=path, start=startPos, end=endPos))

        print("path", path)

        

        # set current pathfinding position back to what it once was

        self.worldMatrix[pathFindingPosY, pathFindingPosX] = lastTileValue

        # return midpoints of screen space of each tile in the path
        screenCoordinatesPath = []

        for pathfindingCoordinates in path:
            x = pathfindingCoordinates[0] * self.pathFindingGridSquareWidth + self.pathFindingGridSquareWidth * 0.5
            y = pathfindingCoordinates[1] * self.pathFindingGridSquareHeight+ self.pathFindingGridSquareHeight * 0.5
            screenCoordinatesPath.append([x,y])


        screenCoordinatesPath.append([targetPos[0], targetPos[1]])
        print("screen path ", screenCoordinatesPath)

        

        return screenCoordinatesPath



